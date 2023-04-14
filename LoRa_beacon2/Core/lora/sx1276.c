/*!
 * \file      sx1276.c
 *
 * \brief     SX1276 driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Wael Guibene ( Semtech )
 */
#include <math.h>
#include <string.h>
#include "sx1276.h"

/*
 * Local types definition
 */

/*!
 * Radio registers definition
 */
typedef struct
{
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;


/*
 * Private functions prototypes
 */

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration( void );

/*!
 * \brief Sets the SX1276 in transmission mode for the given time
 */
void SX1276SetTx( void );

/*!
 * \brief Writes the buffer contents to the SX1276 FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void SX1276WriteFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the SX1276 FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void SX1276ReadFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the SX1276 operating mode
 *
 * \param [IN] opMode New operating mode
 */
void SX1276SetOpMode( uint8_t opMode );

/*
 * SX1276 DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void SX1276OnDio0Irq( void );

/*
 * Private global constants
 */

/*!
 * Radio hardware registers initialization
 *
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in sx1276-board.h file
 */
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*
 * Private global variables
 */

/*!
 * Radio callbacks variable
 */
static RadioEvents_t *RadioEvents;

/*!
 * Reception buffer
 */
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
SX1276_t SX1276;

/*
 * Radio driver functions implementation
 */

bool SX1276Init( RadioEvents_t *events )
{
    uint8_t i;

    RadioEvents = events;

    SX1276IoInit();

    // SX1276Reset( );

    RxChainCalibration( );

    SX1276SetOpMode( RF_OPMODE_SLEEP );

    SX1276IoIrqInit( SX1276OnDio0Irq );

    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1276Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }

    // SetModem( MODEM_LORA );
    SX1276SetOpMode( RF_OPMODE_SLEEP );
    SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );
    SX1276Write( REG_DIOMAPPING1, 0x00 );
    SX1276Write( REG_DIOMAPPING2, 0x00 );

    SX1276.Settings.State = RF_IDLE;

    return (SX1276Read(REG_LR_VERSION) == 0x12);
}

RadioState_t SX1276GetStatus( void )
{
    return SX1276.Settings.State;
}

void SX1276SetChannel( uint32_t freq )
{
    SX1276.Settings.Channel = freq;
    freq = (uint64_t)freq * FREQ_STEP_DENOM / FREQ_STEP_NUM;
    SX1276Write( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    SX1276Write( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    SX1276Write( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );
}

static void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | RF_PACONFIG_PASELECT_PABOOST;
    paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;

    if (power < 2) power = 2;
    if (power > 17) power = 17;
    paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( power - 2 );

    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration( void )
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = SX1276Read( REG_PACONFIG );
    initialFreq = (uint64_t)( ( ( uint32_t )SX1276Read( REG_FRFMSB ) << 16 ) |
                              ( ( uint32_t )SX1276Read( REG_FRFMID ) << 8 ) |
                              ( ( uint32_t )SX1276Read( REG_FRFLSB ) ) ) * FREQ_STEP_NUM / FREQ_STEP_DENOM;

    // Cut the PA just in case, RFO output, power = -1 dBm
    SX1276Write( REG_PACONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    SX1276Write( REG_IMAGECAL, ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    SX1276SetChannel( 868000000 );

    // Launch Rx chain calibration for HF band
    SX1276Write( REG_IMAGECAL, ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Restore context
    SX1276Write( REG_PACONFIG, regPaConfigInitVal );
    SX1276SetChannel( initialFreq );
}

void SX1276SetRxConfig( uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn,
                         bool iqInverted, bool rxContinuous )
{
    if( bandwidth > 2 )
    {
        // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
        while( 1 );
    }
    bandwidth += 7;
    SX1276.Settings.LoRa.Bandwidth = bandwidth;
    SX1276.Settings.LoRa.Datarate = datarate;
    SX1276.Settings.LoRa.Coderate = coderate;
    SX1276.Settings.LoRa.PreambleLen = preambleLen;
    SX1276.Settings.LoRa.FixLen = fixLen;
    SX1276.Settings.LoRa.PayloadLen = payloadLen;
    SX1276.Settings.LoRa.CrcOn = crcOn;
    SX1276.Settings.LoRa.IqInverted = iqInverted;
    SX1276.Settings.LoRa.RxContinuous = rxContinuous;

    if( datarate > 12 )
    {
        datarate = 12;
    }
    else if( datarate < 6 )
    {
        datarate = 6;
    }

    if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
    {
        SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
    }
    else
    {
        SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
    }

    SX1276Write( REG_LR_MODEMCONFIG1,
                 ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                   RFLR_MODEMCONFIG1_BW_MASK &
                   RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                   RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                   ( bandwidth << 4 ) | ( coderate << 1 ) |
                   fixLen );

    SX1276Write( REG_LR_MODEMCONFIG2,
                 ( SX1276Read( REG_LR_MODEMCONFIG2 ) &
                   RFLR_MODEMCONFIG2_SF_MASK &
                   RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                   RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                   ( datarate << 4 ) | ( crcOn << 2 ) |
                   ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

    SX1276Write( REG_LR_MODEMCONFIG3,
                 ( SX1276Read( REG_LR_MODEMCONFIG3 ) &
                   RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                   ( SX1276.Settings.LoRa.LowDatarateOptimize << 3 ) );

    SX1276Write( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

    SX1276Write( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
    SX1276Write( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

    if( fixLen == 1 )
    {
        SX1276Write( REG_LR_PAYLOADLENGTH, payloadLen );
    }

    if( ( bandwidth == 9 ) && ( SX1276.Settings.Channel > RF_MID_BAND_THRESH ) )
    {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        SX1276Write( REG_LR_HIGHBWOPTIMIZE1, 0x02 );
        SX1276Write( REG_LR_HIGHBWOPTIMIZE2, 0x64 );
    }
    else if( bandwidth == 9 )
    {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        SX1276Write( REG_LR_HIGHBWOPTIMIZE1, 0x02 );
        SX1276Write( REG_LR_HIGHBWOPTIMIZE2, 0x7F );
    }
    else
    {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        SX1276Write( REG_LR_HIGHBWOPTIMIZE1, 0x03 );
    }

    if( datarate == 6 )
    {
        SX1276Write( REG_LR_DETECTOPTIMIZE,
                     ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                       RFLR_DETECTIONOPTIMIZE_MASK ) |
                       RFLR_DETECTIONOPTIMIZE_SF6 );
        SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                     RFLR_DETECTIONTHRESH_SF6 );
    }
    else
    {
        SX1276Write( REG_LR_DETECTOPTIMIZE,
                     ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                     RFLR_DETECTIONOPTIMIZE_MASK ) |
                     RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
        SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                     RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
    }
}

void SX1276SetTxConfig( int8_t power,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn,
                        bool iqInverted )
{
    SX1276SetRfTxPower( power );

    SX1276.Settings.LoRa.Power = power;
    if( bandwidth > 2 )
    {
        // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
        while( 1 );
    }
    bandwidth += 7;
    SX1276.Settings.LoRa.Bandwidth = bandwidth;
    SX1276.Settings.LoRa.Datarate = datarate;
    SX1276.Settings.LoRa.Coderate = coderate;
    SX1276.Settings.LoRa.PreambleLen = preambleLen;
    SX1276.Settings.LoRa.FixLen = fixLen;
    SX1276.Settings.LoRa.CrcOn = crcOn;
    SX1276.Settings.LoRa.IqInverted = iqInverted;

    if( datarate > 12 )
    {
        datarate = 12;
    }
    else if( datarate < 6 )
    {
        datarate = 6;
    }
    if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
    {
        SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
    }
    else
    {
        SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
    }

    SX1276Write( REG_LR_MODEMCONFIG1,
                 ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                   RFLR_MODEMCONFIG1_BW_MASK &
                   RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                   RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                   ( bandwidth << 4 ) | ( coderate << 1 ) |
                   fixLen );

    SX1276Write( REG_LR_MODEMCONFIG2,
                 ( SX1276Read( REG_LR_MODEMCONFIG2 ) &
                   RFLR_MODEMCONFIG2_SF_MASK &
                   RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
                   ( datarate << 4 ) | ( crcOn << 2 ) );

    SX1276Write( REG_LR_MODEMCONFIG3,
                 ( SX1276Read( REG_LR_MODEMCONFIG3 ) &
                   RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                   ( SX1276.Settings.LoRa.LowDatarateOptimize << 3 ) );

    SX1276Write( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
    SX1276Write( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

    if( datarate == 6 )
    {
        SX1276Write( REG_LR_DETECTOPTIMIZE,
                     ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                       RFLR_DETECTIONOPTIMIZE_MASK ) |
                       RFLR_DETECTIONOPTIMIZE_SF6 );
        SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                     RFLR_DETECTIONTHRESH_SF6 );
    }
    else
    {
        SX1276Write( REG_LR_DETECTOPTIMIZE,
                     ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                     RFLR_DETECTIONOPTIMIZE_MASK ) |
                     RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
        SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                     RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
    }
}

void SX1276Send( uint8_t *buffer, uint8_t size )
{
    if( SX1276.Settings.LoRa.IqInverted == true )
    {
        SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
        SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
    }
    else
    {
        SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
        SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
    }

    SX1276.Settings.LoRaPacketHandler.Size = size;

    // Initializes the payload size
    SX1276Write( REG_LR_PAYLOADLENGTH, size );

    // Full buffer used for Tx
    SX1276Write( REG_LR_FIFOTXBASEADDR, 0 );
    SX1276Write( REG_LR_FIFOADDRPTR, 0 );

    // FIFO operations can not take place in Sleep mode
    if( ( SX1276Read( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
    {
        SX1276SetStby( );
        SX1276DelayMs( 1 );
    }
    // Write payload buffer
    SX1276WriteFifo( buffer, size );

    SX1276SetTx( );
}

void SX1276SetSleep( void )
{
    SX1276SetOpMode( RF_OPMODE_SLEEP );
    SX1276.Settings.State = RF_IDLE;
}

void SX1276SetStby( void )
{
    SX1276SetOpMode( RF_OPMODE_STANDBY );
    SX1276.Settings.State = RF_IDLE;
}

void SX1276SetRx( void )
{
    bool rxContinuous = false;

    if( SX1276.Settings.LoRa.IqInverted == true )
    {
        SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
        SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
    }
    else
    {
        SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
        SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
    }

    // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
    if( SX1276.Settings.LoRa.Bandwidth < 9 )
    {
        SX1276Write( REG_LR_DETECTOPTIMIZE, SX1276Read( REG_LR_DETECTOPTIMIZE ) & 0x7F );
        SX1276Write( REG_LR_IFFREQ2, 0x00 );
        switch( SX1276.Settings.LoRa.Bandwidth )
        {
        case 0: // 7.8 kHz
            SX1276Write( REG_LR_IFFREQ1, 0x48 );
            SX1276SetChannel(SX1276.Settings.Channel + 7810 );
            break;
        case 1: // 10.4 kHz
            SX1276Write( REG_LR_IFFREQ1, 0x44 );
            SX1276SetChannel(SX1276.Settings.Channel + 10420 );
            break;
        case 2: // 15.6 kHz
            SX1276Write( REG_LR_IFFREQ1, 0x44 );
            SX1276SetChannel(SX1276.Settings.Channel + 15620 );
            break;
        case 3: // 20.8 kHz
            SX1276Write( REG_LR_IFFREQ1, 0x44 );
            SX1276SetChannel(SX1276.Settings.Channel + 20830 );
            break;
        case 4: // 31.2 kHz
            SX1276Write( REG_LR_IFFREQ1, 0x44 );
            SX1276SetChannel(SX1276.Settings.Channel + 31250 );
            break;
        case 5: // 41.4 kHz
            SX1276Write( REG_LR_IFFREQ1, 0x44 );
            SX1276SetChannel(SX1276.Settings.Channel + 41670 );
            break;
        case 6: // 62.5 kHz
            SX1276Write( REG_LR_IFFREQ1, 0x40 );
            break;
        case 7: // 125 kHz
            SX1276Write( REG_LR_IFFREQ1, 0x40 );
            break;
        case 8: // 250 kHz
            SX1276Write( REG_LR_IFFREQ1, 0x40 );
            break;
        }
    }
    else
    {
        SX1276Write( REG_LR_DETECTOPTIMIZE, SX1276Read( REG_LR_DETECTOPTIMIZE ) | 0x80 );
    }

    rxContinuous = SX1276.Settings.LoRa.RxContinuous;

    SX1276Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                      //RFLR_IRQFLAGS_RXDONE |
                                      //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                      RFLR_IRQFLAGS_VALIDHEADER |
                                      RFLR_IRQFLAGS_TXDONE |
                                      RFLR_IRQFLAGS_CADDONE |
                                      RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                      RFLR_IRQFLAGS_CADDETECTED );

    // DIO0=RxDone
    SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );

    SX1276Write( REG_LR_FIFORXBASEADDR, 0 );
    SX1276Write( REG_LR_FIFOADDRPTR, 0 );

    memset( RxTxBuffer, 0, ( size_t )RX_BUFFER_SIZE );

    SX1276.Settings.State = RF_RX_RUNNING;

    if( rxContinuous == true )
    {
        SX1276SetOpMode( RFLR_OPMODE_RECEIVER );
    }
    else
    {
        SX1276SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
    }
}

void SX1276SetTx( void )
{
    SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                      RFLR_IRQFLAGS_RXDONE |
                                      RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                      RFLR_IRQFLAGS_VALIDHEADER |
                                      //RFLR_IRQFLAGS_TXDONE |
                                      RFLR_IRQFLAGS_CADDONE |
                                      RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                      RFLR_IRQFLAGS_CADDETECTED );

    // DIO0=TxDone
    SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );

    SX1276.Settings.State = RF_TX_RUNNING;
    SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
}

int16_t SX1276ReadRssi( void )
{
    int16_t rssi = 0;

    if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
    {
        rssi = RSSI_OFFSET_HF + SX1276Read( REG_LR_RSSIVALUE );
    }
    else
    {
        rssi = RSSI_OFFSET_LF + SX1276Read( REG_LR_RSSIVALUE );
    }

    return rssi;
}

void SX1276SetOpMode( uint8_t opMode )
{
    SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}

void SX1276Write( uint16_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}

uint8_t SX1276Read( uint16_t addr )
{
    uint8_t data;
    SX1276ReadBuffer( addr, &data, 1 );
    return data;
}

void SX1276WriteBuffer( uint16_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    SX1276GpioWriteNSS( 0 );

    SX1276SpiInOut( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        SX1276SpiInOut( buffer[i] );
    }

    SX1276GpioWriteNSS( 1 );
}

void SX1276ReadBuffer( uint16_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    SX1276GpioWriteNSS( 0 );

    SX1276SpiInOut( addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] = SX1276SpiInOut( 0 );
    }

    SX1276GpioWriteNSS( 1 );
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}

void SX1276SetPublicNetwork( bool enable )
{
    SX1276.Settings.LoRa.PublicNetwork = enable;
    if( enable == true )
    {
        // Change LoRa modem SyncWord
        SX1276Write( REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD );
    }
    else
    {
        // Change LoRa modem SyncWord
        SX1276Write( REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD );
    }
}

void SX1276OnDio0Irq( void )
{
    volatile uint8_t irqFlags = 0;

    switch( SX1276.Settings.State )
    {
        case RF_RX_RUNNING:
            // RxDone interrupt

            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

            irqFlags = SX1276Read( REG_LR_IRQFLAGS );
            if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
            {
                // Clear Irq
                SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

                if( SX1276.Settings.LoRa.RxContinuous == false )
                {
                    SX1276.Settings.State = RF_IDLE;
                }

                if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                {
                    RadioEvents->RxError( );
                }
                break;
            }

            // Returns SNR value [dB] rounded to the nearest integer value
            SX1276.Settings.LoRaPacketHandler.SnrValue = ( ( ( int8_t )SX1276Read( REG_LR_PKTSNRVALUE ) ) + 2 ) >> 2;

            int16_t rssi = SX1276Read( REG_LR_PKTRSSIVALUE );
            if( SX1276.Settings.LoRaPacketHandler.SnrValue < 0 )
            {
                if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                {
                    SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +
                                                                  SX1276.Settings.LoRaPacketHandler.SnrValue;
                }
                else
                {
                    SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +
                                                                  SX1276.Settings.LoRaPacketHandler.SnrValue;
                }
            }
            else
            {
                if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                {
                    SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                }
                else
                {
                    SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                }
            }

            SX1276.Settings.LoRaPacketHandler.Size = SX1276Read( REG_LR_RXNBBYTES );
            SX1276Write( REG_LR_FIFOADDRPTR, SX1276Read( REG_LR_FIFORXCURRENTADDR ) );
            SX1276ReadFifo( RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size );

            if( SX1276.Settings.LoRa.RxContinuous == false )
            {
                SX1276.Settings.State = RF_IDLE;
            }

            if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
            {
                RadioEvents->RxDone( RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size, SX1276.Settings.LoRaPacketHandler.RssiValue, SX1276.Settings.LoRaPacketHandler.SnrValue );
            }
            break;
        case RF_TX_RUNNING:
            // TxDone interrupt

            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
            // Intentional fall through
            SX1276.Settings.State = RF_IDLE;
            if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
            {
                RadioEvents->TxDone( );
            }
            break;
        default:
            break;
    }
}

