#include "main.h"
#include "sx1276.h"

static DioIrqHandler *sx1276_dio0_irq;
extern SPI_HandleTypeDef hspi1;

void SX1276IoInit(void)
{
    // by CubeMX
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == RFM_IRQ_Pin) sx1276_dio0_irq();
}

void SX1276IoIrqInit(DioIrqHandler *irqHandler)
{
    // by CubeMX
    sx1276_dio0_irq = irqHandler;
}

void SX1276DelayMs(uint16_t ms)
{
    HAL_Delay(ms);
}

void SX1276GpioWriteNSS(uint8_t value)
{
    if (value) HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
    else HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET);
}

uint8_t SX1276SpiInOut(uint8_t outData)
{
    uint8_t inData;

    HAL_SPI_TransmitReceive(&hspi1, &outData, &inData, 1, HAL_MAX_DELAY);

    return inData;
}

