/**
 * @file bsp_soft_i2c_hmc.c
 * @brief PA0/PA1 bit-banged I2C bus for HMC5883L bring-up.
 */
#include "bsp_soft_i2c_hmc.h"

#define SOFT_I2C_HMC_DELAY_CYCLES 80U

static void SoftI2C_HMC_Delay(void);
static void SoftI2C_HMC_SCL_High(void);
static void SoftI2C_HMC_SCL_Low(void);
static void SoftI2C_HMC_SDA_High(void);
static void SoftI2C_HMC_SDA_Low(void);

void SoftI2C_HMC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(HMC_SCL_GPIO_PORT, HMC_SCL_GPIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HMC_SDA_GPIO_PORT, HMC_SDA_GPIO_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = HMC_SCL_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(HMC_SCL_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = HMC_SDA_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(HMC_SDA_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = HMC_DRDY_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HMC_DRDY_GPIO_PORT, &GPIO_InitStruct);

    SoftI2C_HMC_SCL_High();
    SoftI2C_HMC_SDA_High();
}

void SoftI2C_HMC_Start(void)
{
    SoftI2C_HMC_SDA_High();
    SoftI2C_HMC_SCL_High();
    SoftI2C_HMC_Delay();
    SoftI2C_HMC_SDA_Low();
    SoftI2C_HMC_Delay();
    SoftI2C_HMC_SCL_Low();
    SoftI2C_HMC_Delay();
}

void SoftI2C_HMC_Stop(void)
{
    SoftI2C_HMC_SDA_Low();
    SoftI2C_HMC_Delay();
    SoftI2C_HMC_SCL_High();
    SoftI2C_HMC_Delay();
    SoftI2C_HMC_SDA_High();
    SoftI2C_HMC_Delay();
}

uint8_t SoftI2C_HMC_WriteByte(uint8_t data)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0U; i < 8U; i++)
    {
        if ((data & 0x80U) != 0U)
        {
            SoftI2C_HMC_SDA_High();
        }
        else
        {
            SoftI2C_HMC_SDA_Low();
        }

        SoftI2C_HMC_Delay();
        SoftI2C_HMC_SCL_High();
        SoftI2C_HMC_Delay();
        SoftI2C_HMC_SCL_Low();
        SoftI2C_HMC_Delay();
        data <<= 1;
    }

    SoftI2C_HMC_SDA_High();
    SoftI2C_HMC_Delay();
    SoftI2C_HMC_SCL_High();
    SoftI2C_HMC_Delay();
    ack = (SoftI2C_HMC_ReadSDA() == 0U) ? 1U : 0U;
    SoftI2C_HMC_SCL_Low();
    SoftI2C_HMC_Delay();

    return ack;
}

uint8_t SoftI2C_HMC_ReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t data = 0U;

    SoftI2C_HMC_SDA_High();

    for (i = 0U; i < 8U; i++)
    {
        data <<= 1;
        SoftI2C_HMC_Delay();
        SoftI2C_HMC_SCL_High();
        SoftI2C_HMC_Delay();
        if (SoftI2C_HMC_ReadSDA() != 0U)
        {
            data |= 0x01U;
        }
        SoftI2C_HMC_SCL_Low();
        SoftI2C_HMC_Delay();
    }

    if (ack != 0U)
    {
        SoftI2C_HMC_SDA_Low();
    }
    else
    {
        SoftI2C_HMC_SDA_High();
    }

    SoftI2C_HMC_Delay();
    SoftI2C_HMC_SCL_High();
    SoftI2C_HMC_Delay();
    SoftI2C_HMC_SCL_Low();
    SoftI2C_HMC_SDA_High();
    SoftI2C_HMC_Delay();

    return data;
}

uint8_t SoftI2C_HMC_ReadSDA(void)
{
    return (HAL_GPIO_ReadPin(HMC_SDA_GPIO_PORT, HMC_SDA_GPIO_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

uint8_t SoftI2C_HMC_ReadDRDY(void)
{
    return (HAL_GPIO_ReadPin(HMC_DRDY_GPIO_PORT, HMC_DRDY_GPIO_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static void SoftI2C_HMC_Delay(void)
{
    volatile uint32_t i;

    for (i = 0U; i < SOFT_I2C_HMC_DELAY_CYCLES; i++)
    {
        __NOP();
    }
}

static void SoftI2C_HMC_SCL_High(void)
{
    HAL_GPIO_WritePin(HMC_SCL_GPIO_PORT, HMC_SCL_GPIO_PIN, GPIO_PIN_SET);
}

static void SoftI2C_HMC_SCL_Low(void)
{
    HAL_GPIO_WritePin(HMC_SCL_GPIO_PORT, HMC_SCL_GPIO_PIN, GPIO_PIN_RESET);
}

static void SoftI2C_HMC_SDA_High(void)
{
    HAL_GPIO_WritePin(HMC_SDA_GPIO_PORT, HMC_SDA_GPIO_PIN, GPIO_PIN_SET);
}

static void SoftI2C_HMC_SDA_Low(void)
{
    HAL_GPIO_WritePin(HMC_SDA_GPIO_PORT, HMC_SDA_GPIO_PIN, GPIO_PIN_RESET);
}
