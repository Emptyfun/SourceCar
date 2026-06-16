#include "dev_cs1238.h"

#define CS1238_CFG_REFO_OFF_SHIFT 6U
#define CS1238_CFG_SPEED_SHIFT 4U
#define CS1238_CFG_PGA_SHIFT 2U
#define CS1238_CFG_CH_SHIFT 0U

CS1238_Handle hcs1238_1 = {GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1};
CS1238_Handle hcs1238_2 = {GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_6};

static void CS1238_ClockLow(CS1238_Handle *h);
static void CS1238_PulseClock(CS1238_Handle *h);
static void CS1238_ConfigDataInput(CS1238_Handle *h);
static void CS1238_ConfigDataOutput(CS1238_Handle *h);

void CS1238_InitAll(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

uint8_t CS1238_IsDataReady(CS1238_Handle *h)
{
    if (h == NULL)
    {
        return 0U;
    }

    CS1238_ClockLow(h);
    return (HAL_GPIO_ReadPin(h->data_port, h->data_pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

HAL_StatusTypeDef CS1238_ReadRaw(CS1238_Handle *h, int32_t *raw, uint32_t timeout_ms)
{
    uint32_t start;
    uint32_t dat = 0UL;
    uint8_t i;

    if ((h == NULL) || (raw == NULL))
    {
        return HAL_ERROR;
    }

    CS1238_ConfigDataInput(h);
    CS1238_ClockLow(h);

    start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(h->data_port, h->data_pin) != GPIO_PIN_RESET)
    {
        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms)
        {
            CS1238_ClockLow(h);
            return HAL_TIMEOUT;
        }
    }

    for (i = 0U; i < 24U; i++)
    {
        HAL_GPIO_WritePin(h->clk_port, h->clk_pin, GPIO_PIN_SET);
        dat <<= 1;
        if (HAL_GPIO_ReadPin(h->data_port, h->data_pin) == GPIO_PIN_SET)
        {
            dat |= 1UL;
        }
        HAL_GPIO_WritePin(h->clk_port, h->clk_pin, GPIO_PIN_RESET);
    }

    for (i = 0U; i < 3U; i++)
    {
        CS1238_PulseClock(h);
    }

    if (dat & 0x800000UL)
    {
        dat |= 0xFF000000UL;
    }
    *raw = (int32_t)dat;

    CS1238_ClockLow(h);
    return HAL_OK;
}

HAL_StatusTypeDef CS1238_WriteConfig(CS1238_Handle *h, uint8_t cfg)
{
    uint8_t i;

    if (h == NULL)
    {
        return HAL_ERROR;
    }

    CS1238_ClockLow(h);
    CS1238_ConfigDataOutput(h);

    for (i = 0U; i < 8U; i++)
    {
        HAL_GPIO_WritePin(h->data_port, h->data_pin,
                          ((cfg & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        CS1238_PulseClock(h);
        cfg <<= 1;
    }

    CS1238_ConfigDataInput(h);
    CS1238_ClockLow(h);
    return HAL_OK;
}

uint8_t CS1238_MakeConfig(uint8_t refo_off, uint8_t speed, uint8_t pga, uint8_t ch)
{
    return (uint8_t)(((refo_off & 0x01U) << CS1238_CFG_REFO_OFF_SHIFT) |
                     ((speed & 0x03U) << CS1238_CFG_SPEED_SHIFT) |
                     ((pga & 0x03U) << CS1238_CFG_PGA_SHIFT) |
                     ((ch & 0x03U) << CS1238_CFG_CH_SHIFT));
}

float CS1238_RawToVoltage(int32_t raw, float vref, uint8_t pga)
{
    float gain;

    switch (pga)
    {
    case 0U:
    case 1U:
        gain = (float)pga;
        break;

    case 64U:
        gain = 64.0f;
        break;

    case 128U:
        gain = 128.0f;
        break;

    default:
        gain = 1.0f;
        break;
    }

    if (gain < 1.0f)
    {
        gain = 1.0f;
    }

    return (float)raw * (0.5f * vref / gain) / 8388607.0f;
}

static void CS1238_ClockLow(CS1238_Handle *h)
{
    HAL_GPIO_WritePin(h->clk_port, h->clk_pin, GPIO_PIN_RESET);
}

static void CS1238_PulseClock(CS1238_Handle *h)
{
    HAL_GPIO_WritePin(h->clk_port, h->clk_pin, GPIO_PIN_SET);
    __NOP();
    __NOP();
    HAL_GPIO_WritePin(h->clk_port, h->clk_pin, GPIO_PIN_RESET);
}

static void CS1238_ConfigDataInput(CS1238_Handle *h)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = h->data_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(h->data_port, &GPIO_InitStruct);
}

static void CS1238_ConfigDataOutput(CS1238_Handle *h)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = h->data_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(h->data_port, &GPIO_InitStruct);
}
