#include "dev_cs1238.h"

#define CS1238_CFG_REFO_OFF_SHIFT 6U
#define CS1238_CFG_SPEED_SHIFT 4U
#define CS1238_CFG_PGA_SHIFT 2U
#define CS1238_CFG_CH_SHIFT 0U

#define CS1238_CMD_WRITE_CONFIG 0x65U
#define CS1238_CONFIG_WRITE_TIMEOUT_MS 500U
#define CS1238_SCLK_DELAY_CYCLES 48U

CS1238_Handle hcs1238_1 = {GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15};
CS1238_Handle hcs1238_2 = {GPIOA, GPIO_PIN_11, GPIOB, GPIO_PIN_13};

static void CS1238_EnableGpioClock(GPIO_TypeDef *port);
static void CS1238_ClockLow(CS1238_Handle *h);
static void CS1238_PulseClock(CS1238_Handle *h);
static HAL_StatusTypeDef CS1238_WaitReady(CS1238_Handle *h, uint32_t timeout_ms);
static uint8_t CS1238_ReadBit(CS1238_Handle *h);
static void CS1238_WriteBit(CS1238_Handle *h, uint8_t bit);
static void CS1238_DelayHalfSclk(void);
static void CS1238_ConfigDataInput(CS1238_Handle *h);
static void CS1238_ConfigDataOutput(CS1238_Handle *h);

void CS1238_Init(CS1238_Handle *h)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (h == NULL)
    {
        return;
    }

    CS1238_EnableGpioClock(h->clk_port);
    CS1238_EnableGpioClock(h->data_port);

    HAL_GPIO_WritePin(h->clk_port, h->clk_pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = h->clk_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(h->clk_port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = h->data_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(h->data_port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(h->clk_port, h->clk_pin, GPIO_PIN_RESET);
}

void CS1238_InitAll(void)
{
    CS1238_Init(&hcs1238_1);
    CS1238_Init(&hcs1238_2);
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
    uint32_t dat = 0UL;
    uint8_t i;

    if ((h == NULL) || (raw == NULL))
    {
        return HAL_ERROR;
    }

    CS1238_ConfigDataInput(h);
    CS1238_ClockLow(h);

    if (CS1238_WaitReady(h, timeout_ms) != HAL_OK)
    {
        CS1238_ClockLow(h);
        return HAL_TIMEOUT;
    }

    for (i = 0U; i < 24U; i++)
    {
        dat <<= 1;
        if (CS1238_ReadBit(h) != 0U)
        {
            dat |= 1UL;
        }
    }

    /* Clocks 25/26 are status bits and clock 27 returns DRDY/DOUT high. */
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
    uint8_t command = CS1238_CMD_WRITE_CONFIG;

    if (h == NULL)
    {
        return HAL_ERROR;
    }

    cfg &= 0x7FU;
    CS1238_ConfigDataInput(h);
    CS1238_ClockLow(h);

    if (CS1238_WaitReady(h, CS1238_CONFIG_WRITE_TIMEOUT_MS) != HAL_OK)
    {
        CS1238_ClockLow(h);
        return HAL_TIMEOUT;
    }

    /* 1..24: drain the current ADC conversion before the config phase. */
    for (i = 0U; i < 24U; i++)
    {
        (void)CS1238_ReadBit(h);
    }

    /* 25..26: read write-status flags, 27: force DRDY/DOUT high. */
    (void)CS1238_ReadBit(h);
    (void)CS1238_ReadBit(h);
    CS1238_PulseClock(h);

    /*
     * 28..29 hand the bidirectional pin over to the MCU. Command bits are
     * sent MSB first on clocks 30..36.
     */
    CS1238_ConfigDataOutput(h);
    HAL_GPIO_WritePin(h->data_port, h->data_pin, GPIO_PIN_SET);
    CS1238_PulseClock(h);
    CS1238_PulseClock(h);

    for (i = 0U; i < 7U; i++)
    {
        CS1238_WriteBit(h, (uint8_t)((command >> (6U - i)) & 0x01U));
    }

    /* 37: direction turnaround. For write, the chip keeps DRDY/DOUT as input. */
    HAL_GPIO_WritePin(h->data_port, h->data_pin, GPIO_PIN_SET);
    CS1238_PulseClock(h);

    /* 38..45: config register data, MSB first. */
    for (i = 0U; i < 8U; i++)
    {
        CS1238_WriteBit(h, (uint8_t)((cfg >> (7U - i)) & 0x01U));
    }

    /* 46: latch the write and let DRDY/DOUT return to chip output mode. */
    HAL_GPIO_WritePin(h->data_port, h->data_pin, GPIO_PIN_SET);
    CS1238_PulseClock(h);
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

uint8_t CS1238_PgaGainFromSel(uint8_t pga)
{
    switch (pga)
    {
    case CS1238_PGA_SEL_2:
        return 2U;

    case CS1238_PGA_SEL_64:
        return 64U;

    case CS1238_PGA_SEL_128:
        return 128U;

    case CS1238_PGA_SEL_1:
    default:
        return 1U;
    }
}

float CS1238_RawToVoltage(int32_t raw, float vref, uint8_t pga)
{
    float gain;

    switch (pga)
    {
    case 1U:
        gain = 1.0f;
        break;

    case 2U:
        gain = 2.0f;
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

static void CS1238_EnableGpioClock(GPIO_TypeDef *port)
{
    if (port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (port == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
}

static void CS1238_PulseClock(CS1238_Handle *h)
{
    HAL_GPIO_WritePin(h->clk_port, h->clk_pin, GPIO_PIN_SET);
    CS1238_DelayHalfSclk();
    HAL_GPIO_WritePin(h->clk_port, h->clk_pin, GPIO_PIN_RESET);
    CS1238_DelayHalfSclk();
}

static HAL_StatusTypeDef CS1238_WaitReady(CS1238_Handle *h, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(h->data_port, h->data_pin) != GPIO_PIN_RESET)
    {
        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

static uint8_t CS1238_ReadBit(CS1238_Handle *h)
{
    uint8_t bit;

    HAL_GPIO_WritePin(h->clk_port, h->clk_pin, GPIO_PIN_SET);
    CS1238_DelayHalfSclk();
    bit = (HAL_GPIO_ReadPin(h->data_port, h->data_pin) == GPIO_PIN_SET) ? 1U : 0U;
    HAL_GPIO_WritePin(h->clk_port, h->clk_pin, GPIO_PIN_RESET);
    CS1238_DelayHalfSclk();

    return bit;
}

static void CS1238_WriteBit(CS1238_Handle *h, uint8_t bit)
{
    HAL_GPIO_WritePin(h->data_port,
                      h->data_pin,
                      (bit != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    CS1238_DelayHalfSclk();
    CS1238_PulseClock(h);
}

static void CS1238_DelayHalfSclk(void)
{
    volatile uint32_t i;

    for (i = 0U; i < CS1238_SCLK_DELAY_CYCLES; i++)
    {
        __NOP();
    }
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

    HAL_GPIO_WritePin(h->data_port, h->data_pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = h->data_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(h->data_port, &GPIO_InitStruct);
}
