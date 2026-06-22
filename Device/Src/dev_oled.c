#include "dev_oled.h"

#include "oledfont.h"

#include <string.h>

#define OLED_PAGE_COUNT (OLED_HEIGHT / 8U)

static uint8_t s_oled_gram[OLED_PAGE_COUNT][OLED_WIDTH];

static void OLED_GPIO_Init(void);
static void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
static uint8_t OLED_GetCharWidth(uint8_t size);
static uint8_t OLED_GetCharHeight(uint8_t size);
static uint32_t OLED_Pow(uint8_t base, uint8_t exponent);

void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    uint8_t i;

    if (cmd != 0U)
    {
        OLED_DC_Set();
    }
    else
    {
        OLED_DC_Clr();
    }

    OLED_CS_Clr();

    for (i = 0U; i < 8U; i++)
    {
        OLED_SCL_Clr();

        if ((dat & 0x80U) != 0U)
        {
            OLED_SDA_Set();
        }
        else
        {
            OLED_SDA_Clr();
        }

        OLED_SCL_Set();
        dat <<= 1U;
    }

    OLED_CS_Set();
    OLED_DC_Set();
}

void OLED_Init(void)
{
    OLED_GPIO_Init();

    HAL_Delay(20U);
    OLED_RES_Clr();
    HAL_Delay(20U);
    OLED_RES_Set();
    HAL_Delay(20U);

    OLED_WR_Byte(0xAEU, OLED_CMD);
    OLED_WR_Byte(0x00U, OLED_CMD);
    OLED_WR_Byte(0x10U, OLED_CMD);
    OLED_WR_Byte(0x40U, OLED_CMD);
    OLED_WR_Byte(0x81U, OLED_CMD);
    OLED_WR_Byte(0xCFU, OLED_CMD);
    OLED_WR_Byte(0xA1U, OLED_CMD);
    OLED_WR_Byte(0xC8U, OLED_CMD);
    OLED_WR_Byte(0xA6U, OLED_CMD);
    OLED_WR_Byte(0xA8U, OLED_CMD);
    OLED_WR_Byte(0x3FU, OLED_CMD);
    OLED_WR_Byte(0xD3U, OLED_CMD);
    OLED_WR_Byte(0x00U, OLED_CMD);
    OLED_WR_Byte(0xD5U, OLED_CMD);
    OLED_WR_Byte(0x80U, OLED_CMD);
    OLED_WR_Byte(0xD9U, OLED_CMD);
    OLED_WR_Byte(0xF1U, OLED_CMD);
    OLED_WR_Byte(0xDAU, OLED_CMD);
    OLED_WR_Byte(0x12U, OLED_CMD);
    OLED_WR_Byte(0xDBU, OLED_CMD);
    OLED_WR_Byte(0x40U, OLED_CMD);
    OLED_WR_Byte(0x20U, OLED_CMD);
    OLED_WR_Byte(0x02U, OLED_CMD);
    OLED_WR_Byte(0x8DU, OLED_CMD);
    OLED_WR_Byte(0x14U, OLED_CMD);
    OLED_WR_Byte(0xA4U, OLED_CMD);
    OLED_WR_Byte(0xA6U, OLED_CMD);
    OLED_WR_Byte(0xAFU, OLED_CMD);

    OLED_Clear();
    OLED_Refresh();
}

void OLED_Clear(void)
{
    memset(s_oled_gram, 0, sizeof(s_oled_gram));
}

void OLED_Refresh(void)
{
    uint8_t page;
    uint8_t x;

    for (page = 0U; page < OLED_PAGE_COUNT; page++)
    {
        OLED_WR_Byte((uint8_t)(0xB0U + page), OLED_CMD);
        OLED_WR_Byte(0x00U, OLED_CMD);
        OLED_WR_Byte(0x10U, OLED_CMD);

        for (x = 0U; x < OLED_WIDTH; x++)
        {
            OLED_WR_Byte(s_oled_gram[page][x], OLED_DATA);
        }
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{
    uint8_t glyph_index;
    uint8_t char_width = OLED_GetCharWidth(size);
    uint8_t char_height = OLED_GetCharHeight(size);
    uint8_t scale = (size >= OLED_FONT_10X14) ? 2U : 1U;
    uint8_t foreground = (mode != 0U) ? 1U : 0U;
    uint8_t background = (mode != 0U) ? 0U : 1U;
    uint8_t col;
    uint8_t row;
    uint8_t dx;
    uint8_t dy;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }

    if ((chr < ' ') || (chr > '~'))
    {
        chr = '?';
    }

    glyph_index = (uint8_t)(chr - ' ');

    for (col = 0U; col < char_width; col++)
    {
        for (row = 0U; row < char_height; row++)
        {
            OLED_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row), background);
        }
    }

    for (col = 0U; col < 5U; col++)
    {
        uint8_t glyph_column = OLED_F6x8[glyph_index][col];

        for (row = 0U; row < 7U; row++)
        {
            if ((glyph_column & (uint8_t)(1U << row)) == 0U)
            {
                continue;
            }

            for (dx = 0U; dx < scale; dx++)
            {
                for (dy = 0U; dy < scale; dy++)
                {
                    OLED_DrawPixel((uint8_t)(x + (col * scale) + dx),
                                   (uint8_t)(y + (row * scale) + dy),
                                   foreground);
                }
            }
        }
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *str, uint8_t size, uint8_t mode)
{
    uint8_t char_width = OLED_GetCharWidth(size);

    if (str == NULL)
    {
        return;
    }

    while (*str != '\0')
    {
        if (*str == '\n')
        {
            x = 0U;
            y = (uint8_t)(y + OLED_GetCharHeight(size));
            str++;
            continue;
        }

        if ((x >= OLED_WIDTH) || ((uint16_t)x + char_width > OLED_WIDTH))
        {
            break;
        }

        OLED_ShowChar(x, y, *str, size, mode);
        x = (uint8_t)(x + char_width);
        str++;
    }
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode)
{
    uint8_t i;
    uint8_t char_width = OLED_GetCharWidth(size);

    for (i = 0U; i < len; i++)
    {
        uint8_t digit = (uint8_t)((num / OLED_Pow(10U, (uint8_t)(len - i - 1U))) % 10U);
        OLED_ShowChar((uint8_t)(x + (i * char_width)), y, (uint8_t)('0' + digit), size, mode);
    }
}

static void OLED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    OLED_CS_Set();
    OLED_SCL_Set();
    OLED_SDA_Set();
    OLED_DC_Set();
    OLED_RES_Set();
}

static void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }

    if (color != 0U)
    {
        s_oled_gram[y / 8U][x] |= (uint8_t)(1U << (y % 8U));
    }
    else
    {
        s_oled_gram[y / 8U][x] &= (uint8_t)~(uint8_t)(1U << (y % 8U));
    }
}

static uint8_t OLED_GetCharWidth(uint8_t size)
{
    return (size >= OLED_FONT_10X14) ? 12U : 6U;
}

static uint8_t OLED_GetCharHeight(uint8_t size)
{
    return (size >= OLED_FONT_10X14) ? 16U : 8U;
}

static uint32_t OLED_Pow(uint8_t base, uint8_t exponent)
{
    uint32_t result = 1UL;

    while (exponent > 0U)
    {
        result *= base;
        exponent--;
    }

    return result;
}
