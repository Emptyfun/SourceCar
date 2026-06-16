/**
 * @file bsp_esp8266.c
 * @brief ESP8266 enable-pin control.
 */
#include "bsp_esp8266.h"

#define ESP8266_EN_GPIO_PORT GPIOB
#define ESP8266_EN_GPIO_PIN  GPIO_PIN_12
#define ESP8266_ENABLE_LEVEL GPIO_PIN_RESET
#define ESP8266_DISABLE_LEVEL GPIO_PIN_SET

static uint8_t g_esp8266_enabled;

void BSP_ESP8266_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(ESP8266_EN_GPIO_PORT, ESP8266_EN_GPIO_PIN, ESP8266_DISABLE_LEVEL);

    GPIO_InitStruct.Pin = ESP8266_EN_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ESP8266_EN_GPIO_PORT, &GPIO_InitStruct);

    g_esp8266_enabled = 0U;
}

void BSP_ESP8266_Enable(void)
{
    HAL_GPIO_WritePin(ESP8266_EN_GPIO_PORT, ESP8266_EN_GPIO_PIN, ESP8266_ENABLE_LEVEL);
    g_esp8266_enabled = 1U;
}

void BSP_ESP8266_Disable(void)
{
    HAL_GPIO_WritePin(ESP8266_EN_GPIO_PORT, ESP8266_EN_GPIO_PIN, ESP8266_DISABLE_LEVEL);
    g_esp8266_enabled = 0U;
}

uint8_t BSP_ESP8266_IsEnabled(void)
{
    return g_esp8266_enabled;
}
