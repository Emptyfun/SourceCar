/**
 * @file bsp_gpio.c
 * @brief GPIO initialization for board-level pins.
 *
 * GPIO setup is kept in the BSP layer so application code can use named BSP
 * services instead of directly owning pin configuration details.
 */
#include "bsp_gpio.h"

#include "main.h"

/**
 * @brief Initialize GPIO clocks and board pins.
 *
 * This function prepares PC13 as the status LED output. LED behavior is exposed
 * through bsp_led.c, while this module owns the raw pin direction and default
 * electrical state.
 */
void BSP_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PC13 is driven high by default so the active-low LED starts off. */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
