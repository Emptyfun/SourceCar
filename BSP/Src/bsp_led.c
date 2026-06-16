/**
 * @file bsp_led.c
 * @brief Board status LED control functions.
 *
 * The LED module hides the active-low PC13 wiring detail from the App layer.
 * Application code should call these BSP functions instead of writing the pin
 * directly.
 */
#include "bsp_led.h"

/**
 * @brief Initialize the board status LED abstraction.
 *
 * GPIO direction is configured by BSP_GPIO_Init(); this function only applies
 * the LED module's default off state.
 */
void BSP_LED_Init(void)
{
    /* PC13 has already been initialized in BSP_GPIO_Init(). */
    BSP_LED_Off();
}

/**
 * @brief Toggle the board status LED.
 */
void BSP_LED_Toggle(void)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/**
 * @brief Turn the board status LED on.
 */
void BSP_LED_On(void)
{
    /* PC13 LED is usually active-low. */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

/**
 * @brief Turn the board status LED off.
 */
void BSP_LED_Off(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}
