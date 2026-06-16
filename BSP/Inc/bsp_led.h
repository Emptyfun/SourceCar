#ifndef BSP_LED_H
#define BSP_LED_H

/**
 * @file bsp_led.h
 * @brief Public interface for the board status LED.
 */

#include "main.h"

/**
 * @brief Initialize the LED abstraction after GPIO setup is complete.
 */
void BSP_LED_Init(void);

/**
 * @brief Toggle the board status LED output.
 */
void BSP_LED_Toggle(void);

/**
 * @brief Drive the board status LED to the on state.
 */
void BSP_LED_On(void);

/**
 * @brief Drive the board status LED to the off state.
 */
void BSP_LED_Off(void);

#endif
