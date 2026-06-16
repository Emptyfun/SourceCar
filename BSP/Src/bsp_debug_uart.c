/**
 * @file bsp_debug_uart.c
 * @brief Debug UART abstraction placeholder.
 *
 * This module reserves a BSP-facing debug UART interface without adding command
 * parsing or application behavior.
 */
#include "bsp_debug_uart.h"

/**
 * @brief Send a string through the debug UART interface.
 *
 * The current implementation is a placeholder and intentionally does not touch
 * hardware.
 */
void BSP_DebugUart_SendString(const char *str)
{
    (void)str;
}
