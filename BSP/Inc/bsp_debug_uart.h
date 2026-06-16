#ifndef BSP_DEBUG_UART_H
#define BSP_DEBUG_UART_H

/**
 * @file bsp_debug_uart.h
 * @brief Public interface for the debug UART abstraction.
 */

#include "main.h"

/**
 * @brief Send a string through the debug UART interface.
 */
void BSP_DebugUart_SendString(const char *str);

#endif /* BSP_DEBUG_UART_H */
