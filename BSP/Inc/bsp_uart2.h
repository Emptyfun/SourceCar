#ifndef BSP_UART2_H
#define BSP_UART2_H

/**
 * @file bsp_uart2.h
 * @brief Public interface for the external USART2 channel.
 */

#include "main.h"

/**
 * @brief Initialize USART2 for the external UART link.
 *
 * @param baudrate UART baud rate to configure.
 */
void BSP_UART2_Init(uint32_t baudrate);

/**
 * @brief Return the HAL handle for USART2.
 */
UART_HandleTypeDef *BSP_UART2_GetHandle(void);

#endif /* BSP_UART2_H */
