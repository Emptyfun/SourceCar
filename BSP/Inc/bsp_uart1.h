#ifndef BSP_UART1_H
#define BSP_UART1_H

/**
 * @file bsp_uart1.h
 * @brief Public interface for the PC-facing USART1 channel.
 */

#include "main.h"

/**
 * @brief Initialize USART1 for the Type-C / CH340 PC connection.
 */
void BSP_UART1_Init(void);

/**
 * @brief Return the HAL handle for USART1.
 */
UART_HandleTypeDef *BSP_UART1_GetHandle(void);

/**
 * @brief Enable the USART1 NVIC interrupt.
 */
void BSP_UART1_EnableIrq(void);

#endif /* BSP_UART1_H */
