#ifndef BSP_UART3_H
#define BSP_UART3_H

/**
 * @file bsp_uart3.h
 * @brief Public interface for the ESP8266 USART3 channel.
 */

#include "main.h"

/**
 * @brief Initialize USART3 for the ESP8266 UART link.
 *
 * @param baudrate UART baud rate to configure.
 */
void BSP_UART3_Init(uint32_t baudrate);

/**
 * @brief Return the HAL handle for USART3.
 */
UART_HandleTypeDef *BSP_UART3_GetHandle(void);

#endif /* BSP_UART3_H */
