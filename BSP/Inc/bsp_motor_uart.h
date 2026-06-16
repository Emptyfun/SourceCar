#ifndef BSP_MOTOR_UART_H
#define BSP_MOTOR_UART_H

/**
 * @file bsp_motor_uart.h
 * @brief Public interface for the motor UART abstraction.
 */

#include "main.h"

/**
 * @brief Send a string through the motor UART interface.
 */
void BSP_MotorUart_SendString(const char *str);

#endif /* BSP_MOTOR_UART_H */
