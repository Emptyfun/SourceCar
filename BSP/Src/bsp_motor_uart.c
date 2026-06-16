/**
 * @file bsp_motor_uart.c
 * @brief Motor UART abstraction placeholder.
 *
 * This module reserves a BSP-facing UART interface for motor communication
 * without adding motor control logic.
 */
#include "bsp_motor_uart.h"

/**
 * @brief Send a string through the motor UART interface.
 *
 * The current implementation is a placeholder and intentionally does not touch
 * hardware.
 */
void BSP_MotorUart_SendString(const char *str)
{
    (void)str;
}
