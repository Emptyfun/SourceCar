/**
 * @file bsp_debug_uart.c
 * @brief Debug UART abstraction placeholder.
 *
 * This module reserves a BSP-facing debug UART interface without adding command
 * parsing or application behavior.
 */
#include "bsp_debug_uart.h"

#include "usart.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief Send a string through the debug UART interface.
 *
 * USART1 is the board debug console at 115200-8-N-1.
 */
void BSP_DebugUart_SendString(const char *str)
{
    size_t len;

    if (str == NULL)
    {
        return;
    }

    len = strlen(str);
    while (len > 0U)
    {
        uint16_t chunk = (len > 0xFFFFU) ? 0xFFFFU : (uint16_t)len;

        if (HAL_UART_Transmit(&huart1, (const uint8_t *)str, chunk, 50U) != HAL_OK)
        {
            return;
        }

        str += chunk;
        len -= chunk;
    }
}
