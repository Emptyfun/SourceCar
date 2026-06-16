/**
 * @file bsp_uart2.c
 * @brief USART2 board support for the external UART channel.
 *
 * USART2 is the external serial side of the transparent bridge. It is paired
 * with USART1 so bytes can pass between the PC and the external UART device.
 */
#include "bsp_uart2.h"
#include "usart.h"

/**
 * @brief Initialize USART2 with the requested baud rate.
 *
 * This function configures PA2 as USART2_TX, PA3 as USART2_RX, initializes the
 * HAL UART handle, and enables the USART2 interrupt line.
 */
void BSP_UART2_Init(uint32_t baudrate)
{
    (void)baudrate;
    MX_USART2_UART_Init();
}

/**
 * @brief Return the USART2 HAL handle used by App and ISR code.
 */
UART_HandleTypeDef *BSP_UART2_GetHandle(void)
{
    return &huart2;
}
