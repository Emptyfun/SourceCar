/**
 * @file bsp_uart3.c
 * @brief USART3 board support for the ESP8266 serial channel.
 *
 * USART3 uses PB10 as TX to ESP8266_RX and PB11 as RX from ESP8266_TX.
 */
#include "bsp_uart3.h"
#include "usart.h"

void BSP_UART3_Init(uint32_t baudrate)
{
    (void)baudrate;
    MX_USART3_UART_Init();
}

UART_HandleTypeDef *BSP_UART3_GetHandle(void)
{
    return &huart3;
}
