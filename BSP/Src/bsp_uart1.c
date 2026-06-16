/**
 * @file bsp_uart1.c
 * @brief USART1 board support for the PC-facing serial channel.
 *
 * USART1 is connected to the Type-C / CH340 interface and is used by the
 * transparent serial bridge for communication with the PC.
 */
#include "bsp_uart1.h"
#include "usart.h"

/**
 * @brief Initialize USART1 for 115200-8-N-1 communication.
 *
 * The HAL MSP layer configures USART1 clocks and PA9/PA10 pins. This function
 * configures the UART peripheral and enables its interrupt in the NVIC.
 */
void BSP_UART1_Init(void)
{
    MX_USART1_UART_Init();
}

/**
 * @brief Return the USART1 HAL handle used by App and ISR code.
 */
UART_HandleTypeDef *BSP_UART1_GetHandle(void)
{
    return &huart1;
}

/**
 * @brief Enable the USART1 interrupt line.
 */
void BSP_UART1_EnableIrq(void)
{
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}
