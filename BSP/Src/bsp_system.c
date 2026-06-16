/**
 * @file bsp_system.c
 * @brief Board-level startup sequence for the source car firmware.
 *
 * This module centralizes HAL and peripheral initialization so Core/Src/main.c
 * can stay focused on the high-level startup and main-loop flow.
 */
#include "bsp_system.h"

#include "bsp_clock.h"
#include "bsp_gpio.h"
#include "bsp_uart1.h"
#include "bsp_uart2.h"
#include "main.h"

/**
 * @brief Initialize the board support package.
 *
 * The BSP layer owns hardware setup. Clock, GPIO, and UART initialization are
 * kept in separate modules so each peripheral has a clear responsibility.
 * The App layer uses these BSP interfaces, while the Common layer contains
 * reusable utilities such as buffers that are not tied to one peripheral.
 */
void BSP_System_Init(void)
{
    HAL_Init();
    BSP_Clock_Init();
    BSP_GPIO_Init();
    BSP_UART1_Init();
    BSP_UART2_Init(115200U);
}
