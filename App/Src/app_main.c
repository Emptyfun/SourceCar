/**
 * @file app_main.c
 * @brief Application entry layer for the source car firmware.
 *
 * This module keeps Core/Src/main.c clean by grouping application-level
 * initialization and non-blocking runtime tasks in one place.
 */
#include "app_main.h"

#include "app_cs1238_task.h"
#include "app_serial_bridge.h"
#include "app_tlv493d_a1b6_debug.h"
#include "bsp_esp8266.h"
#include "bsp_led.h"
#include "bsp_uart3.h"
#include "main.h"

#define APP_HEARTBEAT_INTERVAL_MS 500U

/**
 * @brief Initialize application-level modules.
 *
 * This function is called once after board-level initialization is complete.
 * It prepares the status LED abstraction and starts the transparent UART bridge.
 */
void App_Init(void)
{
    BSP_LED_Init();
    BSP_ESP8266_Init();
    BSP_UART3_Init(115200U);

    /*
     * TLV493D stays disabled here.
     * Do not call App_TLV493D_A1B6_DebugRunOnce().
     */
    AppSerialBridge_Init();
    App_CS1238_Init();
}

/**
 * @brief Run application-level tasks continuously.
 *
 * The loop remains non-blocking so the UART bridge can keep forwarding bytes
 * while the LED heartbeat is timed from the HAL system tick.
 */
void App_Loop(void)
{
    static uint32_t last_heartbeat_tick = 0;
    uint32_t now = HAL_GetTick();

    /*
     * TLV493D stays disabled here.
     * Do not call App_TLV493D_A1B6_DebugLoopTick().
     */

    /* Run the UART bridge so pending bytes are forwarded continuously. */
    AppSerialBridge_Process();
    App_CS1238_Task();

    /* Non-blocking LED heartbeat. HAL_GetTick() avoids blocking the main loop. */
    if ((uint32_t)(now - last_heartbeat_tick) >= APP_HEARTBEAT_INTERVAL_MS)
    {
        last_heartbeat_tick = now;
        BSP_LED_Toggle();
    }
}
