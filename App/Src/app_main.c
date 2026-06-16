/**
 * @file app_main.c
 * @brief Application entry layer for the source car firmware.
 *
 * This module keeps Core/Src/main.c clean by grouping application-level
 * initialization and non-blocking runtime tasks in one place.
 */
#include "app_main.h"

#include "app_config.h"
#if APP_ENABLE_CS1238
#include "app_cs1238_task.h"
#endif
#if APP_ENABLE_SERIAL_BRIDGE
#include "app_serial_bridge.h"
#endif
#if APP_ENABLE_ESP8266
#include "bsp_esp8266.h"
#endif
#if APP_ENABLE_LED_HEARTBEAT
#include "bsp_led.h"
#endif
#if APP_ENABLE_UART3
#include "bsp_uart3.h"
#endif
#include "main.h"

#define APP_HEARTBEAT_INTERVAL_MS 500U

/**
 * @brief Initialize application-level modules.
 *
 * This function is called once after board-level initialization is complete.
 * app_config.h selects which feature modules are started for the current
 * firmware bring-up target.
 */
void App_Init(void)
{
#if APP_ENABLE_LED_HEARTBEAT
    BSP_LED_Init();
#endif

#if APP_ENABLE_ESP8266
    BSP_ESP8266_Init();
#endif

#if APP_ENABLE_UART3
    BSP_UART3_Init(115200U);
#endif

    /*
     * TLV493D stays disabled here.
     * Do not call App_TLV493D_A1B6_DebugRunOnce().
     */

#if APP_ENABLE_SERIAL_BRIDGE
    AppSerialBridge_Init();
#endif

#if APP_ENABLE_CS1238
    App_CS1238_Init();
#endif
}

/**
 * @brief Run application-level tasks continuously.
 *
 * The loop remains non-blocking so enabled feature modules can share the HAL
 * system tick without blocking each other.
 */
void App_Loop(void)
{
#if APP_ENABLE_LED_HEARTBEAT
    static uint32_t last_heartbeat_tick = 0;
    uint32_t now = HAL_GetTick();
#endif

    /*
     * TLV493D stays disabled here.
     * Do not call App_TLV493D_A1B6_DebugLoopTick().
     */

#if APP_ENABLE_SERIAL_BRIDGE
    /* Run the UART bridge so pending bytes are forwarded continuously. */
    AppSerialBridge_Process();
#endif

#if APP_ENABLE_CS1238
    App_CS1238_Task();
#endif

#if APP_ENABLE_LED_HEARTBEAT
    /* Non-blocking LED heartbeat. HAL_GetTick() avoids blocking the main loop. */
    if ((uint32_t)(now - last_heartbeat_tick) >= APP_HEARTBEAT_INTERVAL_MS)
    {
        last_heartbeat_tick = now;
        BSP_LED_Toggle();
    }
#endif
}
