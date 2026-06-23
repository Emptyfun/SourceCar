/**
 * @file app_main.c
 * @brief Application entry layer for the source car firmware.
 *
 * This module keeps Core/Src/main.c clean by grouping application-level
 * initialization and non-blocking runtime tasks in one place.
 */
#include "app_main.h"

#include "app_config.h"
#if APP_ENABLE_HMC5883L
#include "app_hmc5883l.h"
#endif
#if APP_ENABLE_BUTTON_TEST
#include "app_button_test.h"
#endif
#if APP_ENABLE_CS1238
#include "app_cs1238_task.h"
#endif
#if APP_ENABLE_MAGNET_DETECT
#include "app_magnet_detect.h"
#endif
#if APP_ENABLE_MAG_CALIB
#include "app_mag_calib.h"
#endif
#if APP_ENABLE_MOTOR_SERIAL
#include "app_motor_serial.h"
#endif
#if APP_ENABLE_MOTOR_AUTO_TEST
#include "app_motor_auto_test.h"
#endif
#if APP_ENABLE_MOTION
#include "app_motion.h"
#endif
#if APP_ENABLE_MOTION_PRIMITIVE
#include "app_motion_primitive.h"
#endif
#if APP_ENABLE_COVERAGE
#include "app_coverage.h"
#endif
#if APP_ENABLE_TURN_CALIB
#include "app_turn_calib.h"
#endif
#if APP_ENABLE_TURN_DEBUG
#include "app_turn_debug.h"
#endif
#if APP_ENABLE_ODOMETRY
#include "app_odometry.h"
#endif
#if APP_ENABLE_OLED
#include "app_oled_ui.h"
#endif
#if APP_ENABLE_RAW_CAPTURE
#include "app_raw_capture.h"
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
#if APP_DEBUG_HMC5883L_ONLY
#if APP_ENABLE_LED_HEARTBEAT
    BSP_LED_Init();
#endif

    App_HMC5883L_Init();
#else
#if APP_ENABLE_LED_HEARTBEAT
    BSP_LED_Init();
#endif

#if APP_ENABLE_ESP8266
    BSP_ESP8266_Init();
#endif

#if APP_ENABLE_UART3
    BSP_UART3_Init(115200U);
#endif

#if APP_ENABLE_OLED
    App_OLED_Init();
#endif

#if APP_ENABLE_BUTTON_TEST
    App_ButtonTest_Init();
#endif

#if APP_ENABLE_SERIAL_BRIDGE
    AppSerialBridge_Init();
#endif

#if APP_ENABLE_CS1238
    App_CS1238_Init();
#endif

#if APP_ENABLE_MAGNET_DETECT
    App_MagDetect_Init();
#endif

#if APP_ENABLE_RAW_CAPTURE
    App_RawCapture_Init();
#endif

#if APP_ENABLE_ODOMETRY
    Odom_Init();
#endif

#if APP_ENABLE_MOTOR_SERIAL
    App_MotorSerial_Init();
#endif

#if APP_ENABLE_HMC5883L
    App_HMC5883L_Init();
#endif

#if APP_ENABLE_MOTION
    Motion_Init();
#endif

#if APP_ENABLE_MOTOR_AUTO_TEST
    App_MotorAutoTest_Init();
#endif

#if APP_ENABLE_MOTION_PRIMITIVE
    MotionPrimitive_Init();
#endif

#if APP_ENABLE_COVERAGE
    Coverage_Init();
#endif

#if APP_ENABLE_MAG_CALIB
    App_MagCalib_Init();
#endif

#if APP_ENABLE_TURN_CALIB
    App_TurnCalib_Init();
#endif

#if APP_ENABLE_TURN_DEBUG
    App_TurnDebug_Init();
#endif
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
#if APP_DEBUG_HMC5883L_ONLY
#if APP_ENABLE_LED_HEARTBEAT
    static uint32_t last_heartbeat_tick = 0;
    uint32_t now = HAL_GetTick();
#endif

    App_HMC5883L_Task();

#if APP_ENABLE_LED_HEARTBEAT
    if ((uint32_t)(now - last_heartbeat_tick) >= APP_HEARTBEAT_INTERVAL_MS)
    {
        last_heartbeat_tick = now;
        BSP_LED_Toggle();
    }
#endif
#else
#if APP_ENABLE_LED_HEARTBEAT
    static uint32_t last_heartbeat_tick = 0;
    uint32_t now = HAL_GetTick();
#endif

#if APP_ENABLE_SERIAL_BRIDGE
    /* Run the UART bridge so pending bytes are forwarded continuously. */
    AppSerialBridge_Process();
#endif

#if APP_ENABLE_CS1238
    App_CS1238_Task();
#endif

#if APP_ENABLE_BUTTON_TEST
    App_ButtonTest_Task();
#endif

#if APP_ENABLE_MAGNET_DETECT
    App_MagDetect_Task();
#endif

#if APP_ENABLE_RAW_CAPTURE
    App_RawCapture_Task();
#endif

#if APP_ENABLE_MOTOR_SERIAL
    App_MotorSerial_Task();
#endif

#if APP_ENABLE_HMC5883L
    App_HMC5883L_Task();
#endif

#if APP_ENABLE_MOTOR_AUTO_TEST
    App_MotorAutoTest_Task();
#endif

#if APP_ENABLE_MOTION_PRIMITIVE
    MotionPrimitive_Task();
#endif

#if APP_ENABLE_COVERAGE
    Coverage_Task();
#endif

#if APP_ENABLE_MAG_CALIB
    App_MagCalib_Task();
#endif

#if APP_ENABLE_TURN_CALIB
    App_TurnCalib_Task();
#endif

#if APP_ENABLE_TURN_DEBUG
    App_TurnDebug_Task();
#endif

#if APP_ENABLE_MOTION
    Motion_Task();
#endif

#if APP_ENABLE_OLED
    App_OLED_Task();
#endif

#if APP_ENABLE_LED_HEARTBEAT
    /* Non-blocking LED heartbeat. HAL_GetTick() avoids blocking the main loop. */
    if ((uint32_t)(now - last_heartbeat_tick) >= APP_HEARTBEAT_INTERVAL_MS)
    {
        last_heartbeat_tick = now;
        BSP_LED_Toggle();
    }
#endif
#endif
}
