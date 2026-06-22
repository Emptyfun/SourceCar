#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/**
 * @file app_config.h
 * @brief Application-wide compile-time configuration values.
 *
 * This header keeps firmware version constants in the App layer instead of
 * scattering them across BSP or Core files.
 */

#define APP_VERSION_MAJOR 0
#define APP_VERSION_MINOR 1
#define APP_VERSION_PATCH 0

/*
 * Temporary focus mode for CS1238 bring-up.
 *
 * Keep USART1 alive as the debug console, then avoid starting the previous
 * car-control UART router, ESP8266 path, and extra UARTs while ADC timing is
 * being verified.
 */
#define APP_ENABLE_CS1238          0
#define APP_ENABLE_OLED            1
#define APP_ENABLE_BUTTON_TEST     1
#define APP_ENABLE_MAGNET_DETECT   0
#define APP_ENABLE_RAW_CAPTURE     0
#define APP_ENABLE_LED_HEARTBEAT   1
#define APP_ENABLE_SERIAL_BRIDGE   0
#define APP_ENABLE_ESP8266         0
#define APP_ENABLE_UART2           1
#define APP_ENABLE_UART3           0

#define APP_ENABLE_MOTOR_SERIAL    1
#define APP_ENABLE_MOTION          1
#define APP_ENABLE_MOTOR_AUTO_TEST 0
#define APP_ENABLE_ODOMETRY        1
#define APP_ENABLE_MOTION_PRIMITIVE 1
#define APP_ENABLE_COVERAGE        0
#define APP_ENABLE_TURN_CALIB      1

#define APP_ENABLE_CS1238_DEBUG_PRINT  0
#define APP_ENABLE_MAGNET_DEBUG_PRINT  0
#define APP_MOTOR_SERIAL_RAW_MIRROR    0
#define APP_MOTOR_SERIAL_FEEDBACK_PRINT 0

#endif /* APP_CONFIG_H */
