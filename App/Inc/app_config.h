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
#define APP_ENABLE_CS1238          1
#define APP_ENABLE_OLED            1
#define APP_ENABLE_BUTTON_TEST     1
#define APP_ENABLE_MAGNET_DETECT   1
#define APP_ENABLE_RAW_CAPTURE     1
#define APP_ENABLE_LED_HEARTBEAT   1
#define APP_ENABLE_SERIAL_BRIDGE   0
#define APP_ENABLE_ESP8266         0
#define APP_ENABLE_UART2           0
#define APP_ENABLE_UART3           0

#define APP_ENABLE_CS1238_DEBUG_PRINT  0
#define APP_ENABLE_MAGNET_DEBUG_PRINT  1

#endif /* APP_CONFIG_H */
