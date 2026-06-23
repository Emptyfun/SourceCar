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

#ifndef APP_DEBUG_HMC5883L_ONLY
#define APP_DEBUG_HMC5883L_ONLY 0
#endif

#ifndef APP_DEBUG_TURN_OPTIMIZE_ONLY
#define APP_DEBUG_TURN_OPTIMIZE_ONLY 0
#endif

/*
 * Temporary focus mode for HMC5883L bring-up.
 *
 * Keep USART1 alive as the debug console, then run only the PA0/PA1 software
 * I2C HMC5883L task plus the optional PC13 heartbeat.
 */
#if APP_DEBUG_HMC5883L_ONLY
#define APP_ENABLE_HMC5883L      1
#define APP_ENABLE_HMC5883L_TURN_CLOSED_LOOP 0
#define APP_ENABLE_HMC5883L_STRAIGHT_HOLD 0
#define APP_HMC5883L_VERBOSE_PRINT 1

#define APP_ENABLE_CS1238          0
#define APP_ENABLE_OLED            0
#define APP_ENABLE_BUTTON_TEST     0
#define APP_ENABLE_KEY             0
#define APP_ENABLE_MAGNET_DETECT   0
#define APP_ENABLE_MAG_DETECT      0
#define APP_ENABLE_RAW_CAPTURE     0
#define APP_ENABLE_LED_HEARTBEAT   1
#define APP_ENABLE_SERIAL_BRIDGE   0
#define APP_ENABLE_ESP8266         0
#define APP_ENABLE_UART2           0
#define APP_ENABLE_UART3           0

#define APP_ENABLE_MOTOR_SERIAL    0
#define APP_ENABLE_MOTOR_USART     0
#define APP_ENABLE_MOTION          0
#define APP_ENABLE_MOTOR_AUTO_TEST 0
#define APP_ENABLE_ODOMETRY        0
#define APP_ENABLE_MOTION_PRIMITIVE 0
#define APP_ENABLE_COVERAGE        0
#define APP_ENABLE_TURN_CALIB      0
#define APP_ENABLE_MAG_CALIB       0
#define APP_ENABLE_TURN_DEBUG      0
#elif APP_DEBUG_TURN_OPTIMIZE_ONLY
/*
 * Dedicated in-place turn tuning firmware.
 *
 * Keep only the PC console, motor UART, calibrated HMC5883L heading, Motion
 * speed output, the turn debug task, and the optional PC13 heartbeat.
 */
#define APP_ENABLE_HMC5883L      1
#define APP_ENABLE_HMC5883L_TURN_CLOSED_LOOP 0
#define APP_ENABLE_HMC5883L_STRAIGHT_HOLD 0
#define APP_HMC5883L_VERBOSE_PRINT 0

#define APP_ENABLE_CS1238          0
#define APP_ENABLE_OLED            0
#define APP_ENABLE_BUTTON_TEST     0
#define APP_ENABLE_KEY             0
#define APP_ENABLE_MAGNET_DETECT   0
#define APP_ENABLE_MAG_DETECT      0
#define APP_ENABLE_RAW_CAPTURE     0
#define APP_ENABLE_LED_HEARTBEAT   1
#define APP_ENABLE_SERIAL_BRIDGE   0
#define APP_ENABLE_ESP8266         0
#define APP_ENABLE_UART2           1
#define APP_ENABLE_UART3           0

#define APP_ENABLE_MOTOR_SERIAL    1
#define APP_ENABLE_MOTOR_USART     1
#define APP_ENABLE_MOTION          1
#define APP_ENABLE_MOTOR_AUTO_TEST 0
#define APP_ENABLE_ODOMETRY        0
#define APP_ENABLE_MOTION_PRIMITIVE 0
#define APP_ENABLE_COVERAGE        0
#define APP_ENABLE_TURN_CALIB      0
#define APP_ENABLE_MAG_CALIB       0
#define APP_ENABLE_TURN_DEBUG      1
#else
/*
 * Coverage mode with calibrated HMC5883L heading feedback.
 *
 * USART1 is the PC debug/control console, USART2 is the motor controller, and
 * PA0/PA1 are dedicated to the HMC5883L software-I2C bus.
 */
#define APP_ENABLE_HMC5883L      1
#define APP_ENABLE_HMC5883L_TURN_CLOSED_LOOP 1
#define APP_ENABLE_HMC5883L_STRAIGHT_HOLD 0
#define APP_HMC5883L_VERBOSE_PRINT 0

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
#define APP_ENABLE_COVERAGE        1
#define APP_ENABLE_TURN_CALIB      0
#define APP_ENABLE_MAG_CALIB       0
#define APP_ENABLE_TURN_DEBUG      0
#endif

#ifndef APP_ENABLE_MAG_CALIB
#define APP_ENABLE_MAG_CALIB       0
#endif

#ifndef APP_ENABLE_TURN_DEBUG
#define APP_ENABLE_TURN_DEBUG      0
#endif

#ifndef APP_ENABLE_KEY
#define APP_ENABLE_KEY             APP_ENABLE_BUTTON_TEST
#endif

#ifndef APP_ENABLE_MAG_DETECT
#define APP_ENABLE_MAG_DETECT      APP_ENABLE_MAGNET_DETECT
#endif

#ifndef APP_ENABLE_MOTOR_USART
#define APP_ENABLE_MOTOR_USART     APP_ENABLE_MOTOR_SERIAL
#endif

#define APP_ESP8266_AUTO_CONNECT   1
#define APP_SERIAL_BOOT_PC_ESP_DEBUG 0
#define APP_ESP8266_WIFI_SSID      "ACT_2202"
#define APP_ESP8266_WIFI_PASSWORD  "ACT_2202"
#define APP_ESP8266_TCP_HOST       "192.168.31.175"
#define APP_ESP8266_TCP_PORT       8080

#define APP_ENABLE_CS1238_DEBUG_PRINT  0
#define APP_ENABLE_MAGNET_DEBUG_PRINT  0
#define APP_MOTOR_SERIAL_RAW_MIRROR    0
#define APP_MOTOR_SERIAL_FEEDBACK_PRINT 0

#endif /* APP_CONFIG_H */
