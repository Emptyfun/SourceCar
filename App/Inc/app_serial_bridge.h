#ifndef APP_SERIAL_BRIDGE_H
#define APP_SERIAL_BRIDGE_H

/**
 * @file app_serial_bridge.h
 * @brief Public interface for the three-UART serial router.
 *
 * The router connects the PC-facing USART1 channel, car-controller USART2
 * channel, and ESP8266 USART3 channel according to the active route mode.
 */

/**
 * @brief Initialize router buffers and start interrupt-driven UART reception.
 */
void AppSerialBridge_Init(void);

/**
 * @brief Route pending bytes between UART queues and start transmissions.
 */
void AppSerialBridge_Process(void);
void AppSerialBridge_SendCarString(const char *s);
void AppSerialBridge_EmergencyStop(void);

#endif /* APP_SERIAL_BRIDGE_H */
