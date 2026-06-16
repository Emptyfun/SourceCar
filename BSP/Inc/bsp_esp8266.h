#ifndef BSP_ESP8266_H
#define BSP_ESP8266_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

void BSP_ESP8266_Init(void);
void BSP_ESP8266_Enable(void);
void BSP_ESP8266_Disable(void);
uint8_t BSP_ESP8266_IsEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ESP8266_H */
