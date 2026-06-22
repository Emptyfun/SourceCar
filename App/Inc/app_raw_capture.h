#ifndef APP_RAW_CAPTURE_H
#define APP_RAW_CAPTURE_H

#include <stdint.h>

void App_RawCapture_Init(void);
void App_RawCapture_Task(void);
void App_RawCapture_SelectNextLabel(void);
void App_RawCapture_Toggle(void);
uint8_t App_RawCapture_IsActive(void);
const char *App_RawCapture_GetLabel(void);

#endif /* APP_RAW_CAPTURE_H */
