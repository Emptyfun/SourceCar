#ifndef APP_OLED_UI_H
#define APP_OLED_UI_H

#include <stdint.h>

typedef enum
{
    OLED_CS_STATUS_WAIT = 0,
    OLED_CS_STATUS_OK,
    OLED_CS_STATUS_TIMEOUT,
    OLED_CS_STATUS_ERROR
} OLED_CS_Status;

typedef enum
{
    APP_OLED_PAGE_KEY_TEST = 0,
    APP_OLED_PAGE_CS1238
} AppOLED_Page;

void App_OLED_Init(void);
void App_OLED_Task(void);
void App_OLED_SetPage(AppOLED_Page page);
AppOLED_Page App_OLED_GetPage(void);
void App_OLED_TogglePage(void);

void App_OLED_SetCS1238Debug(uint8_t data1_level,
                             uint8_t data2_level,
                             OLED_CS_Status cs1_status,
                             OLED_CS_Status cs2_status,
                             int32_t raw1,
                             int32_t raw2);
void App_OLED_SetButtonDebug(uint8_t k1_raw_level,
                             uint8_t k2_raw_level,
                             uint8_t k1_down,
                             uint8_t k2_down,
                             uint32_t k1_press_count,
                             uint32_t k2_press_count,
                             uint32_t k1_hold_ms,
                             uint32_t k2_hold_ms,
                             uint8_t run_enabled,
                             const char *event_text);

#endif /* APP_OLED_UI_H */
