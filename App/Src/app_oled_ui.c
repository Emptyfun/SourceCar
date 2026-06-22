#include "app_oled_ui.h"

#include "dev_oled.h"
#include "stm32f1xx_hal.h"

#include <stdio.h>

#define APP_OLED_REFRESH_INTERVAL_MS 100U
#define APP_OLED_LINE_SIZE 32U
#define APP_OLED_EVENT_SIZE 16U

static uint8_t s_data1_level = 1U;
static uint8_t s_data2_level = 1U;
static OLED_CS_Status s_cs1_status = OLED_CS_STATUS_WAIT;
static OLED_CS_Status s_cs2_status = OLED_CS_STATUS_WAIT;
static int32_t s_raw1 = 0;
static int32_t s_raw2 = 0;
static AppOLED_Page s_page = APP_OLED_PAGE_CS1238;
static uint8_t s_k1_raw_level = 1U;
static uint8_t s_k2_raw_level = 1U;
static uint8_t s_k1_down;
static uint8_t s_k2_down;
static uint8_t s_run_enabled;
static char s_mag_target[APP_OLED_EVENT_SIZE] = "NONE";
static char s_mag_signal[APP_OLED_EVENT_SIZE] = "IDLE";
static uint8_t s_mag_detected;
static uint8_t s_mag_confidence_percent;
static char s_capture_label[APP_OLED_EVENT_SIZE] = "BASE";
static uint8_t s_capture_active;
static uint32_t s_capture_remaining_ms = 10000UL;
static uint32_t s_capture_sample_count;
static int32_t s_capture_adc1_a_raw;
static int32_t s_capture_adc1_b_raw;
static int32_t s_capture_adc2_a_raw;
static int32_t s_capture_adc2_b_raw;
static uint8_t s_cov_running;
static uint8_t s_cov_pending_start;
static uint8_t s_cov_row;
static uint8_t s_cov_line_count = 11U;
static uint8_t s_cov_state;
static uint8_t s_turn_state;
static uint8_t s_turn_action;
static uint8_t s_turn_dir_right = 1U;
static uint16_t s_turn_speed = 100U;
static int8_t s_turn_comp_deg;
static int16_t s_turn_target_deg = 90;
static int16_t s_turn_progress_deg;

static const char *CS_StatusToStr(OLED_CS_Status status);
static const char *Coverage_StateToStr(uint8_t state);
static const char *TurnCalib_StateToStr(uint8_t state);
static const char *TurnCalib_ActionToStr(uint8_t action);
static void App_OLED_DrawCS1238Page(void);
static void App_OLED_DrawKeyTestPage(void);
static void App_OLED_DrawCoveragePage(void);
static void App_OLED_DrawTurnCalibPage(void);
static void App_OLED_ShowLine(uint8_t y, const char *line);

void App_OLED_Init(void)
{
    OLED_Init();
    OLED_Clear();
    App_OLED_ShowLine(0U, "SRC CAR");
    App_OLED_ShowLine(10U, "OLED READY");
    OLED_Refresh();
}

void App_OLED_SetPage(AppOLED_Page page)
{
    s_page = page;
}

AppOLED_Page App_OLED_GetPage(void)
{
    return s_page;
}

void App_OLED_TogglePage(void)
{
    if (s_page == APP_OLED_PAGE_TURN_CALIB)
    {
        s_page = APP_OLED_PAGE_KEY_TEST;
    }
    else if (s_page == APP_OLED_PAGE_COVERAGE)
    {
        s_page = APP_OLED_PAGE_KEY_TEST;
    }
    else if (s_page == APP_OLED_PAGE_KEY_TEST)
    {
        s_page = APP_OLED_PAGE_CS1238;
    }
    else
    {
        s_page = APP_OLED_PAGE_COVERAGE;
    }
}

void App_OLED_Task(void)
{
    static uint32_t last_tick = 0UL;
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_tick) < APP_OLED_REFRESH_INTERVAL_MS)
    {
        return;
    }

    last_tick = now;

    OLED_Clear();
    if (s_page == APP_OLED_PAGE_TURN_CALIB)
    {
        App_OLED_DrawTurnCalibPage();
    }
    else if (s_page == APP_OLED_PAGE_COVERAGE)
    {
        App_OLED_DrawCoveragePage();
    }
    else if (s_page == APP_OLED_PAGE_KEY_TEST)
    {
        App_OLED_DrawKeyTestPage();
    }
    else
    {
        App_OLED_DrawCS1238Page();
    }
    OLED_Refresh();
}

void App_OLED_SetCS1238Debug(uint8_t data1_level,
                             uint8_t data2_level,
                             OLED_CS_Status cs1_status,
                             OLED_CS_Status cs2_status,
                             int32_t raw1,
                             int32_t raw2)
{
    s_data1_level = (data1_level != 0U) ? 1U : 0U;
    s_data2_level = (data2_level != 0U) ? 1U : 0U;
    s_cs1_status = cs1_status;
    s_cs2_status = cs2_status;
    s_raw1 = raw1;
    s_raw2 = raw2;
}

void App_OLED_SetButtonDebug(uint8_t k1_raw_level,
                             uint8_t k2_raw_level,
                             uint8_t k1_down,
                             uint8_t k2_down,
                             uint32_t k1_press_count,
                             uint32_t k2_press_count,
                             uint32_t k1_hold_ms,
                             uint32_t k2_hold_ms,
                             uint8_t run_enabled,
                             const char *event_text)
{
    s_k1_raw_level = (k1_raw_level != 0U) ? 1U : 0U;
    s_k2_raw_level = (k2_raw_level != 0U) ? 1U : 0U;
    s_k1_down = (k1_down != 0U) ? 1U : 0U;
    s_k2_down = (k2_down != 0U) ? 1U : 0U;
    (void)k1_press_count;
    (void)k2_press_count;
    (void)k1_hold_ms;
    (void)k2_hold_ms;
    s_run_enabled = (run_enabled != 0U) ? 1U : 0U;
    (void)event_text;
}

void App_OLED_SetMagnetDebug(const char *target_text,
                             const char *signal_text,
                             uint8_t detected,
                             uint8_t confidence_percent)
{
    if (target_text == NULL)
    {
        target_text = "NONE";
    }
    if (signal_text == NULL)
    {
        signal_text = "IDLE";
    }

    snprintf(s_mag_target, sizeof(s_mag_target), "%s", target_text);
    snprintf(s_mag_signal, sizeof(s_mag_signal), "%s", signal_text);
    s_mag_detected = (detected != 0U) ? 1U : 0U;
    s_mag_confidence_percent = (confidence_percent > 100U) ? 100U : confidence_percent;
}

void App_OLED_SetCaptureDebug(const char *label_text,
                              uint8_t active,
                              uint32_t remaining_ms,
                              uint32_t sample_count,
                              int32_t adc1_a_raw,
                              int32_t adc1_b_raw,
                              int32_t adc2_a_raw,
                              int32_t adc2_b_raw)
{
    if (label_text == NULL)
    {
        label_text = "BASE";
    }

    snprintf(s_capture_label, sizeof(s_capture_label), "%s", label_text);
    s_capture_active = (active != 0U) ? 1U : 0U;
    s_capture_remaining_ms = remaining_ms;
    s_capture_sample_count = sample_count;
    s_capture_adc1_a_raw = adc1_a_raw;
    s_capture_adc1_b_raw = adc1_b_raw;
    s_capture_adc2_a_raw = adc2_a_raw;
    s_capture_adc2_b_raw = adc2_b_raw;
}

void App_OLED_SetCoverageDebug(uint8_t running,
                               uint8_t pending_start,
                               uint8_t row,
                               uint8_t line_count,
                               uint8_t state)
{
    s_cov_running = (running != 0U) ? 1U : 0U;
    s_cov_pending_start = (pending_start != 0U) ? 1U : 0U;
    s_cov_row = row;
    s_cov_line_count = (line_count == 0U) ? 1U : line_count;
    s_cov_state = state;
}

void App_OLED_SetTurnCalibDebug(uint8_t state,
                                uint8_t action,
                                uint8_t dir_right,
                                uint16_t speed,
                                int8_t comp_deg,
                                int16_t target_deg,
                                int16_t progress_deg)
{
    s_turn_state = state;
    s_turn_action = action;
    s_turn_dir_right = (dir_right != 0U) ? 1U : 0U;
    s_turn_speed = speed;
    s_turn_comp_deg = comp_deg;
    s_turn_target_deg = target_deg;
    s_turn_progress_deg = progress_deg;
}

static const char *CS_StatusToStr(OLED_CS_Status status)
{
    switch (status)
    {
    case OLED_CS_STATUS_OK:
        return "OK";

    case OLED_CS_STATUS_TIMEOUT:
        return "TO";

    case OLED_CS_STATUS_ERROR:
        return "ERR";

    case OLED_CS_STATUS_WAIT:
    default:
        return "WAIT";
    }
}

static const char *Coverage_StateToStr(uint8_t state)
{
    switch (state)
    {
    case 0U:
        return "IDLE";

    case 1U:
        return "RUN";

    case 2U:
        return "TURN OUT";

    case 3U:
        return "SHIFT";

    case 4U:
        return "TURN BACK";

    case 6U:
        return "DONE";

    case 7U:
        return "ABORT";

    case 8U:
        return "ERROR";

    case 5U:
    default:
        return "NEXT";
    }
}

static const char *TurnCalib_StateToStr(uint8_t state)
{
    switch (state)
    {
    case 1U:
        return "PENDING";

    case 2U:
        return "RUN";

    case 3U:
        return "DONE";

    case 4U:
        return "ERROR";

    case 5U:
        return "STOP";

    case 0U:
    default:
        return "READY";
    }
}

static const char *TurnCalib_ActionToStr(uint8_t action)
{
    switch (action)
    {
    case 0U:
        return "START";

    case 1U:
        return "C+";

    case 2U:
        return "C-";

    case 3U:
        return "SPD";

    case 4U:
        return "DIR";

    case 5U:
    default:
        return "RESET";
    }
}

static void App_OLED_DrawCS1238Page(void)
{
    char line[APP_OLED_LINE_SIZE];

    snprintf(line, sizeof(line), "CS PAGE RUN:%s", (s_run_enabled != 0U) ? "ON" : "OFF");
    App_OLED_ShowLine(0U, line);

    snprintf(line, sizeof(line), "CS1:%s D1:%u",
             CS_StatusToStr(s_cs1_status),
             (unsigned int)s_data1_level);
    App_OLED_ShowLine(10U, line);

    snprintf(line, sizeof(line), "R1:%ld", (long)s_raw1);
    App_OLED_ShowLine(20U, line);

    snprintf(line, sizeof(line), "CS2:%s D2:%u",
             CS_StatusToStr(s_cs2_status),
             (unsigned int)s_data2_level);
    App_OLED_ShowLine(30U, line);

    snprintf(line, sizeof(line), "R2:%ld", (long)s_raw2);
    App_OLED_ShowLine(40U, line);

    snprintf(line, sizeof(line), "MAG:%.8s %.8s",
             s_mag_target,
             s_mag_signal);
    App_OLED_ShowLine(50U, line);
}

static void App_OLED_DrawKeyTestPage(void)
{
    char line[APP_OLED_LINE_SIZE];

    snprintf(line, sizeof(line), "CAP:%s L:%s",
             (s_capture_active != 0U) ? "RUN" : "IDLE",
             s_capture_label);
    App_OLED_ShowLine(0U, line);

    snprintf(line, sizeof(line), "1A:%ld", (long)s_capture_adc1_a_raw);
    App_OLED_ShowLine(10U, line);

    snprintf(line, sizeof(line), "1B:%ld", (long)s_capture_adc1_b_raw);
    App_OLED_ShowLine(20U, line);

    snprintf(line, sizeof(line), "2A:%ld", (long)s_capture_adc2_a_raw);
    App_OLED_ShowLine(30U, line);

    snprintf(line, sizeof(line), "2B:%ld", (long)s_capture_adc2_b_raw);
    App_OLED_ShowLine(40U, line);

    if (s_capture_active != 0U)
    {
        snprintf(line, sizeof(line), "T:%lu.%lus N:%lu",
                 (unsigned long)(s_capture_remaining_ms / 1000UL),
                 (unsigned long)((s_capture_remaining_ms % 1000UL) / 100UL),
                 (unsigned long)s_capture_sample_count);
    }
    else
    {
        snprintf(line, sizeof(line), "MAG:%.8s %.8s",
                 s_mag_target,
                 s_mag_signal);
    }
    App_OLED_ShowLine(50U, line);
}

static void App_OLED_DrawCoveragePage(void)
{
    char line[APP_OLED_LINE_SIZE];
    const char *mode;

    if (s_cov_pending_start != 0U)
    {
        mode = "PENDING";
    }
    else if (s_cov_running != 0U)
    {
        mode = "RUN";
    }
    else if (s_cov_state == 6U)
    {
        mode = "DONE";
    }
    else if (s_cov_state == 7U)
    {
        mode = "STOP";
    }
    else if (s_cov_state == 8U)
    {
        mode = "ERROR";
    }
    else
    {
        mode = "READY";
    }

    snprintf(line, sizeof(line), "COV %s", mode);
    App_OLED_ShowLine(0U, line);

    snprintf(line, sizeof(line), "ROW:%u/%u",
             (unsigned int)(s_cov_row + 1U),
             (unsigned int)s_cov_line_count);
    App_OLED_ShowLine(10U, line);

    snprintf(line, sizeof(line), "ACT:%s", Coverage_StateToStr(s_cov_state));
    App_OLED_ShowLine(20U, line);

    App_OLED_ShowLine(30U, "AREA:1m x 1m");
    App_OLED_ShowLine(40U, "K2 START/STOP");
    App_OLED_ShowLine(50U, "LONG: E-STOP");
}

static void App_OLED_DrawTurnCalibPage(void)
{
    char line[APP_OLED_LINE_SIZE];

    snprintf(line, sizeof(line), "TURN %s", TurnCalib_StateToStr(s_turn_state));
    App_OLED_ShowLine(0U, line);

    snprintf(line, sizeof(line), "SEL:%s", TurnCalib_ActionToStr(s_turn_action));
    App_OLED_ShowLine(10U, line);

    snprintf(line, sizeof(line), "DIR:%c SPD:%u",
             (s_turn_dir_right != 0U) ? 'R' : 'L',
             (unsigned int)s_turn_speed);
    App_OLED_ShowLine(20U, line);

    snprintf(line, sizeof(line), "CMP:%+d TG:%+d",
             (int)s_turn_comp_deg,
             (int)s_turn_target_deg);
    App_OLED_ShowLine(30U, line);

    snprintf(line, sizeof(line), "ODO:%+d", (int)s_turn_progress_deg);
    App_OLED_ShowLine(40U, line);

    App_OLED_ShowLine(50U, "K1 SEL  K2 OK");
}

static void App_OLED_ShowLine(uint8_t y, const char *line)
{
    OLED_ShowString(0U, y, (const uint8_t *)line, OLED_FONT_6X8, 1U);
}
