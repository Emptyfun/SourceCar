#include "app_raw_capture.h"

#include "app_config.h"
#include "app_cs1238_task.h"
#if APP_ENABLE_OLED
#include "app_oled_ui.h"
#endif
#include "stm32f1xx_hal.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>

#define RAW_CAPTURE_DURATION_MS 10000U

static const char *const s_labels[] = {
    "BASE",
    "S1_NEAR",
    "S2_NEAR",
    "CENTER",
    "FAR"
};

static uint8_t s_label_index;
static uint8_t s_active;
static uint8_t s_have_a;
static uint8_t s_have_b;
static uint8_t s_have_adc1_a;
static uint8_t s_have_adc1_b;
static uint32_t s_start_tick;
static uint32_t s_seq;
static uint32_t s_sample_count;
static uint32_t s_last_a_tick;
static uint32_t s_last_b_tick;
static uint32_t s_last_adc1_a_tick;
static uint32_t s_last_adc1_b_tick;
static int32_t s_last_a_raw;
static int32_t s_last_b_raw;
static int32_t s_last_adc1_a_raw;
static int32_t s_last_adc1_b_raw;

static void RawCapture_Start(void);
static void RawCapture_Stop(const char *reason);
static void RawCapture_ProcessSample(const AppCS1238Snapshot_t *snapshot, uint32_t now);
static void RawCapture_PrintSample(uint32_t now, char channel, int32_t raw);
static void RawCapture_UpdateOLED(const AppCS1238Snapshot_t *snapshot, uint32_t now);
#if APP_ENABLE_OLED
static uint32_t RawCapture_RemainingMs(uint32_t now);
#endif
static void RawCapture_Printf(const char *fmt, ...);

void App_RawCapture_Init(void)
{
    s_label_index = 0U;
    s_active = 0U;
    s_have_a = 0U;
    s_have_b = 0U;
    s_have_adc1_a = 0U;
    s_have_adc1_b = 0U;
    s_start_tick = 0UL;
    s_seq = 0UL;
    s_sample_count = 0UL;
    s_last_a_tick = 0UL;
    s_last_b_tick = 0UL;
    s_last_adc1_a_tick = 0UL;
    s_last_adc1_b_tick = 0UL;
    s_last_a_raw = 0;
    s_last_b_raw = 0;
    s_last_adc1_a_raw = 0;
    s_last_adc1_b_raw = 0;
}

void App_RawCapture_Task(void)
{
    AppCS1238Snapshot_t snapshot;
    uint32_t now = HAL_GetTick();

    App_CS1238_GetSnapshot(&snapshot);

    if (s_active != 0U)
    {
        RawCapture_ProcessSample(&snapshot, now);
        if ((uint32_t)(now - s_start_tick) >= RAW_CAPTURE_DURATION_MS)
        {
            RawCapture_Stop("DONE");
        }
    }

    RawCapture_UpdateOLED(&snapshot, now);
}

void App_RawCapture_SelectNextLabel(void)
{
    if (s_active != 0U)
    {
        return;
    }

    s_label_index++;
    if (s_label_index >= (uint8_t)(sizeof(s_labels) / sizeof(s_labels[0])))
    {
        s_label_index = 0U;
    }
}

void App_RawCapture_Toggle(void)
{
    if (s_active != 0U)
    {
        RawCapture_Stop("STOP");
    }
    else
    {
        RawCapture_Start();
    }
}

uint8_t App_RawCapture_IsActive(void)
{
    return s_active;
}

const char *App_RawCapture_GetLabel(void)
{
    return s_labels[s_label_index];
}

static void RawCapture_Start(void)
{
    s_active = 1U;
    s_have_a = 0U;
    s_have_b = 0U;
    s_have_adc1_a = 0U;
    s_have_adc1_b = 0U;
    s_start_tick = HAL_GetTick();
    s_seq = 0UL;
    s_sample_count = 0UL;
    s_last_a_tick = 0UL;
    s_last_b_tick = 0UL;
    s_last_adc1_a_tick = 0UL;
    s_last_adc1_b_tick = 0UL;

    RawCapture_Printf("#CAP START label=%s target=ADC1_ADC2 duration_ms=%lu speed=40Hz\r\n",
                      App_RawCapture_GetLabel(),
                      (unsigned long)RAW_CAPTURE_DURATION_MS);
    RawCapture_Printf("elapsed_ms,seq,label,ch,raw,adc1_a_raw,adc1_b_raw,adc2_a_raw,adc2_b_raw\r\n");
}

static void RawCapture_Stop(const char *reason)
{
    if (s_active == 0U)
    {
        return;
    }

    s_active = 0U;
    RawCapture_Printf("#CAP END label=%s count=%lu elapsed_ms=%lu reason=%s\r\n",
                      App_RawCapture_GetLabel(),
                      (unsigned long)s_sample_count,
                      (unsigned long)(HAL_GetTick() - s_start_tick),
                      reason);
}

static void RawCapture_ProcessSample(const AppCS1238Snapshot_t *snapshot, uint32_t now)
{
    if (snapshot->adc1_a.status == APP_CS1238_CH_STATUS_OK)
    {
        if ((snapshot->adc1_a.last_update_tick >= s_start_tick) &&
            (snapshot->adc1_a.last_update_tick != s_last_adc1_a_tick))
        {
            s_last_adc1_a_tick = snapshot->adc1_a.last_update_tick;
            s_last_adc1_a_raw = snapshot->adc1_a.raw_last;
            s_have_adc1_a = 1U;
            RawCapture_PrintSample(now, 'A', s_last_adc1_a_raw);
        }
    }

    if (snapshot->adc1_b.status == APP_CS1238_CH_STATUS_OK)
    {
        if ((snapshot->adc1_b.last_update_tick >= s_start_tick) &&
            (snapshot->adc1_b.last_update_tick != s_last_adc1_b_tick))
        {
            s_last_adc1_b_tick = snapshot->adc1_b.last_update_tick;
            s_last_adc1_b_raw = snapshot->adc1_b.raw_last;
            s_have_adc1_b = 1U;
            RawCapture_PrintSample(now, 'B', s_last_adc1_b_raw);
        }
    }

    if (snapshot->adc2_a.status == APP_CS1238_CH_STATUS_OK)
    {
        if ((snapshot->adc2_a.last_update_tick >= s_start_tick) &&
            (snapshot->adc2_a.last_update_tick != s_last_a_tick))
        {
            s_last_a_tick = snapshot->adc2_a.last_update_tick;
            s_last_a_raw = snapshot->adc2_a.raw_last;
            s_have_a = 1U;
            RawCapture_PrintSample(now, 'C', s_last_a_raw);
        }
    }

    if (snapshot->adc2_b.status == APP_CS1238_CH_STATUS_OK)
    {
        if ((snapshot->adc2_b.last_update_tick >= s_start_tick) &&
            (snapshot->adc2_b.last_update_tick != s_last_b_tick))
        {
            s_last_b_tick = snapshot->adc2_b.last_update_tick;
            s_last_b_raw = snapshot->adc2_b.raw_last;
            s_have_b = 1U;
            RawCapture_PrintSample(now, 'D', s_last_b_raw);
        }
    }
}

static void RawCapture_PrintSample(uint32_t now, char channel, int32_t raw)
{
    int32_t adc1_a_raw = s_have_adc1_a ? s_last_adc1_a_raw : 0;
    int32_t adc1_b_raw = s_have_adc1_b ? s_last_adc1_b_raw : 0;
    int32_t adc2_a_raw = s_have_a ? s_last_a_raw : 0;
    int32_t adc2_b_raw = s_have_b ? s_last_b_raw : 0;

    RawCapture_Printf("%lu,%lu,%s,%c,%ld,%ld,%ld,%ld,%ld\r\n",
                      (unsigned long)(now - s_start_tick),
                      (unsigned long)s_seq,
                      App_RawCapture_GetLabel(),
                      channel,
                      (long)raw,
                      (long)adc1_a_raw,
                      (long)adc1_b_raw,
                      (long)adc2_a_raw,
                      (long)adc2_b_raw);
    s_seq++;
    s_sample_count++;
}

static void RawCapture_UpdateOLED(const AppCS1238Snapshot_t *snapshot, uint32_t now)
{
#if APP_ENABLE_OLED
    App_OLED_SetCaptureDebug(App_RawCapture_GetLabel(),
                             s_active,
                             RawCapture_RemainingMs(now),
                             s_sample_count,
                             snapshot->adc1_a.raw_last,
                             snapshot->adc1_b.raw_last,
                             snapshot->adc2_a.raw_last,
                             snapshot->adc2_b.raw_last);
#else
    (void)snapshot;
    (void)now;
#endif
}

#if APP_ENABLE_OLED
static uint32_t RawCapture_RemainingMs(uint32_t now)
{
    uint32_t elapsed;

    if (s_active == 0U)
    {
        return RAW_CAPTURE_DURATION_MS;
    }

    elapsed = (uint32_t)(now - s_start_tick);
    if (elapsed >= RAW_CAPTURE_DURATION_MS)
    {
        return 0UL;
    }

    return RAW_CAPTURE_DURATION_MS - elapsed;
}
#endif

static void RawCapture_Printf(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    int len;

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }

    if (len >= (int)sizeof(buf))
    {
        len = (int)sizeof(buf) - 1;
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 30U);
}
