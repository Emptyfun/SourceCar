#include "app_cs1238_task.h"

#include "dev_cs1238.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define APP_CS1238_WINDOW_SIZE 31U
#define APP_CS1238_DISCARD_COUNT 3U
#define APP_CS1238_PRINT_INTERVAL_MS 5000U
#define APP_CS1238_DEFAULT_CFG 0x0CU
#define APP_CS1238_VREF 3.3f
#define APP_CS1238_PGA_GAIN 128U

typedef enum
{
    APP_CS1238_STATUS_TIMEOUT = 0,
    APP_CS1238_STATUS_OK
} AppCS1238_Status;

typedef struct
{
    CS1238_Handle *handle;
    int32_t samples[APP_CS1238_WINDOW_SIZE];
    uint8_t count;
    uint8_t next;
    uint8_t discard;
    AppCS1238_Status status;
} AppCS1238_Channel;

typedef struct
{
    int32_t avg;
    int32_t median;
    int32_t min;
    int32_t max;
    uint8_t count;
} AppCS1238_Filtered;

static AppCS1238_Channel adc1;
static AppCS1238_Channel adc2;
static uint32_t last_print_tick;

static void App_CS1238_PollChannel(AppCS1238_Channel *ch);
static void App_CS1238_AddSample(AppCS1238_Channel *ch, int32_t raw);
static void App_CS1238_FilterChannel(const AppCS1238_Channel *ch, AppCS1238_Filtered *out);
static void App_CS1238_Sort(int32_t *data, uint8_t count);
static void App_CS1238_PrintChannel(const char *name, const AppCS1238_Channel *ch);
static void CS1238_DebugPrintf(const char *fmt, ...);

void App_CS1238_Init(void)
{
    CS1238_InitAll();

    adc1.handle = &hcs1238_1;
    adc1.count = 0U;
    adc1.next = 0U;
    adc1.discard = APP_CS1238_DISCARD_COUNT;
    adc1.status = APP_CS1238_STATUS_TIMEOUT;

    adc2.handle = &hcs1238_2;
    adc2.count = 0U;
    adc2.next = 0U;
    adc2.discard = APP_CS1238_DISCARD_COUNT;
    adc2.status = APP_CS1238_STATUS_TIMEOUT;

    (void)CS1238_WriteConfig(&hcs1238_1, APP_CS1238_DEFAULT_CFG);
    (void)CS1238_WriteConfig(&hcs1238_2, APP_CS1238_DEFAULT_CFG);

    last_print_tick = HAL_GetTick();
}

void App_CS1238_Task(void)
{
    uint32_t now = HAL_GetTick();

    App_CS1238_PollChannel(&adc1);
    App_CS1238_PollChannel(&adc2);

    if ((uint32_t)(now - last_print_tick) >= APP_CS1238_PRINT_INTERVAL_MS)
    {
        last_print_tick = now;
        CS1238_DebugPrintf("[CS1238] t=%lums\r\n", (unsigned long)now);
        App_CS1238_PrintChannel("ADC1", &adc1);
        App_CS1238_PrintChannel("ADC2", &adc2);
    }
}

static void App_CS1238_PollChannel(AppCS1238_Channel *ch)
{
    int32_t raw;

    if (!CS1238_IsDataReady(ch->handle))
    {
        return;
    }

    if (CS1238_ReadRaw(ch->handle, &raw, 1U) != HAL_OK)
    {
        ch->status = APP_CS1238_STATUS_TIMEOUT;
        return;
    }

    ch->status = APP_CS1238_STATUS_OK;

    if (ch->discard > 0U)
    {
        ch->discard--;
        return;
    }

    App_CS1238_AddSample(ch, raw);
}

static void App_CS1238_AddSample(AppCS1238_Channel *ch, int32_t raw)
{
    ch->samples[ch->next] = raw;
    ch->next = (uint8_t)((ch->next + 1U) % APP_CS1238_WINDOW_SIZE);
    if (ch->count < APP_CS1238_WINDOW_SIZE)
    {
        ch->count++;
    }
}

static void App_CS1238_FilterChannel(const AppCS1238_Channel *ch, AppCS1238_Filtered *out)
{
    int32_t sorted[APP_CS1238_WINDOW_SIZE];
    int64_t sum = 0;
    uint8_t i;
    uint8_t trim;
    uint8_t start;
    uint8_t end;
    uint8_t used = 0U;

    memset(out, 0, sizeof(*out));
    out->count = ch->count;
    if (ch->count == 0U)
    {
        return;
    }

    for (i = 0U; i < ch->count; i++)
    {
        sorted[i] = ch->samples[i];
    }
    App_CS1238_Sort(sorted, ch->count);

    out->min = sorted[0];
    out->max = sorted[ch->count - 1U];
    out->median = sorted[ch->count / 2U];

    trim = (uint8_t)(ch->count / 5U);
    start = trim;
    end = (uint8_t)(ch->count - trim);
    if (start >= end)
    {
        start = 0U;
        end = ch->count;
    }

    for (i = start; i < end; i++)
    {
        sum += sorted[i];
        used++;
    }

    if (used > 0U)
    {
        out->avg = (int32_t)(sum / (int64_t)used);
    }
}

static void App_CS1238_Sort(int32_t *data, uint8_t count)
{
    uint8_t i;
    uint8_t j;

    for (i = 1U; i < count; i++)
    {
        int32_t key = data[i];
        j = i;
        while ((j > 0U) && (data[j - 1U] > key))
        {
            data[j] = data[j - 1U];
            j--;
        }
        data[j] = key;
    }
}

static void App_CS1238_PrintChannel(const char *name, const AppCS1238_Channel *ch)
{
    AppCS1238_Filtered filtered;
    float uv;
    const char *status;

    App_CS1238_FilterChannel(ch, &filtered);
    uv = CS1238_RawToVoltage(filtered.avg, APP_CS1238_VREF, APP_CS1238_PGA_GAIN) * 1000000.0f;

    if (ch->status == APP_CS1238_STATUS_TIMEOUT)
    {
        status = "TIMEOUT";
    }
    else if (filtered.count < 5U)
    {
        status = "LOW_SAMPLE_COUNT";
    }
    else
    {
        status = "OK";
    }

    CS1238_DebugPrintf("  %s: count=%u raw_avg=%ld raw_med=%ld v_avg=%.2fuV min=%ld max=%ld status=%s\r\n",
                       name,
                       (unsigned int)filtered.count,
                       (long)filtered.avg,
                       (long)filtered.median,
                       (double)uv,
                       (long)filtered.min,
                       (long)filtered.max,
                       status);
}

static void CS1238_DebugPrintf(const char *fmt, ...)
{
    char buf[192];
    va_list args;
    int len;

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }

    if (len > (int)sizeof(buf))
    {
        len = (int)sizeof(buf);
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)strlen(buf), 20U);
}
