#include "app_cs1238_task.h"

#include "app_config.h"
#include "dev_cs1238.h"
#if APP_ENABLE_OLED
#include "app_oled_ui.h"
#endif
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define APP_CS1238_WINDOW_SIZE 31U
#define APP_CS1238_DISCARD_COUNT 3U
#define APP_CS1238_PRINT_INTERVAL_MS 1000U
#define APP_CS1238_PRINT_SAMPLE_WINDOW_MS 1200U
#define APP_CS1238_ENABLE_ADC1 1
#define APP_CS1238_ENABLE_ADC2 1
#define APP_CS1238_POLL_ADC2 0
#define APP_CS1238_PRINT_ADC2 0
#define APP_CS1238_VREF_MV 3300U
#define APP_CS1238_REFO_OFF 0U
#define APP_CS1238_SPEED_SEL CS1238_SPEED_10HZ
#define APP_CS1238_PGA_SEL CS1238_PGA_SEL_2
#define APP_CS1238_ADC_FULL_SCALE 8388607LL
#define APP_CS1238_POS_SAT_LIMIT 8388591L
#define APP_CS1238_NEG_SAT_LIMIT (-8388592L)
#define APP_CS1238_CHANNEL_COUNT 2U
#define APP_CS1238_ACTIVE_CHANNEL_COUNT 2U

typedef enum
{
    APP_CS1238_STATUS_WAIT = 0,
    APP_CS1238_STATUS_OK,
    APP_CS1238_STATUS_TIMEOUT,
    APP_CS1238_STATUS_ERROR
} AppCS1238_Status;

typedef struct
{
    const char *name;
    uint8_t channel_sel;
    int32_t samples[APP_CS1238_WINDOW_SIZE];
    uint32_t sample_ticks[APP_CS1238_WINDOW_SIZE];
    int32_t latest;
    uint32_t latest_tick;
    uint8_t count;
    uint8_t next;
    uint8_t has_latest;
} AppCS1238_Stream;

typedef struct
{
    const char *name;
    CS1238_Handle *handle;
    AppCS1238_Stream stream[APP_CS1238_CHANNEL_COUNT];
    uint8_t active;
    uint8_t discard;
    uint8_t scan_enabled;
    HAL_StatusTypeDef config_status;
    AppCS1238_Status status;
} AppCS1238_Device;

typedef struct
{
    int32_t avg;
    int32_t median;
    int32_t min;
    int32_t max;
    uint8_t count;
} AppCS1238_Filtered;

static AppCS1238_Device adc1;
#if APP_CS1238_ENABLE_ADC2
static AppCS1238_Device adc2;
#endif
static uint32_t last_print_tick;

static void App_CS1238_DeviceInit(AppCS1238_Device *dev,
                                  CS1238_Handle *handle,
                                  const char *name,
                                  const char *ch_a_name,
                                  const char *ch_b_name,
                                  uint8_t enabled);
static void App_CS1238_PollDevice(AppCS1238_Device *dev);
static HAL_StatusTypeDef App_CS1238_SelectActiveChannel(AppCS1238_Device *dev);
static uint8_t App_CS1238_ConfigForChannel(uint8_t channel_sel);
static void App_CS1238_AddSample(AppCS1238_Stream *stream, int32_t raw);
static void App_CS1238_FilterRecentStream(const AppCS1238_Stream *stream,
                                          uint32_t now,
                                          AppCS1238_Filtered *out);
static void App_CS1238_PrintDevice(const AppCS1238_Device *dev);
static void App_CS1238_PrintStream(const AppCS1238_Device *dev,
                                   const AppCS1238_Stream *stream,
                                   uint32_t now,
                                   uint8_t stream_index);
static AppCS1238_Status App_CS1238_StatusFromHAL(HAL_StatusTypeDef status);
static const char *App_CS1238_HalStatusText(HAL_StatusTypeDef status);
static void CS1238_DebugPrintf(const char *fmt, ...);
#if APP_ENABLE_OLED
static void App_CS1238_UpdateOLED(void);
static uint8_t App_CS1238_ReadDataLevel(const CS1238_Handle *handle);
static int32_t App_CS1238_GetDisplayRaw(const AppCS1238_Device *dev);
static OLED_CS_Status App_CS1238_ToOLEDStatus(AppCS1238_Status status);
#endif

void App_CS1238_Init(void)
{
    App_CS1238_DeviceInit(&adc1,
                          &hcs1238_1,
                          "ADC1",
                          "ADC1-A",
                          "ADC1-B",
                          APP_CS1238_ENABLE_ADC1);
#if APP_CS1238_ENABLE_ADC2
    App_CS1238_DeviceInit(&adc2,
                          &hcs1238_2,
                          "ADC2",
                          "ADC2-A",
                          "ADC2-B",
                          APP_CS1238_ENABLE_ADC2);
#endif

    last_print_tick = HAL_GetTick();

    CS1238_DebugPrintf("[CS1238] init speed=10Hz pga=%u cfgA=0x%02X cfgB=0x%02X ADC1=%s ADC2=%s\r\n",
                       (unsigned int)CS1238_PgaGainFromSel(APP_CS1238_PGA_SEL),
                       (unsigned int)App_CS1238_ConfigForChannel(CS1238_CH_A),
                       (unsigned int)App_CS1238_ConfigForChannel(CS1238_CH_B),
                       App_CS1238_HalStatusText(adc1.config_status),
                       App_CS1238_HalStatusText(adc2.config_status));

#if APP_ENABLE_OLED
    App_CS1238_UpdateOLED();
#endif
}

void App_CS1238_Task(void)
{
    uint32_t now = HAL_GetTick();

#if APP_CS1238_ENABLE_ADC1
    App_CS1238_PollDevice(&adc1);
#endif
#if APP_CS1238_ENABLE_ADC2 && APP_CS1238_POLL_ADC2
    App_CS1238_PollDevice(&adc2);
#endif

#if APP_ENABLE_OLED
    App_CS1238_UpdateOLED();
#endif

    if ((uint32_t)(now - last_print_tick) >= APP_CS1238_PRINT_INTERVAL_MS)
    {
        last_print_tick = now;
        CS1238_DebugPrintf("[CS1238] t=%lums\r\n", (unsigned long)now);
#if APP_CS1238_ENABLE_ADC1
        App_CS1238_PrintDevice(&adc1);
#endif
#if APP_CS1238_ENABLE_ADC2 && APP_CS1238_PRINT_ADC2
        App_CS1238_PrintDevice(&adc2);
#endif
    }
}

static void App_CS1238_DeviceInit(AppCS1238_Device *dev,
                                  CS1238_Handle *handle,
                                  const char *name,
                                  const char *ch_a_name,
                                  const char *ch_b_name,
                                  uint8_t enabled)
{
    memset(dev, 0, sizeof(*dev));

    dev->name = name;
    dev->handle = handle;
    dev->stream[0].name = ch_a_name;
    dev->stream[0].channel_sel = CS1238_CH_A;
    dev->stream[1].name = ch_b_name;
    dev->stream[1].channel_sel = CS1238_CH_B;
    dev->active = 0U;
    dev->discard = APP_CS1238_DISCARD_COUNT;
    dev->config_status = HAL_ERROR;
    dev->status = APP_CS1238_STATUS_WAIT;

    if (enabled == 0U)
    {
        dev->status = APP_CS1238_STATUS_ERROR;
        return;
    }

    CS1238_Init(handle);
    dev->config_status = App_CS1238_SelectActiveChannel(dev);
    dev->scan_enabled = (dev->config_status == HAL_OK) ? 1U : 0U;
    dev->status = (dev->config_status == HAL_OK) ? APP_CS1238_STATUS_WAIT
                                                  : App_CS1238_StatusFromHAL(dev->config_status);
}

static void App_CS1238_PollDevice(AppCS1238_Device *dev)
{
    HAL_StatusTypeDef read_status;
    int32_t raw;

    if (dev->scan_enabled == 0U)
    {
        return;
    }

    if (!CS1238_IsDataReady(dev->handle))
    {
        return;
    }

    read_status = CS1238_ReadRaw(dev->handle, &raw, 1U);
    if (read_status != HAL_OK)
    {
        dev->status = App_CS1238_StatusFromHAL(read_status);
        dev->scan_enabled = 0U;
        return;
    }

    dev->status = APP_CS1238_STATUS_OK;

    if (dev->discard > 0U)
    {
        dev->discard--;
        return;
    }

    App_CS1238_AddSample(&dev->stream[dev->active], raw);

    dev->active = (uint8_t)((dev->active + 1U) % APP_CS1238_ACTIVE_CHANNEL_COUNT);
    dev->discard = APP_CS1238_DISCARD_COUNT;
    dev->config_status = App_CS1238_SelectActiveChannel(dev);
    if (dev->config_status != HAL_OK)
    {
        dev->status = App_CS1238_StatusFromHAL(dev->config_status);
        dev->scan_enabled = 0U;
    }
}

static HAL_StatusTypeDef App_CS1238_SelectActiveChannel(AppCS1238_Device *dev)
{
    return CS1238_WriteConfig(dev->handle,
                              App_CS1238_ConfigForChannel(dev->stream[dev->active].channel_sel));
}

static uint8_t App_CS1238_ConfigForChannel(uint8_t channel_sel)
{
    return CS1238_MakeConfig(APP_CS1238_REFO_OFF,
                             APP_CS1238_SPEED_SEL,
                             APP_CS1238_PGA_SEL,
                             channel_sel);
}

static void App_CS1238_AddSample(AppCS1238_Stream *stream, int32_t raw)
{
    uint32_t now = HAL_GetTick();

    stream->samples[stream->next] = raw;
    stream->sample_ticks[stream->next] = now;
    stream->latest = raw;
    stream->latest_tick = now;
    stream->has_latest = 1U;
    stream->next = (uint8_t)((stream->next + 1U) % APP_CS1238_WINDOW_SIZE);
    if (stream->count < APP_CS1238_WINDOW_SIZE)
    {
        stream->count++;
    }
}

static void App_CS1238_FilterRecentStream(const AppCS1238_Stream *stream,
                                          uint32_t now,
                                          AppCS1238_Filtered *out)
{
    int64_t sum = 0;
    uint8_t i;
    uint8_t used = 0U;

    memset(out, 0, sizeof(*out));
    if (stream->has_latest == 0U)
    {
        return;
    }

    for (i = 0U; i < stream->count; i++)
    {
        if ((uint32_t)(now - stream->sample_ticks[i]) <= APP_CS1238_PRINT_SAMPLE_WINDOW_MS)
        {
            if (used == 0U)
            {
                out->min = stream->samples[i];
                out->max = stream->samples[i];
            }
            else
            {
                if (stream->samples[i] < out->min)
                {
                    out->min = stream->samples[i];
                }
                if (stream->samples[i] > out->max)
                {
                    out->max = stream->samples[i];
                }
            }
            sum += stream->samples[i];
            used++;
        }
    }

    if (used == 0U)
    {
        out->avg = stream->latest;
        out->median = stream->latest;
        out->min = stream->latest;
        out->max = stream->latest;
        out->count = 1U;
        return;
    }

    out->avg = (int32_t)(sum / (int64_t)used);
    out->median = stream->latest;
    out->count = used;
}

static void App_CS1238_PrintDevice(const AppCS1238_Device *dev)
{
    uint32_t now = HAL_GetTick();

    App_CS1238_PrintStream(dev, &dev->stream[0], now, 0U);
#if APP_CS1238_ACTIVE_CHANNEL_COUNT > 1U
    App_CS1238_PrintStream(dev, &dev->stream[1], now, 1U);
#endif
}

static void App_CS1238_PrintStream(const AppCS1238_Device *dev,
                                   const AppCS1238_Stream *stream,
                                   uint32_t now,
                                   uint8_t stream_index)
{
    AppCS1238_Filtered filtered;

    (void)dev;
    (void)stream_index;
    App_CS1238_FilterRecentStream(stream, now, &filtered);
    CS1238_DebugPrintf("%s raw_avg=%ld\r\n", stream->name, (long)filtered.avg);
}

static AppCS1238_Status App_CS1238_StatusFromHAL(HAL_StatusTypeDef status)
{
    switch (status)
    {
    case HAL_OK:
        return APP_CS1238_STATUS_OK;

    case HAL_TIMEOUT:
        return APP_CS1238_STATUS_TIMEOUT;

    case HAL_BUSY:
    case HAL_ERROR:
    default:
        return APP_CS1238_STATUS_ERROR;
    }
}

static const char *App_CS1238_HalStatusText(HAL_StatusTypeDef status)
{
    switch (status)
    {
    case HAL_OK:
        return "OK";

    case HAL_TIMEOUT:
        return "TIMEOUT";

    case HAL_BUSY:
        return "BUSY";

    case HAL_ERROR:
    default:
        return "ERROR";
    }
}

#if APP_ENABLE_OLED
static void App_CS1238_UpdateOLED(void)
{
    uint8_t data1 = App_CS1238_ReadDataLevel(adc1.handle);
    uint8_t data2 = 1U;
    OLED_CS_Status status1 = App_CS1238_ToOLEDStatus(adc1.status);
    OLED_CS_Status status2 = OLED_CS_STATUS_WAIT;
    int32_t raw1 = App_CS1238_GetDisplayRaw(&adc1);
    int32_t raw2 = 0;

#if APP_CS1238_ENABLE_ADC2
    data2 = App_CS1238_ReadDataLevel(adc2.handle);
    status2 = App_CS1238_ToOLEDStatus(adc2.status);
    raw2 = App_CS1238_GetDisplayRaw(&adc2);
#endif

    App_OLED_SetCS1238Debug(data1, data2, status1, status2, raw1, raw2);
}

static uint8_t App_CS1238_ReadDataLevel(const CS1238_Handle *handle)
{
    if ((handle == NULL) || (handle->data_port == NULL))
    {
        return 1U;
    }

    return (HAL_GPIO_ReadPin(handle->data_port, handle->data_pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static int32_t App_CS1238_GetDisplayRaw(const AppCS1238_Device *dev)
{
    const AppCS1238_Stream *best = NULL;
    uint8_t i;

    if (dev == NULL)
    {
        return 0;
    }

    for (i = 0U; i < APP_CS1238_CHANNEL_COUNT; i++)
    {
        const AppCS1238_Stream *stream = &dev->stream[i];

        if (stream->has_latest == 0U)
        {
            continue;
        }

        if ((best == NULL) ||
            ((uint32_t)(stream->latest_tick - best->latest_tick) < 0x80000000UL))
        {
            best = stream;
        }
    }

    return (best != NULL) ? best->latest : 0;
}

static OLED_CS_Status App_CS1238_ToOLEDStatus(AppCS1238_Status status)
{
    switch (status)
    {
    case APP_CS1238_STATUS_OK:
        return OLED_CS_STATUS_OK;

    case APP_CS1238_STATUS_TIMEOUT:
        return OLED_CS_STATUS_TIMEOUT;

    case APP_CS1238_STATUS_ERROR:
        return OLED_CS_STATUS_ERROR;

    case APP_CS1238_STATUS_WAIT:
    default:
        return OLED_CS_STATUS_WAIT;
    }
}
#endif

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

    if (len >= (int)sizeof(buf))
    {
        len = (int)sizeof(buf) - 1;
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 20U);
}
