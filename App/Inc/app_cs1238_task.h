#ifndef APP_CS1238_TASK_H
#define APP_CS1238_TASK_H

#include <stdint.h>

typedef enum
{
    APP_CS1238_CH_STATUS_WAIT = 0,
    APP_CS1238_CH_STATUS_OK,
    APP_CS1238_CH_STATUS_TIMEOUT,
    APP_CS1238_CH_STATUS_ERROR
} AppCS1238ChStatus_t;

typedef struct
{
    int32_t raw_last;
    int32_t raw_avg;
    uint16_t sample_count;
    AppCS1238ChStatus_t status;
    uint32_t last_update_tick;
} AppCS1238ChannelData_t;

typedef struct
{
    AppCS1238ChannelData_t adc1_a;
    AppCS1238ChannelData_t adc1_b;
    AppCS1238ChannelData_t adc2_a;
    AppCS1238ChannelData_t adc2_b;

    uint8_t adc1_data_level;
    uint8_t adc2_data_level;
} AppCS1238Snapshot_t;

void App_CS1238_Init(void);
void App_CS1238_Task(void);
void App_CS1238_GetSnapshot(AppCS1238Snapshot_t *out);

#endif /* APP_CS1238_TASK_H */
