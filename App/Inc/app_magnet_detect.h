#ifndef APP_MAGNET_DETECT_H
#define APP_MAGNET_DETECT_H

#include <stdint.h>

#define MAG_DETECT_CHANNEL_COUNT 4U

typedef enum
{
    MAG_STATE_INIT = 0,
    MAG_STATE_CALIBRATING,
    MAG_STATE_READY,
    MAG_STATE_SENSOR_ERROR
} MagState_t;

typedef enum
{
    MAG_TARGET_NONE = 0,
    MAG_TARGET_LEFT,
    MAG_TARGET_RIGHT,
    MAG_TARGET_CENTER,
    MAG_TARGET_UNKNOWN
} MagTarget_t;

typedef enum
{
    MAG_SIGNAL_IDLE = 0,
    MAG_SIGNAL_TURN_LEFT,
    MAG_SIGNAL_TURN_RIGHT,
    MAG_SIGNAL_STRAIGHT,
    MAG_SIGNAL_HOLD
} MagSignal_t;

typedef enum
{
    MAG_CH_ADC1_A = 0,
    MAG_CH_ADC1_B,
    MAG_CH_ADC2_A,
    MAG_CH_ADC2_B
} MagChannelIndex_t;

typedef struct
{
    MagState_t state;
    MagTarget_t target;
    MagTarget_t candidate;
    MagSignal_t signal;

    int32_t raw[MAG_DETECT_CHANNEL_COUNT];
    int32_t baseline[MAG_DETECT_CHANNEL_COUNT];
    int32_t delta[MAG_DETECT_CHANNEL_COUNT];
    int32_t strength[MAG_DETECT_CHANNEL_COUNT];
    int32_t filt_strength[MAG_DETECT_CHANNEL_COUNT];

    int32_t left_strength;
    int32_t right_strength;

    uint8_t detected;
    uint8_t holding;
    uint8_t confidence_percent;

    uint32_t valid_sample_count;
    uint32_t last_update_tick;
} MagDetectInfo_t;

void App_MagDetect_Init(void);
void App_MagDetect_Task(void);
void App_MagDetect_RequestRecalibration(void);
const MagDetectInfo_t *App_MagDetect_GetInfo(void);

#endif /* APP_MAGNET_DETECT_H */
