#ifndef APP_HMC5883L_H
#define APP_HMC5883L_H

#include <stdint.h>

typedef enum
{
    HMC_CAL_IDLE = 0,
    HMC_CAL_RUNNING,
    HMC_CAL_DONE,
    HMC_CAL_ERROR
} HMC5883L_CalState_t;

typedef struct
{
    float offset_a;
    float offset_b;
    float scale_a;
    float scale_b;
    float min_a;
    float max_a;
    float min_b;
    float max_b;
    float radius_a;
    float radius_b;
    uint32_t sample_count;
    uint8_t valid;
} HMC5883L_CalParam_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;

    float axis_a_raw;
    float axis_b_raw;
    float axis_a_cal;
    float axis_b_cal;

    float heading_raw_deg;
    float heading_deg;
    float delta_deg;

    uint8_t status_reg;
    uint8_t drdy_level;

    uint8_t id_a;
    uint8_t id_b;
    uint8_t id_c;

    uint8_t init_ok;
    uint8_t read_ok;

    HMC5883L_CalState_t cal_state;
    HMC5883L_CalParam_t cal;

    uint32_t sample_count;
    uint32_t last_update_tick;
} HMC5883L_Info_t;

void App_HMC5883L_Init(void);
void App_HMC5883L_Task(void);

void App_HMC5883L_ResetDeltaReference(void);
void App_HMC5883L_CalStart(void);
void App_HMC5883L_CalStopAndApply(void);
void App_HMC5883L_CalReset(void);
void App_HMC5883L_CalPrint(void);
void App_HMC5883L_SetCalibration(float offset_a,
                                 float offset_b,
                                 float scale_a,
                                 float scale_b);
uint8_t App_HMC5883L_IsReady(void);
float App_HMC5883L_GetHeadingDeg(void);
float App_HMC5883L_GetDeltaDeg(void);
const HMC5883L_Info_t *App_HMC5883L_GetInfo(void);
const HMC5883L_CalParam_t *App_HMC5883L_GetCalibration(void);

#endif /* APP_HMC5883L_H */
