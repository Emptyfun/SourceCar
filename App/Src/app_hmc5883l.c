/**
 * @file app_hmc5883l.c
 * @brief HMC5883L USART1 bring-up task over PA0/PA1 software I2C.
 */
#include "app_hmc5883l.h"

#include "app_config.h"
#if APP_ENABLE_MOTOR_SERIAL
#include "app_motor_serial.h"
#endif
#include "bsp_debug_uart.h"
#include "bsp_soft_i2c_hmc.h"
#include "main.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define HMC5883L_ADDR_7BIT       0x1E
#define HMC5883L_ADDR_WRITE      0x3C
#define HMC5883L_ADDR_READ       0x3D

#define HMC5883L_REG_CONFIG_A    0x00
#define HMC5883L_REG_CONFIG_B    0x01
#define HMC5883L_REG_MODE        0x02

#define HMC5883L_REG_DATA_X_MSB  0x03
#define HMC5883L_REG_DATA_X_LSB  0x04
#define HMC5883L_REG_DATA_Z_MSB  0x05
#define HMC5883L_REG_DATA_Z_LSB  0x06
#define HMC5883L_REG_DATA_Y_MSB  0x07
#define HMC5883L_REG_DATA_Y_LSB  0x08

#define HMC5883L_REG_STATUS      0x09

#define HMC5883L_REG_ID_A        0x0A
#define HMC5883L_REG_ID_B        0x0B
#define HMC5883L_REG_ID_C        0x0C

#define HMC5883L_CONFIG_A_VALUE  0x70
#define HMC5883L_CONFIG_B_VALUE  0x20
#define HMC5883L_MODE_VALUE      0x00

#define HMC5883L_ID_A_VALUE      0x48
#define HMC5883L_ID_B_VALUE      0x34
#define HMC5883L_ID_C_VALUE      0x33

#define HMC5883L_READ_PERIOD_MS   50U
#define HMC5883L_PRINT_PERIOD_MS  200U
#define HMC5883L_REINIT_PERIOD_MS 1000U
#define HMC5883L_READY_TIMEOUT_MS 300U
#define HMC5883L_MAX_READ_FAILS   5U
#define HMC5883L_PI               3.1415926f

#ifndef HMC5883L_CAL_ENABLE
#define HMC5883L_CAL_ENABLE              1
#endif

#ifndef HMC5883L_CAL_PRINT_PERIOD_MS
#define HMC5883L_CAL_PRINT_PERIOD_MS     200U
#endif

#ifndef HMC5883L_CAL_MIN_SAMPLES
#define HMC5883L_CAL_MIN_SAMPLES         100U
#endif

#ifndef HMC5883L_CAL_MAX_DURATION_MS
#define HMC5883L_CAL_MAX_DURATION_MS     60000U
#endif

#ifndef HMC5883L_USE_CALIBRATION_DEFAULT
#define HMC5883L_USE_CALIBRATION_DEFAULT 1
#endif

#ifndef HMC5883L_HEADING_USE_XZ
#define HMC5883L_HEADING_USE_XZ          0
#endif

#ifndef HMC_CAL_OFFSET_A
#define HMC_CAL_OFFSET_A                 -162.0f
#endif

#ifndef HMC_CAL_OFFSET_B
#define HMC_CAL_OFFSET_B                 -41.0f
#endif

#ifndef HMC_CAL_SCALE_A
#define HMC_CAL_SCALE_A                  1.0511f
#endif

#ifndef HMC_CAL_SCALE_B
#define HMC_CAL_SCALE_B                  0.9536f
#endif

static uint8_t HMC5883L_WriteReg(uint8_t reg, uint8_t val);
static uint8_t HMC5883L_ReadReg(uint8_t reg, uint8_t *val);
static uint8_t HMC5883L_ReadMulti(uint8_t start_reg, uint8_t *buf, uint8_t len);
static uint8_t HMC5883L_ReadRaw(int16_t *x, int16_t *y, int16_t *z);

static void HMC5883L_SelectHeadingAxes(int16_t x, int16_t y, int16_t z, float *axis_a, float *axis_b);
static void HMC5883L_ApplyCalibration(float a_raw, float b_raw, float *a_cal, float *b_cal);
static void HMC5883L_UpdateCalibration(float a_raw, float b_raw, uint32_t now);
static void HMC5883L_ResetCalibrationStats(void);
static float HMC5883L_ComputeHeadingDeg(float axis_a, float axis_b);
static float HMC5883L_Normalize360(float deg);
static float HMC5883L_WrapDeltaDeg(float now_deg, float ref_deg);
static int32_t HMC5883L_FloatToDeci(float value);
static int32_t HMC5883L_AbsI32(int32_t value);
static void HMC5883L_PrintCalParam(const char *prefix);
static void HMC5883L_DebugPrintf(const char *fmt, ...);

static HMC5883L_Info_t s_hmc_info;
static uint32_t s_last_read_tick;
static uint32_t s_last_print_tick;
static uint32_t s_last_reinit_tick;
static uint32_t s_cal_start_tick;
static uint32_t s_last_cal_print_tick;
static float s_delta_ref_deg;
static uint8_t s_delta_ref_valid;
static uint8_t s_read_fail_count;

void App_HMC5883L_Init(void)
{
    uint8_t ack_ok;
    uint8_t cfg_a = 0U;
    uint8_t cfg_b = 0U;
    uint8_t mode = 0U;
    uint32_t now = HAL_GetTick();
    HMC5883L_CalParam_t saved_cal = s_hmc_info.cal;
    uint8_t saved_cal_valid = s_hmc_info.cal.valid;

    memset(&s_hmc_info, 0, sizeof(s_hmc_info));
    s_last_read_tick = now;
    s_last_print_tick = now;
    s_last_reinit_tick = now;
    s_cal_start_tick = 0UL;
    s_last_cal_print_tick = now;
    s_delta_ref_deg = 0.0f;
    s_delta_ref_valid = 0U;
    s_read_fail_count = 0U;

    if (saved_cal_valid != 0U)
    {
        s_hmc_info.cal = saved_cal;
    }
    else
    {
#if HMC5883L_USE_CALIBRATION_DEFAULT
        App_HMC5883L_SetCalibration(HMC_CAL_OFFSET_A,
                                    HMC_CAL_OFFSET_B,
                                    HMC_CAL_SCALE_A,
                                    HMC_CAL_SCALE_B);
#else
        App_HMC5883L_CalReset();
#endif
    }
    s_hmc_info.cal_state = HMC_CAL_IDLE;

    HMC5883L_DebugPrintf("[HMC] init start\r\n");

    SoftI2C_HMC_Init();
    HAL_Delay(10U);

    ack_ok = HMC5883L_ReadReg(HMC5883L_REG_ID_A, &s_hmc_info.id_a);
    ack_ok &= HMC5883L_ReadReg(HMC5883L_REG_ID_B, &s_hmc_info.id_b);
    ack_ok &= HMC5883L_ReadReg(HMC5883L_REG_ID_C, &s_hmc_info.id_c);

    if (ack_ok == 0U)
    {
        HMC5883L_DebugPrintf("[HMC] ID ERROR: no ACK\r\n");
        return;
    }

    HMC5883L_DebugPrintf("[HMC] ID: 0x%02X 0x%02X 0x%02X\r\n",
                         (unsigned int)s_hmc_info.id_a,
                         (unsigned int)s_hmc_info.id_b,
                         (unsigned int)s_hmc_info.id_c);

    if ((s_hmc_info.id_a != HMC5883L_ID_A_VALUE) ||
        (s_hmc_info.id_b != HMC5883L_ID_B_VALUE) ||
        (s_hmc_info.id_c != HMC5883L_ID_C_VALUE))
    {
        HMC5883L_DebugPrintf("[HMC] ID ERROR: 0x%02X 0x%02X 0x%02X\r\n",
                             (unsigned int)s_hmc_info.id_a,
                             (unsigned int)s_hmc_info.id_b,
                             (unsigned int)s_hmc_info.id_c);
        return;
    }

    ack_ok = HMC5883L_WriteReg(HMC5883L_REG_CONFIG_A, HMC5883L_CONFIG_A_VALUE);
    ack_ok &= HMC5883L_WriteReg(HMC5883L_REG_CONFIG_B, HMC5883L_CONFIG_B_VALUE);
    ack_ok &= HMC5883L_WriteReg(HMC5883L_REG_MODE, HMC5883L_MODE_VALUE);

    if (ack_ok == 0U)
    {
        HMC5883L_DebugPrintf("[HMC] cfg ERROR: no ACK\r\n");
        return;
    }

    ack_ok = HMC5883L_ReadReg(HMC5883L_REG_CONFIG_A, &cfg_a);
    ack_ok &= HMC5883L_ReadReg(HMC5883L_REG_CONFIG_B, &cfg_b);
    ack_ok &= HMC5883L_ReadReg(HMC5883L_REG_MODE, &mode);

    if (ack_ok == 0U)
    {
        HMC5883L_DebugPrintf("[HMC] cfg ERROR: no ACK\r\n");
        return;
    }

    HMC5883L_DebugPrintf("[HMC] cfgA=0x%02X cfgB=0x%02X mode=0x%02X\r\n",
                         (unsigned int)cfg_a,
                         (unsigned int)cfg_b,
                         (unsigned int)mode);

    if ((cfg_a != HMC5883L_CONFIG_A_VALUE) ||
        (cfg_b != HMC5883L_CONFIG_B_VALUE) ||
        (mode != HMC5883L_MODE_VALUE))
    {
        HMC5883L_DebugPrintf("[HMC] cfg ERROR: 0x%02X 0x%02X 0x%02X\r\n",
                             (unsigned int)cfg_a,
                             (unsigned int)cfg_b,
                             (unsigned int)mode);
        return;
    }

    s_hmc_info.init_ok = 1U;
    HMC5883L_DebugPrintf("[HMC] init OK\r\n");
}

void App_HMC5883L_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t status = 0U;
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    float axis_a_raw = 0.0f;
    float axis_b_raw = 0.0f;
    float axis_a_cal = 0.0f;
    float axis_b_cal = 0.0f;

    if (s_hmc_info.init_ok == 0U)
    {
        if ((uint32_t)(now - s_last_reinit_tick) >= HMC5883L_REINIT_PERIOD_MS)
        {
            App_HMC5883L_Init();
        }
        return;
    }

    if ((uint32_t)(now - s_last_read_tick) < HMC5883L_READ_PERIOD_MS)
    {
        return;
    }
    s_last_read_tick = now;

    s_hmc_info.drdy_level = SoftI2C_HMC_ReadDRDY();

    if ((HMC5883L_ReadReg(HMC5883L_REG_STATUS, &status) == 0U) ||
        (HMC5883L_ReadRaw(&x, &y, &z) == 0U))
    {
        s_hmc_info.read_ok = 0U;
        s_read_fail_count++;
        HMC5883L_DebugPrintf("[HMC] read failed\r\n");

        if (s_read_fail_count >= HMC5883L_MAX_READ_FAILS)
        {
            s_hmc_info.init_ok = 0U;
            s_last_reinit_tick = now;
        }
        return;
    }

    s_hmc_info.status_reg = status;
    s_hmc_info.x = x;
    s_hmc_info.y = y;
    s_hmc_info.z = z;

    HMC5883L_SelectHeadingAxes(x, y, z, &axis_a_raw, &axis_b_raw);
    s_hmc_info.axis_a_raw = axis_a_raw;
    s_hmc_info.axis_b_raw = axis_b_raw;
    s_hmc_info.heading_raw_deg = HMC5883L_ComputeHeadingDeg(axis_a_raw, axis_b_raw);

    HMC5883L_UpdateCalibration(axis_a_raw, axis_b_raw, now);
    HMC5883L_ApplyCalibration(axis_a_raw, axis_b_raw, &axis_a_cal, &axis_b_cal);

    s_hmc_info.axis_a_cal = axis_a_cal;
    s_hmc_info.axis_b_cal = axis_b_cal;
    s_hmc_info.heading_deg = HMC5883L_ComputeHeadingDeg(axis_a_cal, axis_b_cal);

    if (s_delta_ref_valid == 0U)
    {
        s_delta_ref_deg = s_hmc_info.heading_deg;
        s_delta_ref_valid = 1U;
    }

    s_hmc_info.delta_deg = HMC5883L_WrapDeltaDeg(s_hmc_info.heading_deg, s_delta_ref_deg);
    s_hmc_info.read_ok = 1U;
    s_hmc_info.sample_count++;
    s_hmc_info.last_update_tick = now;
    s_read_fail_count = 0U;

    if ((APP_HMC5883L_VERBOSE_PRINT != 0) &&
        ((uint32_t)(now - s_last_print_tick) >= HMC5883L_PRINT_PERIOD_MS))
    {
        int32_t heading_deci = HMC5883L_FloatToDeci(s_hmc_info.heading_deg);
        int32_t raw_heading_deci = HMC5883L_FloatToDeci(s_hmc_info.heading_raw_deg);
        int32_t delta_deci = HMC5883L_FloatToDeci(s_hmc_info.delta_deg);
        int32_t delta_abs_deci = HMC5883L_AbsI32(delta_deci);

        s_last_print_tick = now;
        if (heading_deci >= 3600)
        {
            heading_deci = 0;
        }
        if (raw_heading_deci >= 3600)
        {
            raw_heading_deci = 0;
        }

        HMC5883L_DebugPrintf("[HMC] drdy=%u st=0x%02X x=%d y=%d z=%d raw=%ld.%01ld cal=%ld.%01ld delta=%s%ld.%01ld cnt=%lu\r\n",
                             (unsigned int)s_hmc_info.drdy_level,
                             (unsigned int)s_hmc_info.status_reg,
                             (int)s_hmc_info.x,
                             (int)s_hmc_info.y,
                             (int)s_hmc_info.z,
                             (long)(raw_heading_deci / 10),
                             (long)(raw_heading_deci % 10),
                             (long)(heading_deci / 10),
                             (long)(heading_deci % 10),
                             (delta_deci < 0) ? "-" : "",
                             (long)(delta_abs_deci / 10),
                             (long)(delta_abs_deci % 10),
                             (unsigned long)s_hmc_info.sample_count);
    }
}

void App_HMC5883L_ResetDeltaReference(void)
{
    s_delta_ref_deg = s_hmc_info.heading_deg;
    s_hmc_info.delta_deg = 0.0f;
    s_delta_ref_valid = 1U;
}

void App_HMC5883L_CalStart(void)
{
#if HMC5883L_CAL_ENABLE
    HMC5883L_ResetCalibrationStats();
    s_hmc_info.cal_state = HMC_CAL_RUNNING;
    s_hmc_info.cal.valid = 0U;
    s_cal_start_tick = HAL_GetTick();
    s_last_cal_print_tick = s_cal_start_tick;
    HMC5883L_DebugPrintf("[HMC-CAL] start rotate sensor/car slowly for 1-2 circles\r\n");
#else
    HMC5883L_DebugPrintf("[HMC-CAL] disabled\r\n");
#endif
}

void App_HMC5883L_CalStopAndApply(void)
{
    float radius_a;
    float radius_b;
    float offset_a;
    float offset_b;
    float avg_radius;

    if (s_hmc_info.cal_state != HMC_CAL_RUNNING)
    {
        HMC5883L_DebugPrintf("[HMC-CAL] not running\r\n");
        HMC5883L_PrintCalParam("[HMC-CAL]");
        return;
    }

    radius_a = (s_hmc_info.cal.max_a - s_hmc_info.cal.min_a) * 0.5f;
    radius_b = (s_hmc_info.cal.max_b - s_hmc_info.cal.min_b) * 0.5f;

    if (s_hmc_info.cal.sample_count < HMC5883L_CAL_MIN_SAMPLES)
    {
        s_hmc_info.cal_state = HMC_CAL_ERROR;
        HMC5883L_DebugPrintf("[HMC-CAL] failed samples=%lu min=%lu\r\n",
                             (unsigned long)s_hmc_info.cal.sample_count,
                             (unsigned long)HMC5883L_CAL_MIN_SAMPLES);
        return;
    }

    if ((radius_a <= 1.0f) || (radius_b <= 1.0f))
    {
        s_hmc_info.cal_state = HMC_CAL_ERROR;
        HMC5883L_DebugPrintf("[HMC-CAL] failed radiusA=%ld radiusB=%ld\r\n",
                             (long)radius_a,
                             (long)radius_b);
        return;
    }

    offset_a = (s_hmc_info.cal.max_a + s_hmc_info.cal.min_a) * 0.5f;
    offset_b = (s_hmc_info.cal.max_b + s_hmc_info.cal.min_b) * 0.5f;
    avg_radius = (radius_a + radius_b) * 0.5f;

    s_hmc_info.cal.offset_a = offset_a;
    s_hmc_info.cal.offset_b = offset_b;
    s_hmc_info.cal.radius_a = radius_a;
    s_hmc_info.cal.radius_b = radius_b;
    s_hmc_info.cal.scale_a = avg_radius / radius_a;
    s_hmc_info.cal.scale_b = avg_radius / radius_b;
    s_hmc_info.cal.valid = 1U;
    s_hmc_info.cal_state = HMC_CAL_DONE;

    HMC5883L_ApplyCalibration(s_hmc_info.axis_a_raw,
                              s_hmc_info.axis_b_raw,
                              &s_hmc_info.axis_a_cal,
                              &s_hmc_info.axis_b_cal);
    s_hmc_info.heading_deg = HMC5883L_ComputeHeadingDeg(s_hmc_info.axis_a_cal,
                                                        s_hmc_info.axis_b_cal);
    App_HMC5883L_ResetDeltaReference();

    HMC5883L_DebugPrintf("[HMC-CAL] done samples=%lu\r\n",
                         (unsigned long)s_hmc_info.cal.sample_count);
    HMC5883L_PrintCalParam("[HMC-CAL]");
}

void App_HMC5883L_CalReset(void)
{
    memset(&s_hmc_info.cal, 0, sizeof(s_hmc_info.cal));
    s_hmc_info.cal.scale_a = 1.0f;
    s_hmc_info.cal.scale_b = 1.0f;
    s_hmc_info.cal_state = HMC_CAL_IDLE;
    HMC5883L_DebugPrintf("[HMC-CAL] reset offset=0 scale=1\r\n");
}

void App_HMC5883L_CalPrint(void)
{
    HMC5883L_PrintCalParam("[HMC-CAL]");
}

void App_HMC5883L_SetCalibration(float offset_a,
                                 float offset_b,
                                 float scale_a,
                                 float scale_b)
{
    s_hmc_info.cal.offset_a = offset_a;
    s_hmc_info.cal.offset_b = offset_b;
    s_hmc_info.cal.scale_a = (scale_a > 0.0f) ? scale_a : 1.0f;
    s_hmc_info.cal.scale_b = (scale_b > 0.0f) ? scale_b : 1.0f;
    s_hmc_info.cal.valid = 1U;
    s_hmc_info.cal_state = HMC_CAL_IDLE;
}

uint8_t App_HMC5883L_IsReady(void)
{
    uint32_t now = HAL_GetTick();

    if ((s_hmc_info.init_ok == 0U) || (s_hmc_info.read_ok == 0U))
    {
        return 0U;
    }

    return ((uint32_t)(now - s_hmc_info.last_update_tick) <= HMC5883L_READY_TIMEOUT_MS) ? 1U : 0U;
}

float App_HMC5883L_GetHeadingDeg(void)
{
    return s_hmc_info.heading_deg;
}

float App_HMC5883L_GetDeltaDeg(void)
{
    return s_hmc_info.delta_deg;
}

const HMC5883L_Info_t *App_HMC5883L_GetInfo(void)
{
    return &s_hmc_info;
}

const HMC5883L_CalParam_t *App_HMC5883L_GetCalibration(void)
{
    return &s_hmc_info.cal;
}

static uint8_t HMC5883L_WriteReg(uint8_t reg, uint8_t val)
{
    SoftI2C_HMC_Start();
    if (SoftI2C_HMC_WriteByte(HMC5883L_ADDR_WRITE) == 0U)
    {
        SoftI2C_HMC_Stop();
        return 0U;
    }

    if (SoftI2C_HMC_WriteByte(reg) == 0U)
    {
        SoftI2C_HMC_Stop();
        return 0U;
    }

    if (SoftI2C_HMC_WriteByte(val) == 0U)
    {
        SoftI2C_HMC_Stop();
        return 0U;
    }

    SoftI2C_HMC_Stop();

    return 1U;
}

static uint8_t HMC5883L_ReadReg(uint8_t reg, uint8_t *val)
{
    if (val == NULL)
    {
        return 0U;
    }

    SoftI2C_HMC_Start();
    if (SoftI2C_HMC_WriteByte(HMC5883L_ADDR_WRITE) == 0U)
    {
        SoftI2C_HMC_Stop();
        return 0U;
    }

    if (SoftI2C_HMC_WriteByte(reg) == 0U)
    {
        SoftI2C_HMC_Stop();
        return 0U;
    }

    SoftI2C_HMC_Start();
    if (SoftI2C_HMC_WriteByte(HMC5883L_ADDR_READ) == 0U)
    {
        SoftI2C_HMC_Stop();
        return 0U;
    }

    *val = SoftI2C_HMC_ReadByte(0U);
    SoftI2C_HMC_Stop();

    return 1U;
}

static uint8_t HMC5883L_ReadMulti(uint8_t start_reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    if ((buf == NULL) || (len == 0U))
    {
        return 0U;
    }

    SoftI2C_HMC_Start();
    if (SoftI2C_HMC_WriteByte(HMC5883L_ADDR_WRITE) == 0U)
    {
        SoftI2C_HMC_Stop();
        return 0U;
    }

    if (SoftI2C_HMC_WriteByte(start_reg) == 0U)
    {
        SoftI2C_HMC_Stop();
        return 0U;
    }

    SoftI2C_HMC_Start();
    if (SoftI2C_HMC_WriteByte(HMC5883L_ADDR_READ) == 0U)
    {
        SoftI2C_HMC_Stop();
        return 0U;
    }

    for (i = 0U; i < len; i++)
    {
        buf[i] = SoftI2C_HMC_ReadByte(((uint8_t)(i + 1U) < len) ? 1U : 0U);
    }

    SoftI2C_HMC_Stop();

    return 1U;
}

static uint8_t HMC5883L_ReadRaw(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];

    if ((x == NULL) || (y == NULL) || (z == NULL))
    {
        return 0U;
    }

    if (HMC5883L_ReadMulti(HMC5883L_REG_DATA_X_MSB, buf, sizeof(buf)) == 0U)
    {
        return 0U;
    }

    *x = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    *z = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    *y = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

    return 1U;
}

static void HMC5883L_SelectHeadingAxes(int16_t x, int16_t y, int16_t z, float *axis_a, float *axis_b)
{
    if ((axis_a == NULL) || (axis_b == NULL))
    {
        return;
    }

#if HMC5883L_HEADING_USE_XZ
    (void)y;
    *axis_a = (float)x;
    *axis_b = (float)z;
#else
    (void)z;
    *axis_a = (float)x;
    *axis_b = (float)y;
#endif
}

static void HMC5883L_ApplyCalibration(float a_raw, float b_raw, float *a_cal, float *b_cal)
{
    if ((a_cal == NULL) || (b_cal == NULL))
    {
        return;
    }

    if (s_hmc_info.cal.valid != 0U)
    {
        *a_cal = (a_raw - s_hmc_info.cal.offset_a) * s_hmc_info.cal.scale_a;
        *b_cal = (b_raw - s_hmc_info.cal.offset_b) * s_hmc_info.cal.scale_b;
    }
    else
    {
        *a_cal = a_raw;
        *b_cal = b_raw;
    }
}

static void HMC5883L_UpdateCalibration(float a_raw, float b_raw, uint32_t now)
{
    HMC5883L_CalParam_t *cal = &s_hmc_info.cal;

    if (s_hmc_info.cal_state != HMC_CAL_RUNNING)
    {
        return;
    }

    if (a_raw < cal->min_a) { cal->min_a = a_raw; }
    if (a_raw > cal->max_a) { cal->max_a = a_raw; }
    if (b_raw < cal->min_b) { cal->min_b = b_raw; }
    if (b_raw > cal->max_b) { cal->max_b = b_raw; }
    cal->sample_count++;

    if ((uint32_t)(now - s_cal_start_tick) >= HMC5883L_CAL_MAX_DURATION_MS)
    {
        App_HMC5883L_CalStopAndApply();
        return;
    }

    if ((uint32_t)(now - s_last_cal_print_tick) >= HMC5883L_CAL_PRINT_PERIOD_MS)
    {
        s_last_cal_print_tick = now;
        HMC5883L_DebugPrintf("[HMC-CAL] n=%lu a=%ld b=%ld minA=%ld maxA=%ld minB=%ld maxB=%ld\r\n",
                             (unsigned long)cal->sample_count,
                             (long)a_raw,
                             (long)b_raw,
                             (long)cal->min_a,
                             (long)cal->max_a,
                             (long)cal->min_b,
                             (long)cal->max_b);
    }
}

static void HMC5883L_ResetCalibrationStats(void)
{
    s_hmc_info.cal.min_a = 1000000.0f;
    s_hmc_info.cal.max_a = -1000000.0f;
    s_hmc_info.cal.min_b = 1000000.0f;
    s_hmc_info.cal.max_b = -1000000.0f;
    s_hmc_info.cal.radius_a = 0.0f;
    s_hmc_info.cal.radius_b = 0.0f;
    s_hmc_info.cal.sample_count = 0UL;
}

static float HMC5883L_ComputeHeadingDeg(float axis_a, float axis_b)
{
    return HMC5883L_Normalize360(atan2f(axis_b, axis_a) * 180.0f / HMC5883L_PI);
}

static float HMC5883L_Normalize360(float deg)
{
    while (deg < 0.0f)
    {
        deg += 360.0f;
    }

    while (deg >= 360.0f)
    {
        deg -= 360.0f;
    }

    return deg;
}

static float HMC5883L_WrapDeltaDeg(float now_deg, float ref_deg)
{
    float d = now_deg - ref_deg;

    while (d > 180.0f)
    {
        d -= 360.0f;
    }

    while (d < -180.0f)
    {
        d += 360.0f;
    }

    return d;
}

static int32_t HMC5883L_FloatToDeci(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * 10.0f) + 0.5f);
    }

    return (int32_t)((value * 10.0f) - 0.5f);
}

static int32_t HMC5883L_AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void HMC5883L_PrintCalParam(const char *prefix)
{
    int32_t offset_a_deci = HMC5883L_FloatToDeci(s_hmc_info.cal.offset_a);
    int32_t offset_b_deci = HMC5883L_FloatToDeci(s_hmc_info.cal.offset_b);
    int32_t radius_a_deci = HMC5883L_FloatToDeci(s_hmc_info.cal.radius_a);
    int32_t radius_b_deci = HMC5883L_FloatToDeci(s_hmc_info.cal.radius_b);
    int32_t scale_a_10000 = (int32_t)((s_hmc_info.cal.scale_a * 10000.0f) + 0.5f);
    int32_t scale_b_10000 = (int32_t)((s_hmc_info.cal.scale_b * 10000.0f) + 0.5f);
    int32_t offset_a_abs = HMC5883L_AbsI32(offset_a_deci);
    int32_t offset_b_abs = HMC5883L_AbsI32(offset_b_deci);
    int32_t radius_a_abs = HMC5883L_AbsI32(radius_a_deci);
    int32_t radius_b_abs = HMC5883L_AbsI32(radius_b_deci);

    HMC5883L_DebugPrintf("%s valid=%u samples=%lu offsetA=%s%ld.%01ld offsetB=%s%ld.%01ld\r\n",
                         prefix,
                         (unsigned int)s_hmc_info.cal.valid,
                         (unsigned long)s_hmc_info.cal.sample_count,
                         (offset_a_deci < 0) ? "-" : "",
                         (long)(offset_a_abs / 10),
                         (long)(offset_a_abs % 10),
                         (offset_b_deci < 0) ? "-" : "",
                         (long)(offset_b_abs / 10),
                         (long)(offset_b_abs % 10));
    HMC5883L_DebugPrintf("%s radiusA=%ld.%01ld radiusB=%ld.%01ld scaleA=%ld.%04ld scaleB=%ld.%04ld\r\n",
                         prefix,
                         (long)(radius_a_abs / 10),
                         (long)(radius_a_abs % 10),
                         (long)(radius_b_abs / 10),
                         (long)(radius_b_abs % 10),
                         (long)(scale_a_10000 / 10000),
                         (long)(scale_a_10000 % 10000),
                         (long)(scale_b_10000 / 10000),
                         (long)(scale_b_10000 % 10000));
    HMC5883L_DebugPrintf("%s paste: #define HMC_CAL_OFFSET_A %s%ld.%01ldf\r\n",
                         prefix,
                         (offset_a_deci < 0) ? "-" : "",
                         (long)(offset_a_abs / 10),
                         (long)(offset_a_abs % 10));
    HMC5883L_DebugPrintf("%s paste: #define HMC_CAL_OFFSET_B %s%ld.%01ldf\r\n",
                         prefix,
                         (offset_b_deci < 0) ? "-" : "",
                         (long)(offset_b_abs / 10),
                         (long)(offset_b_abs % 10));
    HMC5883L_DebugPrintf("%s paste: #define HMC_CAL_SCALE_A %ld.%04ldf\r\n",
                         prefix,
                         (long)(scale_a_10000 / 10000),
                         (long)(scale_a_10000 % 10000));
    HMC5883L_DebugPrintf("%s paste: #define HMC_CAL_SCALE_B %ld.%04ldf\r\n",
                         prefix,
                         (long)(scale_b_10000 / 10000),
                         (long)(scale_b_10000 % 10000));
}

static void HMC5883L_DebugPrintf(const char *fmt, ...)
{
    char buf[180];
    va_list args;
    int len;

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0)
    {
        return;
    }

    buf[sizeof(buf) - 1U] = '\0';
#if APP_ENABLE_MOTOR_SERIAL
    MotorSerial_Printf("%s", buf);
#else
    BSP_DebugUart_SendString(buf);
#endif
}
