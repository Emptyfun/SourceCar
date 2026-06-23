#include "app_mag_calib.h"

#include "app_config.h"
#include "app_hmc5883l.h"
#include "app_motion.h"
#include "app_motor_serial.h"
#if APP_ENABLE_MOTION_PRIMITIVE
#include "app_motion_primitive.h"
#endif
#include "stm32f1xx_hal.h"

#define MAG_CALIB_DEFAULT_WZ       40
#define MAG_CALIB_MIN_WZ           15
#define MAG_CALIB_MAX_WZ           100
#define MAG_CALIB_PRINT_PERIOD_MS  100U
#define MAG_CALIB_MAX_RUN_MS       45000U
#define MAG_CALIB_MIN_SAMPLES      80U

typedef struct
{
    int16_t min_x;
    int16_t max_x;
    int16_t min_y;
    int16_t max_y;
    int16_t min_z;
    int16_t max_z;
    uint32_t samples;
    uint8_t valid;
} MagCalibStats_t;

static MagCalibState_t s_state;
static MagCalibStats_t s_stats;
static int8_t s_dir_sign;
static int16_t s_wz_abs;
static uint32_t s_start_tick;
static uint32_t s_last_print_tick;
static uint32_t s_last_sample_count;

static void MagCalib_ResetStats(void);
static void MagCalib_Begin(void);
static void MagCalib_UpdateStats(const HMC5883L_Info_t *info);
static void MagCalib_PrintSample(uint32_t now, const HMC5883L_Info_t *info);
static void MagCalib_StopWithState(MagCalibState_t next_state, const char *reason);
static void MagCalib_PrintResult(const char *reason, uint32_t now);
static int16_t MagCalib_ClampWz(int16_t wz_abs);
static int32_t MagCalib_FloatToDeci(float value);
static int32_t MagCalib_AbsI32(int32_t value);
static int32_t MagCalib_RoundDiv2(int32_t value);
static int32_t MagCalib_Scale1000(int32_t numerator, int32_t denominator);
static const char *MagCalib_StateText(MagCalibState_t state);
static char MagCalib_DirChar(void);

void App_MagCalib_Init(void)
{
    s_state = MAG_CALIB_STATE_IDLE;
    s_dir_sign = 1;
    s_wz_abs = MAG_CALIB_DEFAULT_WZ;
    s_start_tick = 0UL;
    s_last_print_tick = 0UL;
    s_last_sample_count = 0UL;
    MagCalib_ResetStats();
    MotorSerial_Printf("[MCAL] ready K1=dir K2=start/stop serial=mcal help\r\n");
}

void App_MagCalib_Task(void)
{
    uint32_t now = HAL_GetTick();
    const HMC5883L_Info_t *info;

    if (s_state == MAG_CALIB_STATE_PENDING)
    {
        if ((MotorSerial_IsInitDone() != 0U) && (App_HMC5883L_IsReady() != 0U))
        {
            MagCalib_Begin();
        }
        return;
    }

    if (s_state != MAG_CALIB_STATE_RUNNING)
    {
        return;
    }

    if (App_HMC5883L_IsReady() == 0U)
    {
        MagCalib_StopWithState(MAG_CALIB_STATE_ERROR, "hmc-lost");
        return;
    }

    if ((uint32_t)(now - s_start_tick) >= MAG_CALIB_MAX_RUN_MS)
    {
        MagCalib_StopWithState(MAG_CALIB_STATE_DONE, "timeout");
        return;
    }

    info = App_HMC5883L_GetInfo();
    if (info->sample_count == s_last_sample_count)
    {
        return;
    }

    s_last_sample_count = info->sample_count;
    MagCalib_UpdateStats(info);

    if ((uint32_t)(now - s_last_print_tick) >= MAG_CALIB_PRINT_PERIOD_MS)
    {
        s_last_print_tick = now;
        MagCalib_PrintSample(now, info);
    }
}

void App_MagCalib_Start(void)
{
    if (s_state == MAG_CALIB_STATE_RUNNING)
    {
        MotorSerial_Printf("[MCAL] already running dir=%c wz=%d\r\n",
                           MagCalib_DirChar(),
                           (int)s_wz_abs);
        return;
    }

    if ((MotorSerial_IsInitDone() == 0U) || (App_HMC5883L_IsReady() == 0U))
    {
        s_state = MAG_CALIB_STATE_PENDING;
        MotorSerial_Printf("[MCAL] start pending wait motor/HMC dir=%c wz=%d\r\n",
                           MagCalib_DirChar(),
                           (int)s_wz_abs);
        return;
    }

    MagCalib_Begin();
}

void App_MagCalib_Stop(void)
{
    if ((s_state == MAG_CALIB_STATE_RUNNING) || (s_state == MAG_CALIB_STATE_PENDING))
    {
        MagCalib_StopWithState(MAG_CALIB_STATE_STOP, "manual");
        return;
    }

    Motion_Stop();
    MotorSerial_EmergencyStop();
    s_state = MAG_CALIB_STATE_STOP;
    MotorSerial_Printf("[MCAL] stop idle\r\n");
}

void App_MagCalib_Toggle(void)
{
    if ((s_state == MAG_CALIB_STATE_RUNNING) || (s_state == MAG_CALIB_STATE_PENDING))
    {
        App_MagCalib_Stop();
    }
    else
    {
        App_MagCalib_Start();
    }
}

void App_MagCalib_ToggleDirection(void)
{
    uint8_t restart = ((s_state == MAG_CALIB_STATE_RUNNING) ||
                       (s_state == MAG_CALIB_STATE_PENDING)) ? 1U : 0U;

    if (restart != 0U)
    {
        MagCalib_StopWithState(MAG_CALIB_STATE_DONE, "dir-change");
    }

    s_dir_sign = (s_dir_sign > 0) ? -1 : 1;
    MotorSerial_Printf("[MCAL] dir=%c\r\n", MagCalib_DirChar());

    if (restart != 0U)
    {
        App_MagCalib_Start();
    }
}

void App_MagCalib_SetDirection(int8_t dir_sign)
{
    uint8_t restart = ((s_state == MAG_CALIB_STATE_RUNNING) ||
                       (s_state == MAG_CALIB_STATE_PENDING)) ? 1U : 0U;
    int8_t next_dir = (dir_sign >= 0) ? 1 : -1;

    if (s_dir_sign == next_dir)
    {
        MotorSerial_Printf("[MCAL] dir=%c\r\n", MagCalib_DirChar());
        return;
    }

    if (restart != 0U)
    {
        MagCalib_StopWithState(MAG_CALIB_STATE_DONE, "dir-change");
    }

    s_dir_sign = next_dir;
    MotorSerial_Printf("[MCAL] dir=%c\r\n", MagCalib_DirChar());

    if (restart != 0U)
    {
        App_MagCalib_Start();
    }
}

void App_MagCalib_SetTurnSpeed(int16_t wz_abs)
{
    s_wz_abs = MagCalib_ClampWz(wz_abs);
    MotorSerial_Printf("[MCAL] speed wz=%d\r\n", (int)s_wz_abs);

    if (s_state == MAG_CALIB_STATE_RUNNING)
    {
        Motion_SetVxWz(0, (int16_t)(s_dir_sign * s_wz_abs));
    }
}

void App_MagCalib_PrintStatus(void)
{
    MotorSerial_Printf("[MCAL] status state=%s dir=%c wz=%d samples=%lu\r\n",
                       MagCalib_StateText(s_state),
                       MagCalib_DirChar(),
                       (int)s_wz_abs,
                       (unsigned long)s_stats.samples);
}

uint8_t App_MagCalib_IsRunning(void)
{
    return ((s_state == MAG_CALIB_STATE_RUNNING) ||
            (s_state == MAG_CALIB_STATE_PENDING)) ? 1U : 0U;
}

static void MagCalib_ResetStats(void)
{
    s_stats.min_x = 0;
    s_stats.max_x = 0;
    s_stats.min_y = 0;
    s_stats.max_y = 0;
    s_stats.min_z = 0;
    s_stats.max_z = 0;
    s_stats.samples = 0UL;
    s_stats.valid = 0U;
}

static void MagCalib_Begin(void)
{
    const HMC5883L_Info_t *info = App_HMC5883L_GetInfo();

#if APP_ENABLE_MOTION_PRIMITIVE
    MotionPrimitive_Abort();
#endif
    MagCalib_ResetStats();
    App_HMC5883L_CalStart();
    s_start_tick = HAL_GetTick();
    s_last_print_tick = s_start_tick;
    s_last_sample_count = info->sample_count;
    s_state = MAG_CALIB_STATE_RUNNING;
    Motion_SetVxWz(0, (int16_t)(s_dir_sign * s_wz_abs));

    MotorSerial_Printf("[MCAL] start dir=%c wz=%d max_ms=%lu\r\n",
                       MagCalib_DirChar(),
                       (int)s_wz_abs,
                       (unsigned long)MAG_CALIB_MAX_RUN_MS);
    MotorSerial_Printf("[MCAL] fields n ms dir x y z head xmin xmax ymin ymax zmin zmax\r\n");
}

static void MagCalib_UpdateStats(const HMC5883L_Info_t *info)
{
    if (s_stats.valid == 0U)
    {
        s_stats.min_x = info->x;
        s_stats.max_x = info->x;
        s_stats.min_y = info->y;
        s_stats.max_y = info->y;
        s_stats.min_z = info->z;
        s_stats.max_z = info->z;
        s_stats.valid = 1U;
    }
    else
    {
        if (info->x < s_stats.min_x) { s_stats.min_x = info->x; }
        if (info->x > s_stats.max_x) { s_stats.max_x = info->x; }
        if (info->y < s_stats.min_y) { s_stats.min_y = info->y; }
        if (info->y > s_stats.max_y) { s_stats.max_y = info->y; }
        if (info->z < s_stats.min_z) { s_stats.min_z = info->z; }
        if (info->z > s_stats.max_z) { s_stats.max_z = info->z; }
    }

    s_stats.samples++;
}

static void MagCalib_PrintSample(uint32_t now, const HMC5883L_Info_t *info)
{
    int32_t heading_deci = MagCalib_FloatToDeci(info->heading_deg);
    int32_t heading_abs = MagCalib_AbsI32(heading_deci);

    MotorSerial_Printf("[MCAL] sample n=%lu ms=%lu dir=%c x=%d y=%d z=%d head=%s%ld.%01ld xmin=%d xmax=%d ymin=%d ymax=%d zmin=%d zmax=%d\r\n",
                       (unsigned long)s_stats.samples,
                       (unsigned long)(now - s_start_tick),
                       MagCalib_DirChar(),
                       (int)info->x,
                       (int)info->y,
                       (int)info->z,
                       (heading_deci < 0) ? "-" : "",
                       (long)(heading_abs / 10),
                       (long)(heading_abs % 10),
                       (int)s_stats.min_x,
                       (int)s_stats.max_x,
                       (int)s_stats.min_y,
                       (int)s_stats.max_y,
                       (int)s_stats.min_z,
                       (int)s_stats.max_z);
}

static void MagCalib_StopWithState(MagCalibState_t next_state, const char *reason)
{
    uint32_t now = HAL_GetTick();

    Motion_Stop();
    MotorSerial_EmergencyStop();
    s_state = next_state;
    if (next_state == MAG_CALIB_STATE_ERROR)
    {
        App_HMC5883L_CalReset();
    }
    else
    {
        App_HMC5883L_CalStopAndApply();
    }
    MagCalib_PrintResult(reason, now);
}

static void MagCalib_PrintResult(const char *reason, uint32_t now)
{
    int32_t x_off;
    int32_t y_off;
    int32_t z_off;
    int32_t x_rad;
    int32_t y_rad;
    int32_t z_rad;
    int32_t xy_avg_rad;
    int32_t x_scale;
    int32_t y_scale;

    if (s_stats.valid == 0U)
    {
        MotorSerial_Printf("[MCAL] result reason=%s samples=0 no-data\r\n", reason);
        return;
    }

    x_off = MagCalib_RoundDiv2((int32_t)s_stats.min_x + (int32_t)s_stats.max_x);
    y_off = MagCalib_RoundDiv2((int32_t)s_stats.min_y + (int32_t)s_stats.max_y);
    z_off = MagCalib_RoundDiv2((int32_t)s_stats.min_z + (int32_t)s_stats.max_z);
    x_rad = ((int32_t)s_stats.max_x - (int32_t)s_stats.min_x) / 2;
    y_rad = ((int32_t)s_stats.max_y - (int32_t)s_stats.min_y) / 2;
    z_rad = ((int32_t)s_stats.max_z - (int32_t)s_stats.min_z) / 2;
    xy_avg_rad = (x_rad + y_rad) / 2;
    x_scale = MagCalib_Scale1000(xy_avg_rad, x_rad);
    y_scale = MagCalib_Scale1000(xy_avg_rad, y_rad);

    MotorSerial_Printf("[MCAL] result reason=%s dir=%c samples=%lu ms=%lu state=%s\r\n",
                       reason,
                       MagCalib_DirChar(),
                       (unsigned long)s_stats.samples,
                       (unsigned long)(now - s_start_tick),
                       MagCalib_StateText(s_state));
    MotorSerial_Printf("[MCAL] range x=%d..%d y=%d..%d z=%d..%d\r\n",
                       (int)s_stats.min_x,
                       (int)s_stats.max_x,
                       (int)s_stats.min_y,
                       (int)s_stats.max_y,
                       (int)s_stats.min_z,
                       (int)s_stats.max_z);
    MotorSerial_Printf("[MCAL] calib2d x_off=%ld y_off=%ld x_rad=%ld y_rad=%ld x_scale=%ld y_scale=%ld\r\n",
                       (long)x_off,
                       (long)y_off,
                       (long)x_rad,
                       (long)y_rad,
                       (long)x_scale,
                       (long)y_scale);
    MotorSerial_Printf("[MCAL] calib3d z_off=%ld z_rad=%ld formula xh=(x-x_off)*x_scale/1000 yh=(y-y_off)*y_scale/1000\r\n",
                       (long)z_off,
                       (long)z_rad);

    if (s_stats.samples < MAG_CALIB_MIN_SAMPLES)
    {
        MotorSerial_Printf("[MCAL] warn samples<%lu rotate slower/longer for full circle\r\n",
                           (unsigned long)MAG_CALIB_MIN_SAMPLES);
    }
}

static int16_t MagCalib_ClampWz(int16_t wz_abs)
{
    if (wz_abs < 0)
    {
        wz_abs = (int16_t)-wz_abs;
    }

    if (wz_abs < MAG_CALIB_MIN_WZ)
    {
        return MAG_CALIB_MIN_WZ;
    }

    if (wz_abs > MAG_CALIB_MAX_WZ)
    {
        return MAG_CALIB_MAX_WZ;
    }

    return wz_abs;
}

static int32_t MagCalib_FloatToDeci(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * 10.0f) + 0.5f);
    }

    return (int32_t)((value * 10.0f) - 0.5f);
}

static int32_t MagCalib_AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t MagCalib_RoundDiv2(int32_t value)
{
    if (value >= 0)
    {
        return (value + 1) / 2;
    }

    return (value - 1) / 2;
}

static int32_t MagCalib_Scale1000(int32_t numerator, int32_t denominator)
{
    if (denominator <= 0)
    {
        return 1000L;
    }

    return (numerator * 1000L + (denominator / 2L)) / denominator;
}

static const char *MagCalib_StateText(MagCalibState_t state)
{
    switch (state)
    {
    case MAG_CALIB_STATE_PENDING:
        return "PENDING";

    case MAG_CALIB_STATE_RUNNING:
        return "RUNNING";

    case MAG_CALIB_STATE_DONE:
        return "DONE";

    case MAG_CALIB_STATE_ERROR:
        return "ERROR";

    case MAG_CALIB_STATE_STOP:
        return "STOP";

    case MAG_CALIB_STATE_IDLE:
    default:
        return "IDLE";
    }
}

static char MagCalib_DirChar(void)
{
    return (s_dir_sign >= 0) ? 'R' : 'L';
}
