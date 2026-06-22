#include "app_odometry.h"

#define ODOM_MM_PER_TICK_DEFAULT      0.116f
#define ODOM_WHEEL_TRACK_MM_DEFAULT   150.0f
#define ODOM_DEG_PER_TURN_TICK_DEFAULT 0.090f

static float s_x_mm;
static float s_y_mm;
static float s_theta_rad;
static float s_forward_accum_mm;
static float s_turn_accum_deg;
static float s_mm_per_tick = ODOM_MM_PER_TICK_DEFAULT;
static float s_turn_deg_per_tick = ODOM_DEG_PER_TURN_TICK_DEFAULT;

void Odom_Init(void)
{
    (void)ODOM_WHEEL_TRACK_MM_DEFAULT;
    Odom_Reset(0.0f, 0.0f, 0.0f);
}

void Odom_Reset(float x_mm, float y_mm, float theta_rad)
{
    s_x_mm = x_mm;
    s_y_mm = y_mm;
    s_theta_rad = theta_rad;
    Odom_ResetLocalAccum();
}

void Odom_UpdateFromEncoderDelta(const int32_t delta[4])
{
    int32_t forward_1;
    int32_t forward_2;
    int32_t forward_3;
    int32_t forward_4;
    int32_t forward_avg_ticks;
    int32_t turn_avg_ticks;
    float forward_mm;
    float turn_deg;

    if (delta == 0)
    {
        return;
    }

    forward_1 = delta[0];
    forward_2 = -delta[1];
    forward_3 = delta[2];
    forward_4 = -delta[3];
    forward_avg_ticks = (forward_1 + forward_2 + forward_3 + forward_4) / 4;

    turn_avg_ticks = (delta[0] + delta[1] - delta[2] - delta[3]) / 4;

    forward_mm = (float)forward_avg_ticks * s_mm_per_tick;
    turn_deg = (float)turn_avg_ticks * s_turn_deg_per_tick;

    s_forward_accum_mm += forward_mm;
    s_turn_accum_deg += turn_deg;
    s_x_mm += forward_mm;
    s_theta_rad += turn_deg * 0.01745329252f;
}

void Odom_GetPose(float *x_mm, float *y_mm, float *theta_rad)
{
    if (x_mm != 0)
    {
        *x_mm = s_x_mm;
    }
    if (y_mm != 0)
    {
        *y_mm = s_y_mm;
    }
    if (theta_rad != 0)
    {
        *theta_rad = s_theta_rad;
    }
}

float Odom_GetForwardAccumMm(void)
{
    return s_forward_accum_mm;
}

float Odom_GetTurnAccumDeg(void)
{
    return s_turn_accum_deg;
}

void Odom_ResetLocalAccum(void)
{
    s_forward_accum_mm = 0.0f;
    s_turn_accum_deg = 0.0f;
}
