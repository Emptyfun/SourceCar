#ifndef APP_ODOMETRY_H
#define APP_ODOMETRY_H

#include <stdint.h>

void Odom_Init(void);
void Odom_Reset(float x_mm, float y_mm, float theta_rad);
void Odom_UpdateFromEncoderDelta(const int32_t delta[4]);
void Odom_GetPose(float *x_mm, float *y_mm, float *theta_rad);
float Odom_GetForwardAccumMm(void);
float Odom_GetTurnAccumDeg(void);
void Odom_ResetLocalAccum(void);

#endif /* APP_ODOMETRY_H */
