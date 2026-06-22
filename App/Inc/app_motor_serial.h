#ifndef APP_MOTOR_SERIAL_H
#define APP_MOTOR_SERIAL_H

#include <stdint.h>

typedef struct
{
    int32_t encoder_now[4];
    int32_t encoder_delta_10ms[4];
    float speed_mm_s[4];

    uint8_t has_encoder_now;
    uint8_t has_encoder_delta;
    uint8_t has_speed;

    uint32_t last_rx_tick;
    uint32_t frame_count;
} MotorFeedback_t;

void App_MotorSerial_Init(void);
void App_MotorSerial_Task(void);

void MotorSerial_RequestInit(void);
void MotorSerial_SendMotorType(uint8_t type);
void MotorSerial_SendDeadzone(uint16_t deadzone);
void MotorSerial_SendPulseLine(uint16_t line);
void MotorSerial_SendPulsePhase(uint16_t phase);
void MotorSerial_SendWheelDiameter(float diameter_mm);
void MotorSerial_SendPID(float kp, float ki, float kd);
void MotorSerial_SendUploadMode(uint8_t upload_all_encoder,
                                uint8_t upload_10ms_encoder,
                                uint8_t upload_speed);
void MotorSerial_SendSpeed(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
void MotorSerial_SendPWM(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
void MotorSerial_Stop(void);

void MotorSerial_Printf(const char *fmt, ...);
uint8_t MotorSerial_HasRecentFeedback(uint32_t timeout_ms);
const MotorFeedback_t *MotorSerial_GetFeedback(void);
uint8_t MotorSerial_IsInitDone(void);
uint32_t MotorSerial_GetDeltaSequence(void);
void MotorSerial_ResetFeedback(void);

#endif /* APP_MOTOR_SERIAL_H */
