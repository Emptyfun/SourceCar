#include "app_motor_serial.h"

#include "app_config.h"
#include "bsp_uart1.h"
#include "bsp_uart2.h"
#include "ring_buffer.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#if APP_ENABLE_COVERAGE
#include "app_coverage.h"
#endif
#if APP_ENABLE_HMC5883L
#include "app_hmc5883l.h"
#endif
#if APP_ENABLE_MAG_CALIB
#include "app_mag_calib.h"
#endif
#if APP_ENABLE_MOTION
#include "app_motion.h"
#endif
#if APP_ENABLE_MOTION_PRIMITIVE
#include "app_motion_primitive.h"
#endif
#if APP_ENABLE_ODOMETRY
#include "app_odometry.h"
#endif
#if APP_ENABLE_TURN_DEBUG
#include "app_turn_debug.h"
#endif

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MOTOR_UART_BUFFER_SIZE 512U
#define MOTOR_PC_LINE_SIZE 80U
#define MOTOR_FRAME_SIZE 96U
#define MOTOR_INIT_STEP_PERIOD_MS 60U
#define MOTOR_INIT_STOP_SETTLE_MS 100U
#define MOTOR_FEEDBACK_PRINT_PERIOD_MS 250U
#define MOTOR_EMERGENCY_STOP_REPEAT 5U

#ifndef APP_MOTOR_SERIAL_RAW_MIRROR
#define APP_MOTOR_SERIAL_RAW_MIRROR 0
#endif

#ifndef APP_MOTOR_SERIAL_FEEDBACK_PRINT
#define APP_MOTOR_SERIAL_FEEDBACK_PRINT 0
#endif

#ifndef APP_MOTOR_SERIAL_PARSE_SPEED
#define APP_MOTOR_SERIAL_PARSE_SPEED 0
#endif

typedef struct
{
    const char *cmd;
    uint16_t delay_ms;
    uint8_t clear_feedback;
} MotorInitStep_t;

static uint8_t s_pc_rx_storage[MOTOR_UART_BUFFER_SIZE];
static uint8_t s_pc_tx_storage[MOTOR_UART_BUFFER_SIZE];
static uint8_t s_motor_rx_storage[MOTOR_UART_BUFFER_SIZE];
static uint8_t s_motor_tx_storage[MOTOR_UART_BUFFER_SIZE];

static RingBuffer_t s_pc_rx_buffer;
static RingBuffer_t s_pc_tx_buffer;
static RingBuffer_t s_motor_rx_buffer;
static RingBuffer_t s_motor_tx_buffer;

static uint8_t s_pc_rx_byte;
static uint8_t s_motor_rx_byte;
static uint8_t s_pc_tx_byte;
static uint8_t s_motor_tx_byte;
static volatile uint8_t s_pc_tx_busy;
static volatile uint8_t s_motor_tx_busy;

static char s_pc_line[MOTOR_PC_LINE_SIZE];
static uint16_t s_pc_line_len;
static uint8_t s_pc_forward_frame;
static uint8_t s_pc_drop_line;

static char s_motor_frame[MOTOR_FRAME_SIZE];
static uint16_t s_motor_frame_len;
static uint8_t s_motor_frame_active;

static MotorFeedback_t s_feedback;
static uint8_t s_have_last_mall;
static uint8_t s_delta_from_mall;
static int32_t s_last_mall[4];
static uint32_t s_delta_sequence;
static uint8_t s_init_active;
static uint8_t s_init_step;
static uint32_t s_init_next_tick;
static uint32_t s_last_feedback_print_tick;

#if APP_DEBUG_TURN_OPTIMIZE_ONLY
static const MotorInitStep_t s_init_steps[] = {
    {"$spd:0,0,0,0#", MOTOR_INIT_STOP_SETTLE_MS, 0U},
    {"$upload:0,0,0#", MOTOR_INIT_STEP_PERIOD_MS, 1U},
    {"$mtype:1#", MOTOR_INIT_STEP_PERIOD_MS, 0U},
    {"$deadzone:1900#", MOTOR_INIT_STEP_PERIOD_MS, 0U},
    {"$mline:11#", MOTOR_INIT_STEP_PERIOD_MS, 0U},
    {"$mphase:40#", MOTOR_INIT_STEP_PERIOD_MS, 0U},
    {"$wdiameter:65.000#", MOTOR_INIT_STEP_PERIOD_MS, 0U},
    {"$upload:0,1,0#", MOTOR_INIT_STEP_PERIOD_MS, 1U},
    {"$pwm:0,0,0,0#", MOTOR_INIT_STEP_PERIOD_MS, 0U}
};
#else
static const MotorInitStep_t s_init_steps[] = {
    {"$spd:0,0,0,0#", MOTOR_INIT_STOP_SETTLE_MS, 0U},
    {"$upload:0,0,0#", MOTOR_INIT_STEP_PERIOD_MS, 1U},
    {"$mtype:1#", MOTOR_INIT_STEP_PERIOD_MS, 0U},
    {"$deadzone:1900#", MOTOR_INIT_STEP_PERIOD_MS, 0U},
    {"$mline:11#", MOTOR_INIT_STEP_PERIOD_MS, 0U},
    {"$mphase:40#", MOTOR_INIT_STEP_PERIOD_MS, 0U},
    {"$wdiameter:65.000#", MOTOR_INIT_STEP_PERIOD_MS, 0U},
    {"$upload:1,0,0#", MOTOR_INIT_STEP_PERIOD_MS, 1U},
    {"$spd:0,0,0,0#", MOTOR_INIT_STEP_PERIOD_MS, 0U}
};
#endif

static void MotorSerial_StartPcRx(void);
static void MotorSerial_StartMotorRx(void);
static void MotorSerial_StartPcTx(void);
static void MotorSerial_StartMotorTx(void);
static void MotorSerial_StartPendingTx(void);
static void MotorSerial_ProcessPcRx(void);
static void MotorSerial_ProcessMotorRx(void);
static void MotorSerial_ProcessInit(uint32_t now);
static void MotorSerial_HandlePcByte(uint8_t data);
static void MotorSerial_HandlePcCommand(void);
static void MotorSerial_HandleMotorByte(uint8_t data);
static void MotorSerial_ParseMotorFrame(const char *frame);
static void MotorSerial_PrintFeedback(uint32_t now);
static uint8_t MotorSerial_Parse4Int(const char *text, int32_t out[4]);
#if APP_MOTOR_SERIAL_PARSE_SPEED
static uint8_t MotorSerial_Parse4Fixed3(const char *text, int32_t out[4]);
#endif
static const char *MotorSerial_FindPayload(const char *frame, const char *tag);
static void MotorSerial_ClearFeedback(void);
static void MotorSerial_QueuePcByte(uint8_t data);
static void MotorSerial_QueueMotorByte(uint8_t data);
static void MotorSerial_QueuePcString(const char *s);
static void MotorSerial_QueueMotorString(const char *s);
static void MotorSerial_ClearMotorTxQueue(void);
static void MotorSerial_QueueCommand4(const char *name, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
static void MotorSerial_QueueFixed3Command(const char *name, float value);
static void MotorSerial_QueueFixed3TripleCommand(const char *name, float a, float b, float c);
static long MotorSerial_FloatToFixed3(float value);
static void MotorSerial_PrintHelp(void);
static void MotorSerial_HandleStopCommand(void);
static void MotorSerial_HandleManualMove(const char *cmd);
static void MotorSerial_HandleTurnCommand(const char *cmd);
static void MotorSerial_HandleSpinCommand(const char *cmd);
static void MotorSerial_HandleHmcCommand(const char *cmd);
static void MotorSerial_HandleTurnDebugCommand(const char *cmd);
static void MotorSerial_HandleMagCalibCommand(const char *cmd);
static int32_t MotorSerial_ParseCommandInt(const char *text, int32_t default_value);
static uint8_t MotorSerial_StartsWith(const char *text, const char *prefix);
static uint8_t MotorSerial_ToLower(uint8_t ch);

void App_MotorSerial_Init(void)
{
    RingBuffer_Init(&s_pc_rx_buffer, s_pc_rx_storage, MOTOR_UART_BUFFER_SIZE);
    RingBuffer_Init(&s_pc_tx_buffer, s_pc_tx_storage, MOTOR_UART_BUFFER_SIZE);
    RingBuffer_Init(&s_motor_rx_buffer, s_motor_rx_storage, MOTOR_UART_BUFFER_SIZE);
    RingBuffer_Init(&s_motor_tx_buffer, s_motor_tx_storage, MOTOR_UART_BUFFER_SIZE);

    s_pc_tx_busy = 0U;
    s_motor_tx_busy = 0U;
    s_pc_line_len = 0U;
    s_pc_forward_frame = 0U;
    s_pc_drop_line = 0U;
    s_motor_frame_len = 0U;
    s_motor_frame_active = 0U;
    s_last_feedback_print_tick = HAL_GetTick();
    MotorSerial_ClearFeedback();

    MotorSerial_StartPcRx();
    MotorSerial_StartMotorRx();
#if APP_DEBUG_TURN_OPTIMIZE_ONLY
    MotorSerial_QueuePcString("\r\n[BOOT] turn optimize firmware\r\n");
    MotorSerial_QueuePcString("[TURNDBG] serial: turn help | turn status | turn hmc R 90 | turn fixed R 80 3000\r\n");
    MotorSerial_QueuePcString("[STOP] stop uses final pwm zero output\r\n");
#elif APP_ENABLE_MAG_CALIB
    MotorSerial_QueuePcString("\r\n[BOOT] motor magnetic calibration firmware\r\n");
    MotorSerial_QueuePcString("[KEY] K1 direction, K2 start/stop mag calib, long press emergency stop\r\n");
    MotorSerial_QueuePcString("[MCAL] serial: mcal start|stop|left|right|dir|speed N|status\r\n");
#elif APP_ENABLE_COVERAGE
    MotorSerial_QueuePcString("\r\n[BOOT] motor coverage firmware\r\n");
    MotorSerial_QueuePcString("[KEY] K2 start/stop coverage, long press emergency stop\r\n");
    MotorSerial_QueuePcString("[COV] 1m x 1m lawnmower path, serial input optional\r\n");
#else
    MotorSerial_QueuePcString("\r\n[BOOT] motor debug firmware\r\n");
    MotorSerial_QueuePcString("[KEY] long press emergency stop\r\n");
#endif
    MotorSerial_RequestInit();
}

void App_MotorSerial_Task(void)
{
    uint32_t now = HAL_GetTick();

    MotorSerial_ProcessPcRx();
    MotorSerial_ProcessMotorRx();
    MotorSerial_ProcessInit(now);
#if APP_MOTOR_SERIAL_FEEDBACK_PRINT
    MotorSerial_PrintFeedback(now);
#else
    (void)MotorSerial_PrintFeedback;
#endif
    MotorSerial_StartPendingTx();
}

void MotorSerial_RequestInit(void)
{
    s_init_active = 1U;
    s_init_step = 0U;
    s_init_next_tick = HAL_GetTick();
#if APP_DEBUG_TURN_OPTIMIZE_ONLY
    MotorSerial_QueuePcString("[TURNDBG] motor init start\r\n");
#else
    MotorSerial_QueuePcString("[MOTOR] init start\r\n");
#endif
}

void MotorSerial_SendMotorType(uint8_t type)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "$mtype:%u#", (unsigned int)type);
    MotorSerial_QueueMotorString(buf);
}

void MotorSerial_SendDeadzone(uint16_t deadzone)
{
    char buf[28];
    snprintf(buf, sizeof(buf), "$deadzone:%u#", (unsigned int)deadzone);
    MotorSerial_QueueMotorString(buf);
}

void MotorSerial_SendPulseLine(uint16_t line)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "$mline:%u#", (unsigned int)line);
    MotorSerial_QueueMotorString(buf);
}

void MotorSerial_SendPulsePhase(uint16_t phase)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "$mphase:%u#", (unsigned int)phase);
    MotorSerial_QueueMotorString(buf);
}

void MotorSerial_SendWheelDiameter(float diameter_mm)
{
    MotorSerial_QueueFixed3Command("wdiameter", diameter_mm);
}

void MotorSerial_SendPID(float kp, float ki, float kd)
{
    MotorSerial_QueueFixed3TripleCommand("mpid", kp, ki, kd);
}

void MotorSerial_SendUploadMode(uint8_t upload_all_encoder,
                                uint8_t upload_10ms_encoder,
                                uint8_t upload_speed)
{
    char buf[28];
    snprintf(buf,
             sizeof(buf),
             "$upload:%u,%u,%u#",
             (unsigned int)(upload_all_encoder != 0U),
             (unsigned int)(upload_10ms_encoder != 0U),
             (unsigned int)(upload_speed != 0U));
    MotorSerial_QueueMotorString(buf);
}

void MotorSerial_SendSpeed(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    MotorSerial_QueueCommand4("spd", m1, m2, m3, m4);
}

void MotorSerial_SendPWM(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    MotorSerial_QueueCommand4("pwm", m1, m2, m3, m4);
}

void MotorSerial_Stop(void)
{
    MotorSerial_SendSpeed(0, 0, 0, 0);
    MotorSerial_SendPWM(0, 0, 0, 0);
}

void MotorSerial_StopOutput(void)
{
    MotorSerial_SendPWM(0, 0, 0, 0);
}

void MotorSerial_EmergencyStop(void)
{
    uint8_t i;

    MotorSerial_ClearMotorTxQueue();
    for (i = 0U; i < MOTOR_EMERGENCY_STOP_REPEAT; i++)
    {
        MotorSerial_SendSpeed(0, 0, 0, 0);
        MotorSerial_SendPWM(0, 0, 0, 0);
    }
}

void MotorSerial_Printf(const char *fmt, ...)
{
    char buf[180];
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

    MotorSerial_QueuePcString(buf);
}

uint8_t MotorSerial_HasRecentFeedback(uint32_t timeout_ms)
{
    if (s_feedback.last_rx_tick == 0UL)
    {
        return 0U;
    }

    return ((uint32_t)(HAL_GetTick() - s_feedback.last_rx_tick) <= timeout_ms) ? 1U : 0U;
}

const MotorFeedback_t *MotorSerial_GetFeedback(void)
{
    return &s_feedback;
}

uint8_t MotorSerial_IsInitDone(void)
{
    return (s_init_active == 0U) ? 1U : 0U;
}

uint32_t MotorSerial_GetDeltaSequence(void)
{
    return s_delta_sequence;
}

void MotorSerial_ResetFeedback(void)
{
    MotorSerial_ClearFeedback();
}

#if APP_ENABLE_MOTOR_SERIAL
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        (void)RingBuffer_Push(&s_pc_rx_buffer, s_pc_rx_byte);
        MotorSerial_StartPcRx();
    }
    else if (huart->Instance == USART2)
    {
        (void)RingBuffer_Push(&s_motor_rx_buffer, s_motor_rx_byte);
        MotorSerial_StartMotorRx();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        s_pc_tx_busy = 0U;
        MotorSerial_StartPcTx();
    }
    else if (huart->Instance == USART2)
    {
        s_motor_tx_busy = 0U;
        MotorSerial_StartMotorTx();
    }
}
#endif

static void MotorSerial_StartPcRx(void)
{
    (void)HAL_UART_Receive_IT(BSP_UART1_GetHandle(), &s_pc_rx_byte, 1U);
}

static void MotorSerial_StartMotorRx(void)
{
    (void)HAL_UART_Receive_IT(BSP_UART2_GetHandle(), &s_motor_rx_byte, 1U);
}

static void MotorSerial_StartPcTx(void)
{
    if (s_pc_tx_busy != 0U)
    {
        return;
    }

    if (RingBuffer_Pop(&s_pc_tx_buffer, &s_pc_tx_byte) != 0U)
    {
        s_pc_tx_busy = 1U;
        if (HAL_UART_Transmit_IT(BSP_UART1_GetHandle(), &s_pc_tx_byte, 1U) != HAL_OK)
        {
            s_pc_tx_busy = 0U;
        }
    }
}

static void MotorSerial_StartMotorTx(void)
{
    if (s_motor_tx_busy != 0U)
    {
        return;
    }

    if (RingBuffer_Pop(&s_motor_tx_buffer, &s_motor_tx_byte) != 0U)
    {
        s_motor_tx_busy = 1U;
        if (HAL_UART_Transmit_IT(BSP_UART2_GetHandle(), &s_motor_tx_byte, 1U) != HAL_OK)
        {
            s_motor_tx_busy = 0U;
        }
    }
}

static void MotorSerial_StartPendingTx(void)
{
    MotorSerial_StartPcTx();
    MotorSerial_StartMotorTx();
}

static void MotorSerial_ProcessPcRx(void)
{
    uint8_t data;

    while (RingBuffer_Pop(&s_pc_rx_buffer, &data) != 0U)
    {
        MotorSerial_HandlePcByte(data);
    }
}

static void MotorSerial_ProcessMotorRx(void)
{
    uint8_t data;

    while (RingBuffer_Pop(&s_motor_rx_buffer, &data) != 0U)
    {
        MotorSerial_HandleMotorByte(data);
    }
}

static void MotorSerial_ProcessInit(uint32_t now)
{
    if (s_init_active == 0U)
    {
        return;
    }

    if ((uint32_t)(now - s_init_next_tick) >= 0x80000000UL)
    {
        return;
    }

    if (s_init_step >= (uint8_t)(sizeof(s_init_steps) / sizeof(s_init_steps[0])))
    {
        s_init_active = 0U;
#if APP_DEBUG_TURN_OPTIMIZE_ONLY
        MotorSerial_QueuePcString("[TURNDBG] motor init done\r\n");
#else
        MotorSerial_QueuePcString("[MOTOR] init done\r\n");
#endif
        return;
    }

    if (s_init_steps[s_init_step].clear_feedback != 0U)
    {
        MotorSerial_ClearFeedback();
    }

    MotorSerial_QueueMotorString(s_init_steps[s_init_step].cmd);
    MotorSerial_QueuePcString("[MOTOR] -> ");
    MotorSerial_QueuePcString(s_init_steps[s_init_step].cmd);
    MotorSerial_QueuePcString("\r\n");
    s_init_next_tick = now + (uint32_t)s_init_steps[s_init_step].delay_ms;
    s_init_step++;
}

static void MotorSerial_HandlePcByte(uint8_t data)
{
    if (s_pc_forward_frame != 0U)
    {
        MotorSerial_QueueMotorByte(data);
        if (data == '#')
        {
            s_pc_forward_frame = 0U;
        }
        return;
    }

    if (data == '$')
    {
        MotorSerial_QueueMotorByte(data);
        s_pc_forward_frame = 1U;
        s_pc_line_len = 0U;
        s_pc_drop_line = 0U;
        return;
    }

    if ((data == '\r') || (data == '\n'))
    {
        if ((s_pc_line_len != 0U) && (s_pc_drop_line == 0U))
        {
            s_pc_line[s_pc_line_len] = '\0';
            MotorSerial_HandlePcCommand();
        }
        s_pc_line_len = 0U;
        s_pc_drop_line = 0U;
        return;
    }

    if (s_pc_drop_line != 0U)
    {
        return;
    }

    if (s_pc_line_len < (uint16_t)(sizeof(s_pc_line) - 1U))
    {
        s_pc_line[s_pc_line_len] = (char)MotorSerial_ToLower(data);
        s_pc_line_len++;
    }
    else
    {
        s_pc_drop_line = 1U;
        MotorSerial_QueuePcString("[CMD] too long\r\n");
    }
}

static void MotorSerial_HandlePcCommand(void)
{
    if ((strcmp(s_pc_line, "help") == 0) || (strcmp(s_pc_line, "?") == 0))
    {
        MotorSerial_PrintHelp();
        return;
    }

    if ((strcmp(s_pc_line, "motor init") == 0) || (strcmp(s_pc_line, "init") == 0))
    {
        MotorSerial_RequestInit();
        return;
    }

    if ((strcmp(s_pc_line, "stop") == 0) || (strcmp(s_pc_line, "motor stop") == 0))
    {
        MotorSerial_HandleStopCommand();
        return;
    }

    if (strcmp(s_pc_line, "read_flash") == 0)
    {
        MotorSerial_QueueMotorString("$read_flash#");
        return;
    }

    if (strcmp(s_pc_line, "read_vol") == 0)
    {
        MotorSerial_QueueMotorString("$read_vol#");
        return;
    }

    if (strcmp(s_pc_line, "upload off") == 0)
    {
        MotorSerial_ClearFeedback();
        MotorSerial_SendUploadMode(0U, 0U, 0U);
        return;
    }

    if (strcmp(s_pc_line, "upload all") == 0)
    {
        MotorSerial_ClearFeedback();
        MotorSerial_SendUploadMode(1U, 0U, 0U);
        return;
    }

    if (strcmp(s_pc_line, "upload mtep") == 0)
    {
        MotorSerial_ClearFeedback();
        MotorSerial_SendUploadMode(0U, 1U, 0U);
        return;
    }

    if (MotorSerial_StartsWith(s_pc_line, "deadzone "))
    {
        MotorSerial_SendDeadzone((uint16_t)MotorSerial_ParseCommandInt(&s_pc_line[9], 1900L));
        return;
    }

    if (MotorSerial_StartsWith(s_pc_line, "fwd ") ||
        MotorSerial_StartsWith(s_pc_line, "back ") ||
        MotorSerial_StartsWith(s_pc_line, "left ") ||
        MotorSerial_StartsWith(s_pc_line, "right "))
    {
        MotorSerial_HandleManualMove(s_pc_line);
        return;
    }

    if (MotorSerial_StartsWith(s_pc_line, "move "))
    {
#if APP_ENABLE_MOTION_PRIMITIVE
        MotionPrimitive_MoveDistance_Start((float)MotorSerial_ParseCommandInt(&s_pc_line[5], 300L), 120);
#else
        MotorSerial_QueuePcString("[CMD] motion primitive disabled\r\n");
#endif
        return;
    }

#if APP_ENABLE_TURN_DEBUG
    if ((strcmp(s_pc_line, "turn") == 0) || MotorSerial_StartsWith(s_pc_line, "turn "))
    {
        App_TurnDebug_HandleCommand(s_pc_line);
        return;
    }
#endif

    if (MotorSerial_StartsWith(s_pc_line, "turn "))
    {
        MotorSerial_HandleTurnCommand(s_pc_line);
        return;
    }

    if (MotorSerial_StartsWith(s_pc_line, "spin "))
    {
        MotorSerial_HandleSpinCommand(s_pc_line);
        return;
    }

    if ((strcmp(s_pc_line, "hmc") == 0) ||
        (strcmp(s_pc_line, "mag") == 0) ||
        MotorSerial_StartsWith(s_pc_line, "hmc "))
    {
        MotorSerial_HandleHmcCommand(s_pc_line);
        return;
    }

    if (MotorSerial_StartsWith(s_pc_line, "tdbg "))
    {
        MotorSerial_HandleTurnDebugCommand(s_pc_line);
        return;
    }

    if ((strcmp(s_pc_line, "mcal") == 0) || MotorSerial_StartsWith(s_pc_line, "mcal "))
    {
        MotorSerial_HandleMagCalibCommand(s_pc_line);
        return;
    }

    if (strcmp(s_pc_line, "cov start") == 0)
    {
#if APP_ENABLE_COVERAGE
        Coverage_Start();
#else
        MotorSerial_QueuePcString("[CMD] coverage disabled\r\n");
#endif
        return;
    }

    if (strcmp(s_pc_line, "cov stop") == 0)
    {
#if APP_ENABLE_COVERAGE
        Coverage_Stop();
#else
        MotorSerial_Stop();
#endif
        return;
    }

    if (strcmp(s_pc_line, "status") == 0)
    {
        MotorSerial_Printf("[MOTOR] frames=%lu last=%lums enc=%ld,%ld,%ld,%ld step=%ld,%ld,%ld,%ld\r\n",
                           (unsigned long)s_feedback.frame_count,
                           (unsigned long)s_feedback.last_rx_tick,
                           (long)s_feedback.encoder_now[0],
                           (long)s_feedback.encoder_now[1],
                           (long)s_feedback.encoder_now[2],
                           (long)s_feedback.encoder_now[3],
                           (long)s_feedback.encoder_delta_10ms[0],
                           (long)s_feedback.encoder_delta_10ms[1],
                           (long)s_feedback.encoder_delta_10ms[2],
                           (long)s_feedback.encoder_delta_10ms[3]);
        return;
    }

    MotorSerial_QueuePcString("[CMD] unknown, type help\r\n");
}

static void MotorSerial_HandleMotorByte(uint8_t data)
{
#if APP_MOTOR_SERIAL_RAW_MIRROR
    MotorSerial_QueuePcByte(data);
#else
    (void)MotorSerial_QueuePcByte;
#endif

    if (data == '$')
    {
        s_motor_frame_active = 1U;
        s_motor_frame_len = 0U;
        return;
    }

    if (s_motor_frame_active == 0U)
    {
        return;
    }

    if (data == '#')
    {
        s_motor_frame[s_motor_frame_len] = '\0';
        s_motor_frame_active = 0U;
        MotorSerial_ParseMotorFrame(s_motor_frame);
        return;
    }

    if (s_motor_frame_len < (uint16_t)(sizeof(s_motor_frame) - 1U))
    {
        s_motor_frame[s_motor_frame_len] = (char)data;
        s_motor_frame_len++;
    }
    else
    {
        s_motor_frame_active = 0U;
        s_motor_frame_len = 0U;
    }
}

static void MotorSerial_ParseMotorFrame(const char *frame)
{
    const char *payload;
    int32_t values_i[4];
    uint8_t i;
#if APP_MOTOR_SERIAL_PARSE_SPEED
    int32_t values_fixed[4];
#endif

    payload = MotorSerial_FindPayload(frame, "MAll");
    if ((payload != NULL) && (MotorSerial_Parse4Int(payload, values_i) != 0U))
    {
        for (i = 0U; i < 4U; i++)
        {
            s_feedback.encoder_now[i] = values_i[i];
            if (s_have_last_mall != 0U)
            {
                s_feedback.encoder_delta_10ms[i] = values_i[i] - s_last_mall[i];
            }
            s_last_mall[i] = values_i[i];
        }

        s_feedback.has_encoder_now = 1U;
        s_feedback.frame_count++;
        s_feedback.last_rx_tick = HAL_GetTick();
        if (s_have_last_mall != 0U)
        {
            s_feedback.has_encoder_delta = 1U;
            s_delta_from_mall = 1U;
            s_delta_sequence++;
#if APP_ENABLE_ODOMETRY
            Odom_UpdateFromEncoderDelta(s_feedback.encoder_delta_10ms);
#endif
        }
        s_have_last_mall = 1U;
        return;
    }

    payload = MotorSerial_FindPayload(frame, "MTEP");
    if ((payload != NULL) && (MotorSerial_Parse4Int(payload, values_i) != 0U))
    {
        for (i = 0U; i < 4U; i++)
        {
            s_feedback.encoder_delta_10ms[i] = values_i[i];
        }
        s_feedback.has_encoder_delta = 1U;
        s_delta_from_mall = 0U;
        s_delta_sequence++;
        s_feedback.frame_count++;
        s_feedback.last_rx_tick = HAL_GetTick();
#if APP_ENABLE_ODOMETRY
        Odom_UpdateFromEncoderDelta(s_feedback.encoder_delta_10ms);
#endif
        return;
    }

#if APP_MOTOR_SERIAL_PARSE_SPEED
    payload = MotorSerial_FindPayload(frame, "MSPD");
    if ((payload != NULL) && (MotorSerial_Parse4Fixed3(payload, values_fixed) != 0U))
    {
        for (i = 0U; i < 4U; i++)
        {
            s_feedback.speed_mm_s[i] = (float)values_fixed[i] / 1000.0f;
        }
        s_feedback.has_speed = 1U;
        s_feedback.frame_count++;
        s_feedback.last_rx_tick = HAL_GetTick();
    }
#endif
}

static void MotorSerial_PrintFeedback(uint32_t now)
{
    if ((uint32_t)(now - s_last_feedback_print_tick) < MOTOR_FEEDBACK_PRINT_PERIOD_MS)
    {
        return;
    }
    s_last_feedback_print_tick = now;

    if (s_feedback.has_encoder_delta == 0U)
    {
        return;
    }

    MotorSerial_Printf("[%s] %ld,%ld,%ld,%ld\r\n",
                       (s_delta_from_mall != 0U) ? "MAllD" : "MTEP",
                       (long)s_feedback.encoder_delta_10ms[0],
                       (long)s_feedback.encoder_delta_10ms[1],
                       (long)s_feedback.encoder_delta_10ms[2],
                       (long)s_feedback.encoder_delta_10ms[3]);
}

static uint8_t MotorSerial_Parse4Int(const char *text, int32_t out[4])
{
    char *end;
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        while ((*text == ' ') || (*text == ','))
        {
            text++;
        }

        out[i] = (int32_t)strtol(text, &end, 10);
        if (end == text)
        {
            return 0U;
        }
        text = end;
    }

    return 1U;
}

#if APP_MOTOR_SERIAL_PARSE_SPEED
static uint8_t MotorSerial_Parse4Fixed3(const char *text, int32_t out[4])
{
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        int32_t sign = 1L;
        int32_t whole = 0L;
        int32_t frac = 0L;
        uint8_t frac_digits = 0U;
        uint8_t has_digit = 0U;

        while ((*text == ' ') || (*text == ','))
        {
            text++;
        }

        if (*text == '-')
        {
            sign = -1L;
            text++;
        }
        else if (*text == '+')
        {
            text++;
        }

        while ((*text >= '0') && (*text <= '9'))
        {
            whole = (whole * 10L) + (int32_t)(*text - '0');
            has_digit = 1U;
            text++;
        }

        if (*text == '.')
        {
            text++;
            while ((*text >= '0') && (*text <= '9'))
            {
                if (frac_digits < 3U)
                {
                    frac = (frac * 10L) + (int32_t)(*text - '0');
                    frac_digits++;
                }
                has_digit = 1U;
                text++;
            }
        }

        if (has_digit == 0U)
        {
            return 0U;
        }

        while (frac_digits < 3U)
        {
            frac *= 10L;
            frac_digits++;
        }

        out[i] = sign * ((whole * 1000L) + frac);
    }

    return 1U;
}
#endif

static const char *MotorSerial_FindPayload(const char *frame, const char *tag)
{
    size_t tag_len = strlen(tag);

    if ((strncmp(frame, tag, tag_len) == 0) && (frame[tag_len] == ':'))
    {
        return &frame[tag_len + 1U];
    }

    return NULL;
}

static void MotorSerial_ClearFeedback(void)
{
    memset(&s_feedback, 0, sizeof(s_feedback));
    memset(s_last_mall, 0, sizeof(s_last_mall));
    s_have_last_mall = 0U;
    s_delta_from_mall = 0U;
    s_delta_sequence = 0UL;
}

static void MotorSerial_QueuePcByte(uint8_t data)
{
    (void)RingBuffer_Push(&s_pc_tx_buffer, data);
    MotorSerial_StartPcTx();
}

static void MotorSerial_QueueMotorByte(uint8_t data)
{
    (void)RingBuffer_Push(&s_motor_tx_buffer, data);
    MotorSerial_StartMotorTx();
}

static void MotorSerial_QueuePcString(const char *s)
{
    if (s == NULL)
    {
        return;
    }

    while (*s != '\0')
    {
        if (RingBuffer_Push(&s_pc_tx_buffer, (uint8_t)*s) == 0U)
        {
            break;
        }
        s++;
    }

    MotorSerial_StartPcTx();
}

static void MotorSerial_QueueMotorString(const char *s)
{
    if (s == NULL)
    {
        return;
    }

    while (*s != '\0')
    {
        if (RingBuffer_Push(&s_motor_tx_buffer, (uint8_t)*s) == 0U)
        {
            break;
        }
        s++;
    }

    MotorSerial_StartMotorTx();
}

static void MotorSerial_ClearMotorTxQueue(void)
{
    __disable_irq();
    s_motor_tx_buffer.head = 0U;
    s_motor_tx_buffer.tail = 0U;
    __enable_irq();
}

static void MotorSerial_QueueCommand4(const char *name, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "$%s:%d,%d,%d,%d#", name, (int)v1, (int)v2, (int)v3, (int)v4);
    MotorSerial_QueueMotorString(buf);
}

static void MotorSerial_QueueFixed3Command(const char *name, float value)
{
    char buf[36];
    long fixed = MotorSerial_FloatToFixed3(value);
    long abs_fixed = (fixed < 0L) ? -fixed : fixed;

    snprintf(buf,
             sizeof(buf),
             "$%s:%s%ld.%03ld#",
             name,
             (fixed < 0L) ? "-" : "",
             abs_fixed / 1000L,
             abs_fixed % 1000L);
    MotorSerial_QueueMotorString(buf);
}

static void MotorSerial_QueueFixed3TripleCommand(const char *name, float a, float b, float c)
{
    char buf[64];
    long fa = MotorSerial_FloatToFixed3(a);
    long fb = MotorSerial_FloatToFixed3(b);
    long fc = MotorSerial_FloatToFixed3(c);
    long aa = (fa < 0L) ? -fa : fa;
    long ab = (fb < 0L) ? -fb : fb;
    long ac = (fc < 0L) ? -fc : fc;

    snprintf(buf,
             sizeof(buf),
             "$%s:%s%ld.%03ld,%s%ld.%03ld,%s%ld.%03ld#",
             name,
             (fa < 0L) ? "-" : "",
             aa / 1000L,
             aa % 1000L,
             (fb < 0L) ? "-" : "",
             ab / 1000L,
             ab % 1000L,
             (fc < 0L) ? "-" : "",
             ac / 1000L,
             ac % 1000L);
    MotorSerial_QueueMotorString(buf);
}

static long MotorSerial_FloatToFixed3(float value)
{
    if (value >= 0.0f)
    {
        return (long)((value * 1000.0f) + 0.5f);
    }

    return (long)((value * 1000.0f) - 0.5f);
}

static void MotorSerial_PrintHelp(void)
{
    MotorSerial_QueuePcString("[CMD] direct motor board command: $...#\r\n");
#if APP_ENABLE_TURN_DEBUG
    MotorSerial_QueuePcString("[CMD] turn debug: turn help | turn status | turn fixed R 80 3000 | turn hmc R 90 | turn stop\r\n");
#endif
    MotorSerial_QueuePcString("[CMD] local: motor init | stop | fwd N | back N | left N | right N | spin WZ\r\n");
    MotorSerial_QueuePcString("[CMD] local: move MM | turn REL_DEG [SPD] | hmc | tdbg on/off | cov start | cov stop | status\r\n");
    MotorSerial_QueuePcString("[CMD] hmc: hmc cal start|stop|reset|print | hmc ref | hmc help\r\n");
#if APP_ENABLE_MAG_CALIB
    MotorSerial_QueuePcString("[CMD] magcal: mcal start|stop|left|right|dir|speed N|status\r\n");
#endif
}

static void MotorSerial_HandleStopCommand(void)
{
#if APP_DEBUG_TURN_OPTIMIZE_ONLY
#if APP_ENABLE_TURN_DEBUG
    App_TurnDebug_Stop();
#else
    MotorSerial_StopOutput();
#endif
    MotorSerial_QueuePcString("[STOP] pwm zero\r\n");
    return;
#else
#if APP_ENABLE_TURN_DEBUG
    App_TurnDebug_Stop();
#endif
#if APP_ENABLE_COVERAGE
    Coverage_Stop();
#endif
#if APP_ENABLE_MAG_CALIB
    App_MagCalib_Stop();
#endif
#if APP_ENABLE_MOTION_PRIMITIVE
    MotionPrimitive_Abort();
#endif
#if APP_ENABLE_MOTION
    Motion_Stop();
#else
    MotorSerial_Stop();
#endif
    MotorSerial_EmergencyStop();
    MotorSerial_QueuePcString("[STOP] emergency\r\n");
#endif
}

static void MotorSerial_HandleManualMove(const char *cmd)
{
#if APP_ENABLE_MOTION
    int32_t speed;

    if (MotorSerial_StartsWith(cmd, "fwd "))
    {
        speed = MotorSerial_ParseCommandInt(&cmd[4], 100L);
        Motion_SetVxWz((int16_t)speed, 0);
    }
    else if (MotorSerial_StartsWith(cmd, "back "))
    {
        speed = MotorSerial_ParseCommandInt(&cmd[5], 100L);
        Motion_SetVxWz((int16_t)-speed, 0);
    }
    else if (MotorSerial_StartsWith(cmd, "left "))
    {
        speed = MotorSerial_ParseCommandInt(&cmd[5], 100L);
        Motion_SetVxWz(0, (int16_t)-speed);
    }
    else
    {
        speed = MotorSerial_ParseCommandInt(&cmd[6], 100L);
        Motion_SetVxWz(0, (int16_t)speed);
    }
#else
    (void)cmd;
    MotorSerial_QueuePcString("[CMD] motion disabled\r\n");
#endif
}

static void MotorSerial_HandleTurnCommand(const char *cmd)
{
#if APP_ENABLE_MOTION_PRIMITIVE
    const char *args = &cmd[5];
    char *end;
    int32_t angle;
    int32_t speed = 80L;

    while (*args == ' ')
    {
        args++;
    }

    angle = (int32_t)strtol(args, &end, 10);
    if (end == args)
    {
        angle = 90L;
    }
    else
    {
        while (*end == ' ')
        {
            end++;
        }

        if (*end != '\0')
        {
            speed = (int32_t)strtol(end, NULL, 10);
        }
    }

    if (speed < 1L)
    {
        speed = 1L;
    }
    if (speed > 200L)
    {
        speed = 200L;
    }

    MotorSerial_Printf("[CMD] turn relative angle=%ld speed=%ld\r\n", (long)angle, (long)speed);
    MotionPrimitive_TurnAngle_Start((float)angle, (int16_t)speed);
#else
    (void)cmd;
    MotorSerial_QueuePcString("[CMD] motion primitive disabled\r\n");
#endif
}

static void MotorSerial_HandleSpinCommand(const char *cmd)
{
#if APP_ENABLE_MOTION
    int32_t wz = MotorSerial_ParseCommandInt(&cmd[5], 40L);

    if (wz > 200L)
    {
        wz = 200L;
    }
    if (wz < -200L)
    {
        wz = -200L;
    }

#if APP_ENABLE_MOTION_PRIMITIVE
    MotionPrimitive_Abort();
#endif
    Motion_SetVxWz(0, (int16_t)wz);
    MotorSerial_Printf("[CMD] spin wz=%ld wheel=%ld,%ld,%ld,%ld\r\n",
                       (long)wz,
                       (long)wz,
                       (long)wz,
                       (long)-wz,
                       (long)-wz);
#else
    (void)cmd;
    MotorSerial_QueuePcString("[CMD] motion disabled\r\n");
#endif
}

static void MotorSerial_HandleHmcCommand(const char *cmd)
{
#if APP_ENABLE_HMC5883L
    const HMC5883L_Info_t *info = App_HMC5883L_GetInfo();
    const char *arg = cmd;
    long heading_deci = (long)((info->heading_deg >= 0.0f) ?
                               ((info->heading_deg * 10.0f) + 0.5f) :
                               ((info->heading_deg * 10.0f) - 0.5f));
    long raw_heading_deci = (long)((info->heading_raw_deg >= 0.0f) ?
                                   ((info->heading_raw_deg * 10.0f) + 0.5f) :
                                   ((info->heading_raw_deg * 10.0f) - 0.5f));
    long delta_deci = (long)((info->delta_deg >= 0.0f) ?
                             ((info->delta_deg * 10.0f) + 0.5f) :
                             ((info->delta_deg * 10.0f) - 0.5f));
    long raw_heading_abs = (raw_heading_deci < 0L) ? -raw_heading_deci : raw_heading_deci;
    long delta_abs = (delta_deci < 0L) ? -delta_deci : delta_deci;

    while ((*arg != '\0') && (*arg != ' '))
    {
        arg++;
    }
    while (*arg == ' ')
    {
        arg++;
    }

    if (strcmp(arg, "cal start") == 0)
    {
        MotorSerial_QueuePcString("[HMC-CMD] hmc cal start\r\n");
        App_HMC5883L_CalStart();
        return;
    }

    if (strcmp(arg, "cal stop") == 0)
    {
        MotorSerial_QueuePcString("[HMC-CMD] hmc cal stop\r\n");
        App_HMC5883L_CalStopAndApply();
        return;
    }

    if (strcmp(arg, "cal reset") == 0)
    {
        MotorSerial_QueuePcString("[HMC-CMD] hmc cal reset\r\n");
        App_HMC5883L_CalReset();
        return;
    }

    if (strcmp(arg, "cal print") == 0)
    {
        App_HMC5883L_CalPrint();
        return;
    }

    if (strcmp(arg, "ref") == 0)
    {
        App_HMC5883L_ResetDeltaReference();
        MotorSerial_QueuePcString("[HMC-CMD] hmc ref delta=0\r\n");
        return;
    }

    if (strcmp(arg, "help") == 0)
    {
        MotorSerial_QueuePcString("[HMC-CMD] hmc | hmc ref | hmc cal start|stop|reset|print\r\n");
        return;
    }

    if (*arg != '\0')
    {
        MotorSerial_QueuePcString("[HMC-CMD] unknown, use hmc help\r\n");
        return;
    }

    MotorSerial_Printf("[HMC] ready=%u init=%u read=%u drdy=%u st=0x%02X x=%d y=%d z=%d raw=%s%ld.%01ld cal=%ld.%01ld delta=%s%ld.%01ld cnt=%lu age=%lums\r\n",
                       (unsigned int)App_HMC5883L_IsReady(),
                       (unsigned int)info->init_ok,
                       (unsigned int)info->read_ok,
                       (unsigned int)info->drdy_level,
                       (unsigned int)info->status_reg,
                       (int)info->x,
                       (int)info->y,
                       (int)info->z,
                       (raw_heading_deci < 0L) ? "-" : "",
                       raw_heading_abs / 10L,
                       raw_heading_abs % 10L,
                       heading_deci / 10L,
                       heading_deci % 10L,
                       (delta_deci < 0L) ? "-" : "",
                       delta_abs / 10L,
                       delta_abs % 10L,
                       (unsigned long)info->sample_count,
                       (unsigned long)(HAL_GetTick() - info->last_update_tick));
    MotorSerial_Printf("[HMC] axis a=%ld b=%ld ac=%ld bc=%ld cal_valid=%u cal_state=%u\r\n",
                       (long)info->axis_a_raw,
                       (long)info->axis_b_raw,
                       (long)info->axis_a_cal,
                       (long)info->axis_b_cal,
                       (unsigned int)info->cal.valid,
                       (unsigned int)info->cal_state);
#else
    (void)cmd;
    MotorSerial_QueuePcString("[CMD] HMC disabled\r\n");
#endif
}

static void MotorSerial_HandleTurnDebugCommand(const char *cmd)
{
#if APP_ENABLE_MOTION_PRIMITIVE
    if ((strcmp(&cmd[5], "on") == 0) || (strcmp(&cmd[5], "1") == 0))
    {
        MotionPrimitive_SetTurnDebug(1U);
        MotorSerial_QueuePcString("[TDBG] on\r\n");
    }
    else
    {
        MotionPrimitive_SetTurnDebug(0U);
        MotorSerial_QueuePcString("[TDBG] off\r\n");
    }
#else
    (void)cmd;
    MotorSerial_QueuePcString("[CMD] motion primitive disabled\r\n");
#endif
}

static void MotorSerial_HandleMagCalibCommand(const char *cmd)
{
#if APP_ENABLE_MAG_CALIB
    const char *arg = cmd;

    while ((*arg != '\0') && (*arg != ' '))
    {
        arg++;
    }
    while (*arg == ' ')
    {
        arg++;
    }

    if ((*arg == '\0') || (strcmp(arg, "status") == 0))
    {
        App_MagCalib_PrintStatus();
    }
    else if ((strcmp(arg, "start") == 0) || (strcmp(arg, "run") == 0))
    {
        App_MagCalib_Start();
    }
    else if ((strcmp(arg, "stop") == 0) || (strcmp(arg, "end") == 0))
    {
        App_MagCalib_Stop();
    }
    else if ((strcmp(arg, "toggle") == 0) || (strcmp(arg, "ok") == 0))
    {
        App_MagCalib_Toggle();
    }
    else if ((strcmp(arg, "dir") == 0) || (strcmp(arg, "reverse") == 0))
    {
        App_MagCalib_ToggleDirection();
    }
    else if ((strcmp(arg, "left") == 0) || (strcmp(arg, "l") == 0))
    {
        App_MagCalib_SetDirection(-1);
    }
    else if ((strcmp(arg, "right") == 0) || (strcmp(arg, "r") == 0))
    {
        App_MagCalib_SetDirection(1);
    }
    else if (MotorSerial_StartsWith(arg, "speed "))
    {
        App_MagCalib_SetTurnSpeed((int16_t)MotorSerial_ParseCommandInt(&arg[6], 40L));
    }
    else
    {
        MotorSerial_QueuePcString("[MCAL] usage: mcal start|stop|left|right|dir|speed N|status\r\n");
    }
#else
    (void)cmd;
    MotorSerial_QueuePcString("[CMD] mag calib disabled\r\n");
#endif
}

static int32_t MotorSerial_ParseCommandInt(const char *text, int32_t default_value)
{
    char *end;
    int32_t value;

    if (text == NULL)
    {
        return default_value;
    }

    while (*text == ' ')
    {
        text++;
    }

    value = (int32_t)strtol(text, &end, 10);
    if (end == text)
    {
        return default_value;
    }

    return value;
}

static uint8_t MotorSerial_StartsWith(const char *text, const char *prefix)
{
    while (*prefix != '\0')
    {
        if (*text != *prefix)
        {
            return 0U;
        }
        text++;
        prefix++;
    }

    return 1U;
}

static uint8_t MotorSerial_ToLower(uint8_t ch)
{
    if ((ch >= 'A') && (ch <= 'Z'))
    {
        return (uint8_t)(ch + ('a' - 'A'));
    }

    return ch;
}
