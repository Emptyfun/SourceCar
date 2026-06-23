/**
 * @file app_serial_bridge.c
 * @brief Three-UART router for PC, car-controller, and ESP8266 channels.
 *
 * USART1 is the PC / CH340 channel, USART2 is the car-controller channel, and
 * USART3 is the ESP8266 channel. The router owns the UART RX callbacks and
 * uses ring buffers so routing work stays out of interrupt context.
 */
#include "app_serial_bridge.h"

#include "app_config.h"
#include "bsp_esp8266.h"
#include "bsp_uart1.h"
#include "bsp_uart2.h"
#include "bsp_uart3.h"
#include "ring_buffer.h"

#include <stdio.h>
#include <string.h>

#define SERIAL_ROUTER_BUFFER_SIZE 256U
#define SERIAL_ROUTER_COMMAND_SIZE 64U
#define APP_SERIAL_FORWARD_CAR_TO_WIFI_MODE 0
#define APP_SERIAL_MIRROR_CAR_TO_PC_IN_WIFI_MODE 0
#define APP_SERIAL_WIFI_STARTUP_DROP_MS 3000U
#define SERIAL_ROUTER_IPD_PREFIX "+IPD,"
#define WIFI_PAYLOAD_LOG_SIZE 96U

#define ESP8266_BOOT_DELAY_MS 2000U
#define ESP8266_RECONNECT_INTERVAL_MS 3000U
#define ESP8266_AT_TIMEOUT_MS 3000U
#define ESP8266_WIFI_TIMEOUT_MS 15000U
#define ESP8266_TCP_TIMEOUT_MS 5000U
#define ESP8266_LINE_SIZE 128U

typedef enum
{
    APP_ROUTE_PC_CAR = 0,
    APP_ROUTE_WIFI_CAR,
    APP_ROUTE_PC_ESP_DEBUG
} AppSerialRouteMode;

typedef enum
{
    WIFI_IPD_FIND_PREFIX = 0,
    WIFI_IPD_MATCH_PREFIX,
    WIFI_IPD_HEADER,
    WIFI_IPD_PAYLOAD
} WifiIpdParseState;

typedef enum
{
    ESP_STATE_OFF = 0,
    ESP_STATE_BOOT,
    ESP_STATE_CHECK_AT,
    ESP_STATE_GET_VERSION,
    ESP_STATE_SET_ECHO,
    ESP_STATE_SET_STATION,
    ESP_STATE_JOIN_WIFI,
    ESP_STATE_GET_IP,
    ESP_STATE_SET_SINGLE_CONN,
    ESP_STATE_CONNECT_TCP,
    ESP_STATE_CONNECTED,
    ESP_STATE_ERROR,
    ESP_STATE_RECONNECT
} Esp8266State_t;

static uint8_t uart1_rx_storage[SERIAL_ROUTER_BUFFER_SIZE];
static uint8_t uart1_tx_storage[SERIAL_ROUTER_BUFFER_SIZE];
static uint8_t uart2_rx_storage[SERIAL_ROUTER_BUFFER_SIZE];
static uint8_t uart2_tx_storage[SERIAL_ROUTER_BUFFER_SIZE];
static uint8_t uart3_rx_storage[SERIAL_ROUTER_BUFFER_SIZE];
static uint8_t uart3_tx_storage[SERIAL_ROUTER_BUFFER_SIZE];

static RingBuffer_t uart1_rx_buffer;
static RingBuffer_t uart1_tx_buffer;
static RingBuffer_t uart2_rx_buffer;
static RingBuffer_t uart2_tx_buffer;
static RingBuffer_t uart3_rx_buffer;
static RingBuffer_t uart3_tx_buffer;

static uint8_t uart1_rx_byte;
static uint8_t uart2_rx_byte;
static uint8_t uart3_rx_byte;
static uint8_t uart1_tx_byte;
static uint8_t uart2_tx_byte;
static uint8_t uart3_tx_byte;

static volatile uint8_t uart1_tx_busy;
static volatile uint8_t uart2_tx_busy;
static volatile uint8_t uart3_tx_busy;

static AppSerialRouteMode g_route_mode;
static uint32_t wifi_startup_drop_tick;
static uint8_t wifi_startup_drop_active;
static WifiIpdParseState wifi_ipd_state;
static uint8_t wifi_ipd_prefix_index;
static uint16_t wifi_ipd_current_number;
static uint16_t wifi_ipd_payload_remaining;
static uint8_t wifi_ipd_saw_digit;
static char wifi_payload_log[WIFI_PAYLOAD_LOG_SIZE];
static uint16_t wifi_payload_log_len;
static uint8_t wifi_payload_log_truncated;

static char pc_command_buffer[SERIAL_ROUTER_COMMAND_SIZE];
static uint16_t pc_command_length;
static uint8_t pc_command_active;
static uint8_t pc_command_drop_until_eol;
static uint8_t pc_ignore_lf_once;
static uint8_t pc_at_line_start;
static uint8_t pc_esp_debug_ignore_lf_once;

static Esp8266State_t esp_state;
static uint32_t esp_state_tick;
static uint8_t esp_command_sent;
static uint8_t esp_auto_enabled;
static uint8_t esp_seen_ok;
static uint8_t esp_seen_error;
static uint8_t esp_seen_fail;
static uint8_t esp_seen_connect;
static uint8_t esp_seen_closed;
static uint8_t esp_seen_got_ip;
static uint8_t esp_seen_wifi_disconnect;
static char esp_line_buffer[ESP8266_LINE_SIZE];
static uint16_t esp_line_len;

static void SerialRouter_StartUart1Rx(void);
static void SerialRouter_StartUart2Rx(void);
static void SerialRouter_StartUart3Rx(void);
static void SerialRouter_StartUart1Tx(void);
static void SerialRouter_StartUart2Tx(void);
static void SerialRouter_StartUart3Tx(void);
static void SerialRouter_ProcessUart1Rx(void);
static void SerialRouter_ProcessUart2Rx(void);
static void SerialRouter_ProcessUart3Rx(void);
static void SerialRouter_HandlePcByte(uint8_t data);
static void SerialRouter_HandlePcCommandEol(uint8_t eol);
static void SerialRouter_HandleLocalCommand(const char *cmd);
static void SerialRouter_ForwardPcByte(uint8_t data);
static void SerialRouter_HandleWifiByte(uint8_t data);
static void SerialRouter_ResetWifiIpdParser(void);
static void SerialRouter_QueueUart1String(const char *s);
static void SerialRouter_QueueUart3String(const char *s);
static void SerialRouter_SendEspLine(const char *s);
static void SerialRouter_FlushBuffer(RingBuffer_t *rb);
static uint8_t SerialRouter_ShouldDropWifiStartupByte(void);
static void SerialRouter_StartPendingTx(void);
static const char *SerialRouter_ModeName(AppSerialRouteMode mode);
static uint8_t SerialRouter_ToLower(uint8_t ch);
static void AppSerialBridge_PrintStartupMessage(void);
static void SerialRouter_StartEspAutoConnect(void);
static void SerialRouter_ProcessEspAuto(uint32_t now);
static void SerialRouter_EnterEspState(Esp8266State_t state);
static void SerialRouter_ResetEspResponseFlags(void);
static void SerialRouter_HandleEspAutoByte(uint8_t data);
static void SerialRouter_HandleEspAutoLine(const char *line);
static uint8_t SerialRouter_LineHas(const char *line, const char *pattern);
static void SerialRouter_PrintEspConfig(void);
static void SerialRouter_ResetWifiPayloadLog(void);
static void SerialRouter_AddWifiPayloadLogByte(uint8_t data);
static void SerialRouter_PrintWifiPayloadLog(void);

void AppSerialBridge_Init(void)
{
    RingBuffer_Init(&uart1_rx_buffer, uart1_rx_storage, SERIAL_ROUTER_BUFFER_SIZE);
    RingBuffer_Init(&uart1_tx_buffer, uart1_tx_storage, SERIAL_ROUTER_BUFFER_SIZE);
    RingBuffer_Init(&uart2_rx_buffer, uart2_rx_storage, SERIAL_ROUTER_BUFFER_SIZE);
    RingBuffer_Init(&uart2_tx_buffer, uart2_tx_storage, SERIAL_ROUTER_BUFFER_SIZE);
    RingBuffer_Init(&uart3_rx_buffer, uart3_rx_storage, SERIAL_ROUTER_BUFFER_SIZE);
    RingBuffer_Init(&uart3_tx_buffer, uart3_tx_storage, SERIAL_ROUTER_BUFFER_SIZE);

    uart1_tx_busy = 0U;
    uart2_tx_busy = 0U;
    uart3_tx_busy = 0U;
#if APP_SERIAL_BOOT_PC_ESP_DEBUG
    g_route_mode = APP_ROUTE_PC_ESP_DEBUG;
#else
    g_route_mode = APP_ROUTE_PC_CAR;
#endif
    wifi_startup_drop_tick = 0U;
    wifi_startup_drop_active = 0U;
    SerialRouter_ResetWifiIpdParser();
    SerialRouter_ResetWifiPayloadLog();
    SerialRouter_ResetEspResponseFlags();
    esp_state = ESP_STATE_OFF;
    esp_state_tick = 0U;
    esp_command_sent = 0U;
    esp_line_len = 0U;
#if APP_SERIAL_BOOT_PC_ESP_DEBUG
    esp_auto_enabled = 0U;
#elif APP_ESP8266_AUTO_CONNECT
    esp_auto_enabled = 1U;
#else
    esp_auto_enabled = 0U;
#endif

    pc_command_length = 0U;
    pc_command_active = 0U;
    pc_command_drop_until_eol = 0U;
    pc_ignore_lf_once = 0U;
    pc_at_line_start = 1U;
    pc_esp_debug_ignore_lf_once = 0U;

    SerialRouter_StartUart1Rx();
    SerialRouter_StartUart2Rx();
    SerialRouter_StartUart3Rx();

    AppSerialBridge_PrintStartupMessage();
#if APP_SERIAL_BOOT_PC_ESP_DEBUG
    BSP_ESP8266_Enable();
    SerialRouter_QueueUart1String("[MODE] PC <-> ESP8266 DEBUG\r\n");
    SerialRouter_QueueUart1String("[ESP8266] PB12 LOW, enabled\r\n");
#endif
    SerialRouter_PrintEspConfig();
    if (esp_auto_enabled != 0U)
    {
        SerialRouter_StartEspAutoConnect();
    }
}

void AppSerialBridge_Process(void)
{
    uint32_t now = HAL_GetTick();

    SerialRouter_ProcessUart1Rx();
    SerialRouter_ProcessUart2Rx();
    SerialRouter_ProcessUart3Rx();
    SerialRouter_ProcessEspAuto(now);
    SerialRouter_StartPendingTx();
}

void AppSerialBridge_SendCarString(const char *s)
{
    if (s == NULL)
    {
        return;
    }

    while (*s != '\0')
    {
        if (!RingBuffer_Push(&uart2_tx_buffer, (uint8_t)*s))
        {
            break;
        }
        s++;
    }

    SerialRouter_StartUart2Tx();
}

void AppSerialBridge_EmergencyStop(void)
{
    AppSerialBridge_SendCarString("$pwm:0,0,0,0#");
    SerialRouter_QueueUart1String("[STOP] key long -> USART2 $pwm:0,0,0,0#\r\n");
}

#if APP_ENABLE_SERIAL_BRIDGE
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        (void)RingBuffer_Push(&uart1_rx_buffer, uart1_rx_byte);
        SerialRouter_StartUart1Rx();
    }
    else if (huart->Instance == USART2)
    {
        (void)RingBuffer_Push(&uart2_rx_buffer, uart2_rx_byte);
        SerialRouter_StartUart2Rx();
    }
    else if (huart->Instance == USART3)
    {
        (void)RingBuffer_Push(&uart3_rx_buffer, uart3_rx_byte);
        SerialRouter_StartUart3Rx();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart1_tx_busy = 0U;
        SerialRouter_StartUart1Tx();
    }
    else if (huart->Instance == USART2)
    {
        uart2_tx_busy = 0U;
        SerialRouter_StartUart2Tx();
    }
    else if (huart->Instance == USART3)
    {
        uart3_tx_busy = 0U;
        SerialRouter_StartUart3Tx();
    }
}
#endif

static void SerialRouter_StartUart1Rx(void)
{
    (void)HAL_UART_Receive_IT(BSP_UART1_GetHandle(), &uart1_rx_byte, 1);
}

static void SerialRouter_StartUart2Rx(void)
{
    (void)HAL_UART_Receive_IT(BSP_UART2_GetHandle(), &uart2_rx_byte, 1);
}

static void SerialRouter_StartUart3Rx(void)
{
    (void)HAL_UART_Receive_IT(BSP_UART3_GetHandle(), &uart3_rx_byte, 1);
}

static void SerialRouter_StartUart1Tx(void)
{
    if (uart1_tx_busy)
    {
        return;
    }

    if (RingBuffer_Pop(&uart1_tx_buffer, &uart1_tx_byte))
    {
        uart1_tx_busy = 1U;
        if (HAL_UART_Transmit_IT(BSP_UART1_GetHandle(), &uart1_tx_byte, 1) != HAL_OK)
        {
            uart1_tx_busy = 0U;
        }
    }
}

static void SerialRouter_StartUart2Tx(void)
{
    if (uart2_tx_busy)
    {
        return;
    }

    if (RingBuffer_Pop(&uart2_tx_buffer, &uart2_tx_byte))
    {
        uart2_tx_busy = 1U;
        if (HAL_UART_Transmit_IT(BSP_UART2_GetHandle(), &uart2_tx_byte, 1) != HAL_OK)
        {
            uart2_tx_busy = 0U;
        }
    }
}

static void SerialRouter_StartUart3Tx(void)
{
    if (uart3_tx_busy)
    {
        return;
    }

    if (RingBuffer_Pop(&uart3_tx_buffer, &uart3_tx_byte))
    {
        uart3_tx_busy = 1U;
        if (HAL_UART_Transmit_IT(BSP_UART3_GetHandle(), &uart3_tx_byte, 1) != HAL_OK)
        {
            uart3_tx_busy = 0U;
        }
    }
}

static void SerialRouter_ProcessUart1Rx(void)
{
    uint8_t data;

    while (RingBuffer_Pop(&uart1_rx_buffer, &data))
    {
        SerialRouter_HandlePcByte(data);
    }
}

static void SerialRouter_ProcessUart2Rx(void)
{
    uint8_t data;

    while (RingBuffer_Pop(&uart2_rx_buffer, &data))
    {
        switch (g_route_mode)
        {
        case APP_ROUTE_PC_CAR:
            (void)RingBuffer_Push(&uart1_tx_buffer, data);
            break;

        case APP_ROUTE_WIFI_CAR:
#if APP_SERIAL_FORWARD_CAR_TO_WIFI_MODE
            (void)RingBuffer_Push(&uart3_tx_buffer, data);
#endif
#if APP_SERIAL_MIRROR_CAR_TO_PC_IN_WIFI_MODE
            (void)RingBuffer_Push(&uart1_tx_buffer, data);
#endif
            break;

        case APP_ROUTE_PC_ESP_DEBUG:
        default:
            break;
        }
    }
}

static void SerialRouter_ProcessUart3Rx(void)
{
    uint8_t data;

    while (RingBuffer_Pop(&uart3_rx_buffer, &data))
    {
        SerialRouter_HandleEspAutoByte(data);

        switch (g_route_mode)
        {
        case APP_ROUTE_WIFI_CAR:
            if (SerialRouter_ShouldDropWifiStartupByte())
            {
                break;
            }

            SerialRouter_HandleWifiByte(data);
            break;

        case APP_ROUTE_PC_ESP_DEBUG:
            (void)RingBuffer_Push(&uart1_tx_buffer, data);
            break;

        case APP_ROUTE_PC_CAR:
        default:
            break;
        }
    }
}

static void SerialRouter_HandlePcByte(uint8_t data)
{
    if (pc_ignore_lf_once && (data == '\n'))
    {
        pc_ignore_lf_once = 0U;
        pc_at_line_start = 1U;
        return;
    }
    pc_ignore_lf_once = 0U;

    if (pc_command_drop_until_eol)
    {
        if ((data == '\r') || (data == '\n'))
        {
            pc_command_drop_until_eol = 0U;
            pc_command_active = 0U;
            pc_command_length = 0U;
            pc_at_line_start = 1U;
            if (data == '\r')
            {
                pc_ignore_lf_once = 1U;
            }
        }
        return;
    }

    if (pc_command_active)
    {
        if ((data == '\r') || (data == '\n'))
        {
            SerialRouter_HandlePcCommandEol(data);
            return;
        }

        if (pc_command_length < (uint16_t)(SERIAL_ROUTER_COMMAND_SIZE - 1U))
        {
            pc_command_buffer[pc_command_length] = (char)SerialRouter_ToLower(data);
            pc_command_length++;
        }
        else
        {
            pc_command_drop_until_eol = 1U;
            pc_command_active = 0U;
            pc_command_length = 0U;
            SerialRouter_QueueUart1String("[CMD] command too long\r\n");
        }
        return;
    }

    if (pc_at_line_start && (data == '#'))
    {
        pc_command_active = 1U;
        pc_command_length = 0U;
        return;
    }

    SerialRouter_ForwardPcByte(data);

    if ((data == '\r') || (data == '\n'))
    {
        pc_at_line_start = 1U;
    }
    else
    {
        pc_at_line_start = 0U;
    }
}

static void SerialRouter_HandlePcCommandEol(uint8_t eol)
{
    pc_command_buffer[pc_command_length] = '\0';

    if (pc_command_length > 0U)
    {
        SerialRouter_HandleLocalCommand(pc_command_buffer);
    }

    pc_command_active = 0U;
    pc_command_length = 0U;
    pc_at_line_start = 1U;

    if (eol == '\r')
    {
        pc_ignore_lf_once = 1U;
    }
}

static void SerialRouter_HandleLocalCommand(const char *cmd)
{
    if (strcmp(cmd, "pc_car") == 0)
    {
        esp_auto_enabled = 0U;
        g_route_mode = APP_ROUTE_PC_CAR;
        wifi_startup_drop_active = 0U;
        SerialRouter_ResetWifiIpdParser();
        SerialRouter_QueueUart1String("[MODE] PC <-> CAR\r\n");
        return;
    }

    if (strcmp(cmd, "wifi_car") == 0)
    {
        uint8_t esp_was_off = !BSP_ESP8266_IsEnabled();

        esp_auto_enabled = 0U;
        if (!BSP_ESP8266_IsEnabled())
        {
            BSP_ESP8266_Enable();
        }
        SerialRouter_FlushBuffer(&uart3_rx_buffer);
        SerialRouter_ResetWifiIpdParser();
        g_route_mode = APP_ROUTE_WIFI_CAR;
        SerialRouter_QueueUart1String("[MODE] WIFI <-> CAR\r\n");
        if (esp_was_off)
        {
            wifi_startup_drop_tick = HAL_GetTick();
            wifi_startup_drop_active = 1U;
            SerialRouter_QueueUart1String("[WIFI] dropping ESP8266 startup bytes before car routing\r\n");
        }
        return;
    }

    if (strcmp(cmd, "esp_debug") == 0)
    {
        esp_auto_enabled = 0U;
        g_route_mode = APP_ROUTE_PC_ESP_DEBUG;
        wifi_startup_drop_active = 0U;
        SerialRouter_ResetWifiIpdParser();
        SerialRouter_QueueUart1String("[MODE] PC <-> ESP8266 DEBUG\r\n");
        return;
    }

    if (strcmp(cmd, "esp_on") == 0)
    {
        BSP_ESP8266_Enable();
        SerialRouter_QueueUart1String("[ESP8266] PB12 LOW, enabled\r\n");
        return;
    }

    if (strcmp(cmd, "esp_off") == 0)
    {
        esp_auto_enabled = 0U;
        BSP_ESP8266_Disable();
        if ((g_route_mode == APP_ROUTE_WIFI_CAR) ||
            (g_route_mode == APP_ROUTE_PC_ESP_DEBUG))
        {
            g_route_mode = APP_ROUTE_PC_CAR;
        }
        wifi_startup_drop_active = 0U;
        SerialRouter_ResetWifiIpdParser();
        SerialRouter_QueueUart1String("[ESP8266] PB12 HIGH, disabled; mode PC <-> CAR\r\n");
        return;
    }

    if (strcmp(cmd, "status") == 0)
    {
        SerialRouter_QueueUart1String("[STATUS] mode=");
        SerialRouter_QueueUart1String(SerialRouter_ModeName(g_route_mode));
        SerialRouter_QueueUart1String("\r\n");
        if (BSP_ESP8266_IsEnabled())
        {
            SerialRouter_QueueUart1String("[STATUS] ESP8266=enabled, PB12 LOW\r\n");
        }
        else
        {
            SerialRouter_QueueUart1String("[STATUS] ESP8266=disabled, PB12 HIGH\r\n");
        }
        SerialRouter_QueueUart1String("[STATUS] ESP auto=");
        SerialRouter_QueueUart1String((esp_auto_enabled != 0U) ? "on\r\n" : "off\r\n");
        return;
    }

    SerialRouter_QueueUart1String("[CMD] unknown command\r\n");
}

static void SerialRouter_ForwardPcByte(uint8_t data)
{
    switch (g_route_mode)
    {
    case APP_ROUTE_PC_CAR:
        pc_esp_debug_ignore_lf_once = 0U;
        (void)RingBuffer_Push(&uart2_tx_buffer, data);
        break;

    case APP_ROUTE_PC_ESP_DEBUG:
        if (pc_esp_debug_ignore_lf_once && (data == '\n'))
        {
            pc_esp_debug_ignore_lf_once = 0U;
            break;
        }

        pc_esp_debug_ignore_lf_once = 0U;

        if ((data == '\r') || (data == '\n'))
        {
            (void)RingBuffer_Push(&uart3_tx_buffer, '\r');
            (void)RingBuffer_Push(&uart3_tx_buffer, '\n');
            if (data == '\r')
            {
                pc_esp_debug_ignore_lf_once = 1U;
            }
        }
        else
        {
            (void)RingBuffer_Push(&uart3_tx_buffer, data);
        }
        break;

    case APP_ROUTE_WIFI_CAR:
    default:
        pc_esp_debug_ignore_lf_once = 0U;
        break;
    }
}

static void SerialRouter_HandleWifiByte(uint8_t data)
{
    switch (wifi_ipd_state)
    {
    case WIFI_IPD_FIND_PREFIX:
        if (data == (uint8_t)SERIAL_ROUTER_IPD_PREFIX[0])
        {
            wifi_ipd_prefix_index = 1U;
            wifi_ipd_state = WIFI_IPD_MATCH_PREFIX;
        }
        break;

    case WIFI_IPD_MATCH_PREFIX:
        if (data == (uint8_t)SERIAL_ROUTER_IPD_PREFIX[wifi_ipd_prefix_index])
        {
            wifi_ipd_prefix_index++;
            if (SERIAL_ROUTER_IPD_PREFIX[wifi_ipd_prefix_index] == '\0')
            {
                wifi_ipd_state = WIFI_IPD_HEADER;
                wifi_ipd_current_number = 0U;
                wifi_ipd_payload_remaining = 0U;
                wifi_ipd_saw_digit = 0U;
            }
        }
        else if (data == (uint8_t)SERIAL_ROUTER_IPD_PREFIX[0])
        {
            wifi_ipd_prefix_index = 1U;
        }
        else
        {
            SerialRouter_ResetWifiIpdParser();
        }
        break;

    case WIFI_IPD_HEADER:
        if ((data >= '0') && (data <= '9'))
        {
            wifi_ipd_current_number =
                (uint16_t)((wifi_ipd_current_number * 10U) + (uint16_t)(data - '0'));
            wifi_ipd_saw_digit = 1U;
        }
        else if (data == ',')
        {
            if (!wifi_ipd_saw_digit)
            {
                SerialRouter_ResetWifiIpdParser();
            }
            else
            {
                wifi_ipd_current_number = 0U;
                wifi_ipd_saw_digit = 0U;
            }
        }
        else if (data == ':')
        {
            if (!wifi_ipd_saw_digit)
            {
                SerialRouter_ResetWifiIpdParser();
            }
            else
            {
                wifi_ipd_payload_remaining = wifi_ipd_current_number;
                if (wifi_ipd_payload_remaining == 0U)
                {
                    SerialRouter_ResetWifiIpdParser();
                }
                else
                {
                    SerialRouter_ResetWifiPayloadLog();
                    wifi_ipd_state = WIFI_IPD_PAYLOAD;
                }
            }
        }
        else
        {
            SerialRouter_ResetWifiIpdParser();
        }
        break;

    case WIFI_IPD_PAYLOAD:
        (void)RingBuffer_Push(&uart2_tx_buffer, data);
        SerialRouter_AddWifiPayloadLogByte(data);
        wifi_ipd_payload_remaining--;
        if (wifi_ipd_payload_remaining == 0U)
        {
            SerialRouter_PrintWifiPayloadLog();
            SerialRouter_ResetWifiIpdParser();
        }
        break;

    default:
        SerialRouter_ResetWifiIpdParser();
        break;
    }
}

static void SerialRouter_ResetWifiIpdParser(void)
{
    wifi_ipd_state = WIFI_IPD_FIND_PREFIX;
    wifi_ipd_prefix_index = 0U;
    wifi_ipd_current_number = 0U;
    wifi_ipd_payload_remaining = 0U;
    wifi_ipd_saw_digit = 0U;
}

static void SerialRouter_QueueUart1String(const char *s)
{
    if (s == NULL)
    {
        return;
    }

    while (*s != '\0')
    {
        if (!RingBuffer_Push(&uart1_tx_buffer, (uint8_t)*s))
        {
            break;
        }
        s++;
    }

    SerialRouter_StartUart1Tx();
}

static void SerialRouter_QueueUart3String(const char *s)
{
    if (s == NULL)
    {
        return;
    }

    while (*s != '\0')
    {
        if (!RingBuffer_Push(&uart3_tx_buffer, (uint8_t)*s))
        {
            break;
        }
        s++;
    }

    SerialRouter_StartUart3Tx();
}

static void SerialRouter_SendEspLine(const char *s)
{
    SerialRouter_QueueUart3String(s);
    SerialRouter_QueueUart3String("\r\n");
}

static void SerialRouter_StartEspAutoConnect(void)
{
    BSP_ESP8266_Enable();
    SerialRouter_FlushBuffer(&uart3_rx_buffer);
    SerialRouter_ResetWifiIpdParser();
    SerialRouter_ResetWifiPayloadLog();
    wifi_startup_drop_active = 0U;
    g_route_mode = APP_ROUTE_PC_CAR;
    SerialRouter_EnterEspState(ESP_STATE_BOOT);
}

static void SerialRouter_ProcessEspAuto(uint32_t now)
{
    char cmd[128];

    if (esp_auto_enabled == 0U)
    {
        return;
    }

    switch (esp_state)
    {
    case ESP_STATE_BOOT:
        if ((uint32_t)(now - esp_state_tick) >= ESP8266_BOOT_DELAY_MS)
        {
            SerialRouter_EnterEspState(ESP_STATE_CHECK_AT);
        }
        break;

    case ESP_STATE_CHECK_AT:
        if (esp_command_sent == 0U)
        {
            SerialRouter_ResetEspResponseFlags();
            SerialRouter_SendEspLine("AT");
            esp_command_sent = 1U;
            esp_state_tick = now;
        }
        if (esp_seen_ok != 0U)
        {
            SerialRouter_QueueUart1String("[ESP] AT OK\r\n");
            SerialRouter_EnterEspState(ESP_STATE_GET_VERSION);
        }
        else if ((esp_seen_error != 0U) ||
                 ((uint32_t)(now - esp_state_tick) >= ESP8266_AT_TIMEOUT_MS))
        {
            SerialRouter_EnterEspState(ESP_STATE_ERROR);
        }
        break;

    case ESP_STATE_GET_VERSION:
        if (esp_command_sent == 0U)
        {
            SerialRouter_ResetEspResponseFlags();
            SerialRouter_SendEspLine("AT+GMR");
            esp_command_sent = 1U;
            esp_state_tick = now;
        }
        if (esp_seen_ok != 0U)
        {
            SerialRouter_EnterEspState(ESP_STATE_SET_ECHO);
        }
        else if ((esp_seen_error != 0U) ||
                 ((uint32_t)(now - esp_state_tick) >= ESP8266_AT_TIMEOUT_MS))
        {
            SerialRouter_EnterEspState(ESP_STATE_SET_ECHO);
        }
        break;

    case ESP_STATE_SET_ECHO:
        if (esp_command_sent == 0U)
        {
            SerialRouter_ResetEspResponseFlags();
            SerialRouter_SendEspLine("ATE1");
            esp_command_sent = 1U;
            esp_state_tick = now;
        }
        if (esp_seen_ok != 0U)
        {
            SerialRouter_EnterEspState(ESP_STATE_SET_STATION);
        }
        else if ((esp_seen_error != 0U) ||
                 ((uint32_t)(now - esp_state_tick) >= ESP8266_AT_TIMEOUT_MS))
        {
            SerialRouter_EnterEspState(ESP_STATE_ERROR);
        }
        break;

    case ESP_STATE_SET_STATION:
        if (esp_command_sent == 0U)
        {
            SerialRouter_ResetEspResponseFlags();
            SerialRouter_SendEspLine("AT+CWMODE=1");
            esp_command_sent = 1U;
            esp_state_tick = now;
        }
        if (esp_seen_ok != 0U)
        {
            SerialRouter_QueueUart1String("[ESP] CWMODE=1 OK\r\n");
            SerialRouter_EnterEspState(ESP_STATE_JOIN_WIFI);
        }
        else if ((esp_seen_error != 0U) ||
                 ((uint32_t)(now - esp_state_tick) >= ESP8266_AT_TIMEOUT_MS))
        {
            SerialRouter_EnterEspState(ESP_STATE_ERROR);
        }
        break;

    case ESP_STATE_JOIN_WIFI:
        if (esp_command_sent == 0U)
        {
            SerialRouter_ResetEspResponseFlags();
            snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"",
                     APP_ESP8266_WIFI_SSID,
                     APP_ESP8266_WIFI_PASSWORD);
            SerialRouter_QueueUart1String("[ESP] joining WiFi: ");
            SerialRouter_QueueUart1String(APP_ESP8266_WIFI_SSID);
            SerialRouter_QueueUart1String("\r\n");
            SerialRouter_SendEspLine(cmd);
            esp_command_sent = 1U;
            esp_state_tick = now;
        }
        if (esp_seen_ok != 0U)
        {
            SerialRouter_EnterEspState(ESP_STATE_GET_IP);
        }
        else if ((esp_seen_error != 0U) || (esp_seen_fail != 0U) ||
                 ((uint32_t)(now - esp_state_tick) >= ESP8266_WIFI_TIMEOUT_MS))
        {
            SerialRouter_EnterEspState(ESP_STATE_ERROR);
        }
        break;

    case ESP_STATE_GET_IP:
        if (esp_command_sent == 0U)
        {
            SerialRouter_ResetEspResponseFlags();
            SerialRouter_SendEspLine("AT+CIFSR");
            esp_command_sent = 1U;
            esp_state_tick = now;
        }
        if (esp_seen_ok != 0U)
        {
            SerialRouter_EnterEspState(ESP_STATE_SET_SINGLE_CONN);
        }
        else if ((esp_seen_error != 0U) ||
                 ((uint32_t)(now - esp_state_tick) >= ESP8266_AT_TIMEOUT_MS))
        {
            SerialRouter_EnterEspState(ESP_STATE_SET_SINGLE_CONN);
        }
        break;

    case ESP_STATE_SET_SINGLE_CONN:
        if (esp_command_sent == 0U)
        {
            SerialRouter_ResetEspResponseFlags();
            SerialRouter_SendEspLine("AT+CIPMUX=0");
            esp_command_sent = 1U;
            esp_state_tick = now;
        }
        if (esp_seen_ok != 0U)
        {
            SerialRouter_EnterEspState(ESP_STATE_CONNECT_TCP);
        }
        else if ((esp_seen_error != 0U) ||
                 ((uint32_t)(now - esp_state_tick) >= ESP8266_AT_TIMEOUT_MS))
        {
            SerialRouter_EnterEspState(ESP_STATE_ERROR);
        }
        break;

    case ESP_STATE_CONNECT_TCP:
        if (esp_command_sent == 0U)
        {
            SerialRouter_ResetEspResponseFlags();
            snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u",
                     APP_ESP8266_TCP_HOST,
                     (unsigned int)APP_ESP8266_TCP_PORT);
            SerialRouter_QueueUart1String("[ESP] connecting TCP ");
            SerialRouter_QueueUart1String(APP_ESP8266_TCP_HOST);
            SerialRouter_QueueUart1String(":");
            snprintf(cmd, sizeof(cmd), "%u", (unsigned int)APP_ESP8266_TCP_PORT);
            SerialRouter_QueueUart1String(cmd);
            SerialRouter_QueueUart1String("\r\n");
            snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u",
                     APP_ESP8266_TCP_HOST,
                     (unsigned int)APP_ESP8266_TCP_PORT);
            SerialRouter_SendEspLine(cmd);
            esp_command_sent = 1U;
            esp_state_tick = now;
        }
        if ((esp_seen_ok != 0U) || (esp_seen_connect != 0U))
        {
            SerialRouter_EnterEspState(ESP_STATE_CONNECTED);
        }
        else if ((esp_seen_error != 0U) || (esp_seen_fail != 0U) ||
                 ((uint32_t)(now - esp_state_tick) >= ESP8266_TCP_TIMEOUT_MS))
        {
            SerialRouter_EnterEspState(ESP_STATE_ERROR);
        }
        break;

    case ESP_STATE_CONNECTED:
        if ((esp_seen_closed != 0U) || (esp_seen_wifi_disconnect != 0U) ||
            (esp_seen_error != 0U) || (esp_seen_fail != 0U))
        {
            SerialRouter_QueueUart1String("[ESP] connection lost, reconnect\r\n");
            SerialRouter_EnterEspState(ESP_STATE_RECONNECT);
        }
        break;

    case ESP_STATE_ERROR:
        SerialRouter_QueueUart1String("[ESP] error, reconnect\r\n");
        SerialRouter_EnterEspState(ESP_STATE_RECONNECT);
        break;

    case ESP_STATE_RECONNECT:
        if (esp_command_sent == 0U)
        {
            esp_command_sent = 1U;
            esp_state_tick = now;
        }
        if ((uint32_t)(now - esp_state_tick) >= ESP8266_RECONNECT_INTERVAL_MS)
        {
            SerialRouter_ResetWifiIpdParser();
            SerialRouter_ResetWifiPayloadLog();
            SerialRouter_EnterEspState(ESP_STATE_CHECK_AT);
        }
        break;

    case ESP_STATE_OFF:
    default:
        break;
    }
}

static void SerialRouter_EnterEspState(Esp8266State_t state)
{
    esp_state = state;
    esp_state_tick = HAL_GetTick();
    esp_command_sent = 0U;
    esp_line_len = 0U;
    SerialRouter_ResetEspResponseFlags();

    if (state == ESP_STATE_BOOT)
    {
        SerialRouter_QueueUart1String("[ESP] boot\r\n");
    }
    else if (state == ESP_STATE_CONNECTED)
    {
        g_route_mode = APP_ROUTE_WIFI_CAR;
        wifi_startup_drop_active = 0U;
        SerialRouter_ResetWifiIpdParser();
        SerialRouter_QueueUart1String("[ESP] TCP CONNECTED\r\n");
        SerialRouter_QueueUart1String("[MODE] WIFI <-> CAR\r\n");
    }
}

static void SerialRouter_ResetEspResponseFlags(void)
{
    esp_seen_ok = 0U;
    esp_seen_error = 0U;
    esp_seen_fail = 0U;
    esp_seen_connect = 0U;
    esp_seen_closed = 0U;
    esp_seen_got_ip = 0U;
    esp_seen_wifi_disconnect = 0U;
}

static void SerialRouter_HandleEspAutoByte(uint8_t data)
{
    if (esp_auto_enabled == 0U)
    {
        return;
    }

    if ((data == '\r') || (data == '\n'))
    {
        if (esp_line_len > 0U)
        {
            esp_line_buffer[esp_line_len] = '\0';
            SerialRouter_HandleEspAutoLine(esp_line_buffer);
            esp_line_len = 0U;
        }
        return;
    }

    if (esp_line_len < (uint16_t)(sizeof(esp_line_buffer) - 1U))
    {
        esp_line_buffer[esp_line_len] = (char)data;
        esp_line_len++;
    }
    else
    {
        esp_line_len = 0U;
    }
}

static void SerialRouter_HandleEspAutoLine(const char *line)
{
    if ((line == NULL) || (line[0] == '\0'))
    {
        return;
    }

    if (strcmp(line, "OK") == 0)
    {
        esp_seen_ok = 1U;
    }
    if (strcmp(line, "ERROR") == 0)
    {
        esp_seen_error = 1U;
    }
    if ((strcmp(line, "FAIL") == 0) || SerialRouter_LineHas(line, "FAIL"))
    {
        esp_seen_fail = 1U;
    }
    if (SerialRouter_LineHas(line, "WIFI GOT IP"))
    {
        esp_seen_got_ip = 1U;
        SerialRouter_QueueUart1String("[ESP] WIFI GOT IP\r\n");
    }
    if (SerialRouter_LineHas(line, "WIFI DISCONNECT"))
    {
        esp_seen_wifi_disconnect = 1U;
    }
    if (SerialRouter_LineHas(line, "CLOSED"))
    {
        esp_seen_closed = 1U;
    }
    if (SerialRouter_LineHas(line, "CONNECT"))
    {
        esp_seen_connect = 1U;
    }

    if ((esp_state == ESP_STATE_GET_VERSION) &&
        (strcmp(line, "AT+GMR") != 0) &&
        (strcmp(line, "OK") != 0))
    {
        SerialRouter_QueueUart1String("[ESP] version: ");
        SerialRouter_QueueUart1String(line);
        SerialRouter_QueueUart1String("\r\n");
    }

    if ((esp_state == ESP_STATE_GET_IP) &&
        (SerialRouter_LineHas(line, "STAIP") || SerialRouter_LineHas(line, "+CIFSR")))
    {
        SerialRouter_QueueUart1String("[ESP] IP: ");
        SerialRouter_QueueUart1String(line);
        SerialRouter_QueueUart1String("\r\n");
    }
}

static uint8_t SerialRouter_LineHas(const char *line, const char *pattern)
{
    return (strstr(line, pattern) != NULL) ? 1U : 0U;
}

static void SerialRouter_PrintEspConfig(void)
{
#if APP_ESP8266_AUTO_CONNECT
    SerialRouter_QueueUart1String("[ESP] auto WiFi SSID=");
    SerialRouter_QueueUart1String(APP_ESP8266_WIFI_SSID);
    SerialRouter_QueueUart1String("\r\n");
    SerialRouter_QueueUart1String("[ESP] auto TCP ");
    SerialRouter_QueueUart1String(APP_ESP8266_TCP_HOST);
    SerialRouter_QueueUart1String(":");
    {
        char port_text[12];
        snprintf(port_text, sizeof(port_text), "%u", (unsigned int)APP_ESP8266_TCP_PORT);
        SerialRouter_QueueUart1String(port_text);
    }
    SerialRouter_QueueUart1String("\r\n");
#endif
}

static void SerialRouter_ResetWifiPayloadLog(void)
{
    wifi_payload_log_len = 0U;
    wifi_payload_log_truncated = 0U;
    wifi_payload_log[0] = '\0';
}

static void SerialRouter_AddWifiPayloadLogByte(uint8_t data)
{
    if (wifi_payload_log_len < (uint16_t)(sizeof(wifi_payload_log) - 1U))
    {
        wifi_payload_log[wifi_payload_log_len] = (char)data;
        wifi_payload_log_len++;
        wifi_payload_log[wifi_payload_log_len] = '\0';
    }
    else
    {
        wifi_payload_log_truncated = 1U;
    }
}

static void SerialRouter_PrintWifiPayloadLog(void)
{
    SerialRouter_QueueUart1String("[ESP] RX TCP: ");
    SerialRouter_QueueUart1String(wifi_payload_log);
    if (wifi_payload_log_truncated != 0U)
    {
        SerialRouter_QueueUart1String("...");
    }
    SerialRouter_QueueUart1String("\r\n");
    SerialRouter_QueueUart1String("[ESP->MOTOR] ");
    SerialRouter_QueueUart1String(wifi_payload_log);
    if (wifi_payload_log_truncated != 0U)
    {
        SerialRouter_QueueUart1String("...");
    }
    SerialRouter_QueueUart1String("\r\n");
}

static void SerialRouter_FlushBuffer(RingBuffer_t *rb)
{
    uint8_t data;

    while (RingBuffer_Pop(rb, &data))
    {
    }
}

static uint8_t SerialRouter_ShouldDropWifiStartupByte(void)
{
    if (!wifi_startup_drop_active)
    {
        return 0U;
    }

    if ((uint32_t)(HAL_GetTick() - wifi_startup_drop_tick) < APP_SERIAL_WIFI_STARTUP_DROP_MS)
    {
        return 1U;
    }

    wifi_startup_drop_active = 0U;
    return 0U;
}

static void SerialRouter_StartPendingTx(void)
{
    SerialRouter_StartUart1Tx();
    SerialRouter_StartUart2Tx();
    SerialRouter_StartUart3Tx();
}

static const char *SerialRouter_ModeName(AppSerialRouteMode mode)
{
    switch (mode)
    {
    case APP_ROUTE_PC_CAR:
        return "PC_CAR";

    case APP_ROUTE_WIFI_CAR:
        return "WIFI_CAR";

    case APP_ROUTE_PC_ESP_DEBUG:
        return "PC_ESP_DEBUG";

    default:
        return "UNKNOWN";
    }
}

static uint8_t SerialRouter_ToLower(uint8_t ch)
{
    if ((ch >= 'A') && (ch <= 'Z'))
    {
        return (uint8_t)(ch + ('a' - 'A'));
    }

    return ch;
}

static void AppSerialBridge_PrintStartupMessage(void)
{
#if APP_SERIAL_BOOT_PC_ESP_DEBUG
    static const char msg[] =
        "\r\n"
        "[BOOT] 3-UART router ready.\r\n"
        "[MODE] default: PC <-> ESP8266 DEBUG.\r\n"
        "[CMD] direct AT command mode, use CRLF line ending.\r\n"
        "[CMD] #pc_car, #wifi_car, #esp_debug, #esp_on, #esp_off, #status\r\n";
#else
    static const char msg[] =
        "\r\n"
        "[BOOT] 3-UART router ready.\r\n"
        "[MODE] default: PC <-> CAR.\r\n"
        "[CMD] #pc_car, #wifi_car, #esp_debug, #esp_on, #esp_off, #status\r\n"
        "[ESP8266] PB12 HIGH, disabled.\r\n";
#endif

    SerialRouter_QueueUart1String(msg);
}
