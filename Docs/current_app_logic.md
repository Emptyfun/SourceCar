# SourceCar_HAL 当前应用逻辑记录

本文记录当前这一版工程里“自己写的应用层/BSP/公共模块”的核心逻辑，方便以后回看和继续扩展。STM32Cube/HAL 生成代码不作为重点，只说明和应用逻辑有关的接口。

## 1. 总体结构

工程按层拆分：

- `Core/`：STM32Cube/HAL 启动、外设句柄、中断入口等底层工程代码。
- `App/`：应用层主逻辑，目前核心是 3-UART router。
- `BSP/`：板级外设封装，比如 LED、UART、ESP8266 使能脚。
- `Common/`：通用工具，目前主要是环形缓冲区。
- `Device/`：外设驱动，比如 TLV493D 驱动代码，目前应用层没有启用。
- `Protocol/`：协议相关占位/扩展模块。

主入口在 `Core/Src/main.c`：

```c
BSP_System_Init();
App_Init();

while (1)
{
    App_Loop();
}
```

也就是说，`main.c` 只负责启动 BSP 和 App，真正业务逻辑集中在 `App/`。

## 2. 启动流程

`BSP_System_Init()` 位于 `BSP/Src/bsp_system.c`，负责：

- `HAL_Init()`
- 系统时钟初始化
- GPIO 初始化
- USART1 初始化
- USART2 初始化

`App_Init()` 位于 `App/Src/app_main.c`，负责：

- 初始化 LED 抽象
- 初始化 ESP8266 使能脚 PB12
- 初始化 USART3，用作 ESP8266 UART
- 保持 TLV493D 禁用
- 初始化 3-UART router

当前 `App_Init()` 里的关键顺序是：

```c
BSP_LED_Init();
BSP_ESP8266_Init();
BSP_UART3_Init(115200U);
AppSerialBridge_Init();
```

## 3. 串口硬件分配

当前串口分配：

```text
USART1: PC / CH340
PA9  = USART1_TX
PA10 = USART1_RX
115200 8N1

USART2: car controller UART
PA2 = USART2_TX
PA3 = USART2_RX
115200 8N1

USART3: ESP8266 UART
PB10 = USART3_TX -> ESP8266_RX
PB11 = USART3_RX <- ESP8266_TX
115200 8N1

ESP8266 enable:
PB12 -> ESP8266_EN / CH_PD
PB12 LOW  = ESP8266 enabled
PB12 HIGH = ESP8266 disabled
```

注意：PB10 现在是 USART3_TX，不再作为 GPIO 控制 ESP8266。

## 4. ESP8266 使能控制

ESP8266 使能脚封装在：

- `BSP/Inc/bsp_esp8266.h`
- `BSP/Src/bsp_esp8266.c`

当前电平定义：

```c
#define ESP8266_ENABLE_LEVEL  GPIO_PIN_RESET
#define ESP8266_DISABLE_LEVEL GPIO_PIN_SET
```

含义：

```text
BSP_ESP8266_Enable()  -> PB12 LOW  -> ESP8266 enabled
BSP_ESP8266_Disable() -> PB12 HIGH -> ESP8266 disabled
```

`BSP_ESP8266_Init()` 会把 PB12 配置为推挽输出，并默认设置为关闭状态：

```text
PB12 HIGH, ESP8266 disabled
```

## 5. 3-UART Router

核心文件：

- `App/Src/app_serial_bridge.c`
- `App/Inc/app_serial_bridge.h`

虽然文件名仍叫 `serial_bridge`，当前实际功能已经是 3-UART router。

路由模式枚举：

```c
typedef enum
{
    APP_ROUTE_PC_CAR = 0,
    APP_ROUTE_WIFI_CAR,
    APP_ROUTE_PC_ESP_DEBUG
} AppSerialRouteMode;
```

默认模式：

```text
APP_ROUTE_PC_CAR
```

即开机默认：

```text
PC / USART1 <-> STM32 <-> USART2 / car controller
ESP8266 disabled
```

### APP_ROUTE_PC_CAR

PC 直接控制车控板：

```text
USART1_RX -> USART2_TX
USART2_RX -> USART1_TX
USART3_RX -> drop
```

这是默认模式。

### APP_ROUTE_WIFI_CAR

ESP8266 网络侧控制车控板：

```text
USART3_RX -> USART2_TX
USART2_RX -> USART3_TX
USART1_RX -> 只解析本地 # 命令，不转发普通 PC 数据到车控
```

当前启用了监控镜像：

```c
#define APP_SERIAL_MIRROR_CAR_TO_PC_IN_WIFI_MODE 1
```

所以在 `WIFI_CAR` 模式下，车控返回也会复制一份到 PC：

```text
USART2_RX -> USART3_TX
USART2_RX -> USART1_TX
```

这样 PC 能看到车控回复，但不能同时控制车，避免 USART1 和 USART3 同时写 USART2 导致命令交错。

如果执行 `#wifi_car` 时 ESP8266 原本处于关闭状态，固件会先拉低 PB12 启用 ESP8266，并短时间丢弃 USART3 的启动输出，避免 ESP8266 开机乱码、`ready`、`WIFI...` 等文本直接进入 USART2 车控通道。

当前 `WIFI_CAR` 模式支持 ESP8266 普通 AT `+IPD` 输入解析。比如 USART3 收到：

```text
+IPD,27:$spd:1000,-1000,1000,-1000#
```

STM32 只会把 payload 部分转发给 USART2：

```text
$spd:1000,-1000,1000,-1000#
```

同时兼容多连接格式：

```text
+IPD,0,27:$spd:1000,-1000,1000,-1000#
```

`WIFI_CAR` 模式不会自动给网络侧数据追加 `\r\n`，因为车控通道必须尽量保持原始字节流；如果车控协议需要换行，网络端或 ESP8266 侧应发送完整的协议字节。

### APP_ROUTE_PC_ESP_DEBUG

PC 调试 ESP8266 AT 指令：

```text
USART1_RX -> USART3_TX
USART3_RX -> USART1_TX
USART2_RX -> drop
```

这个模式用于配置 ESP8266，不是正常控车模式。

## 6. PC 本地命令

本地命令从 USART1 / PC 终端输入，必须以 `#` 开头。

支持命令：

```text
#pc_car
#wifi_car
#esp_debug
#esp_on
#esp_off
#status
```

命令行为：

```text
#pc_car:
  切换到 APP_ROUTE_PC_CAR

#wifi_car:
  如果 ESP8266 当前关闭，先启用 ESP8266
  切换到 APP_ROUTE_WIFI_CAR

#esp_debug:
  切换到 APP_ROUTE_PC_ESP_DEBUG

#esp_on:
  PB12 LOW, 启用 ESP8266

#esp_off:
  PB12 HIGH, 关闭 ESP8266
  如果当前在 WIFI_CAR 或 PC_ESP_DEBUG，则自动切回 PC_CAR

#status:
  打印当前路由模式和 ESP8266 启用状态
```

所有 `#` 命令都会被 STM32 本地消费：

```text
不会转发到 USART2
不会转发到 USART3
```

普通数据不以 `#` 开头时，按当前路由模式转发。

## 7. ESP Debug 模式的换行处理

ESP8266 AT 指令通常需要 `\r\n` 结尾。

当前代码在 `APP_ROUTE_PC_ESP_DEBUG` 模式下做了换行规范化：

```text
PC 发普通字符 -> 原样转发给 USART3
PC 发 \r      -> 转成 \r\n 发给 USART3
PC 发 \n      -> 转成 \r\n 发给 USART3
PC 发 \r\n    -> 只转成一个 \r\n
```

因此在 `#esp_debug` 模式下，从 PC 输入 `AT` 并按回车，STM32 会给 ESP8266 发送：

```text
AT\r\n
```

ESP8266 返回的 `OK` 会通过 USART3_RX 回到 STM32，再转发到 USART1_TX，最终显示在 PC 终端。

## 8. UART 中断和缓冲区

`app_serial_bridge.c` 拥有唯一的 `HAL_UART_RxCpltCallback()`。

不要再创建第二个 `HAL_UART_RxCpltCallback()`。

当前接收方式：

```c
HAL_UART_Receive_IT(..., 1);
```

三个 UART 都是一字节中断接收：

```text
USART1 -> uart1_rx_buffer
USART2 -> uart2_rx_buffer
USART3 -> uart3_rx_buffer
```

发送也使用环形缓冲区：

```text
uart1_tx_buffer
uart2_tx_buffer
uart3_tx_buffer
```

中断里只把字节放进 RX buffer，并立即重启下一字节接收。真正的路由转发在 `AppSerialBridge_Process()` 中执行。

环形缓冲区实现在：

- `Common/Inc/ring_buffer.h`
- `Common/Src/ring_buffer.c`

## 9. TLV493D 当前状态

TLV493D 驱动代码仍然存在：

- `Device/Src/dev_tlv493d_a1b6.c`
- `App/Src/app_tlv493d_a1b6_debug.c`

但当前应用层没有调用：

```c
App_TLV493D_A1B6_DebugRunOnce();
App_TLV493D_A1B6_DebugLoopTick();
```

也就是说 TLV493D 当前保持禁用，不会周期性占用 I2C，也不会往 USART1 打印调试信息。

## 10. 当前版本使用流程

开机默认：

```text
PB12 HIGH
ESP8266 disabled
mode = PC_CAR
PC USART1 <-> car USART2
```

PC 控车：

```text
默认即可，或输入 #pc_car
```

PC 配置 ESP8266：

```text
#esp_debug
#esp_on
AT
```

如果 ESP8266 已经开启，也可以直接：

```text
#esp_debug
AT
```

ESP8266 网络控车：

```text
#wifi_car
```

关闭 ESP8266 并回到 PC 控车：

```text
#esp_off
```

查看状态：

```text
#status
```

## 11. 后续扩展建议

当前已经支持网络到车控方向的 ESP8266 AT `+IPD` 解析：

```text
+IPD,<len>:payload
+IPD,<link_id>,<len>:payload
```

STM32 会只把 `payload` 转发给 USART2 车控。

如果后续需要把车控回复真正发回网络客户端，并且 ESP8266 仍工作在普通 AT 非透明模式，那么 USART2 到 USART3 方向也需要新增发送封装，例如：

```text
AT+CIPSEND=<len>
payload
```

如果 ESP8266 配成透明传输模式，或者 ESP8266 固件保证 UART 输入会自动发到网络，则当前 USART2 到 USART3 的原始转发可以继续保持。
