# SourceCar_HAL 当前应用逻辑记录

本文记录当前工程里手写的应用层、BSP、公共模块和设备驱动逻辑。STM32Cube/HAL
生成代码不作为重点，只说明它和当前应用行为的连接点。

## 1. 当前固件目标

当前默认配置是 CS1238 ADC 调试/采样版本，而不是车控串口路由版本。

配置入口在 `App/Inc/app_config.h`：

```c
#define APP_ENABLE_CS1238          1
#define APP_ENABLE_LED_HEARTBEAT   1
#define APP_ENABLE_SERIAL_BRIDGE   0
#define APP_ENABLE_ESP8266         0
#define APP_ENABLE_UART2           0
#define APP_ENABLE_UART3           0
```

也就是说：

- USART1 保持启用，用作调试输出。
- LED 心跳保持启用，用来观察主循环是否正常运行。
- CS1238 采样任务启用。
- 3-UART router、ESP8266、USART2、USART3 当前默认不启动，避免影响 ADC 调试时序。

## 2. 总体结构

工程按层拆分：

- `Core/`：STM32Cube/HAL 启动、外设句柄、中断入口等底层工程代码。
- `App/`：应用层主逻辑，负责按 `app_config.h` 启停功能模块。
- `BSP/`：板级外设封装，比如时钟、GPIO、UART、LED、ESP8266 使能脚。
- `Common/`：通用工具，目前主要是环形缓冲区。
- `Device/`：外设驱动，目前包含 CS1238。
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

`main.c` 只负责启动 BSP 和 App，真正业务逻辑集中在 `App/`。

## 3. 启动流程

`BSP_System_Init()` 位于 `BSP/Src/bsp_system.c`，当前负责：

- `HAL_Init()`
- 系统时钟初始化
- GPIO 初始化
- USART1 初始化
- 如果 `APP_ENABLE_UART2` 为 1，再初始化 USART2

`App_Init()` 位于 `App/Src/app_main.c`，当前按宏开关选择性初始化：

- 如果 `APP_ENABLE_LED_HEARTBEAT` 为 1，初始化 LED 抽象。
- 如果 `APP_ENABLE_ESP8266` 为 1，初始化 ESP8266 使能脚。
- 如果 `APP_ENABLE_UART3` 为 1，初始化 USART3。
- 如果 `APP_ENABLE_SERIAL_BRIDGE` 为 1，初始化 3-UART router。
- 如果 `APP_ENABLE_CS1238` 为 1，初始化 CS1238 采样任务。

`App_Loop()` 同样按宏开关执行：

- 运行串口路由任务 `AppSerialBridge_Process()`，如果该功能启用。
- 运行 CS1238 周期任务 `App_CS1238_Task()`，如果该功能启用。
- 使用 `HAL_GetTick()` 做非阻塞 LED 心跳。

## 4. 当前 CS1238 硬件分配

CS1238 句柄定义在 `Device/Src/dev_cs1238.c`：

```text
hcs1238_1:
  SCLK = PB0
  DOUT = PB1

hcs1238_2:
  SCLK = PA0
  DOUT = PA6
```

驱动初始化时会根据句柄自动打开对应 GPIO 端口时钟，把 SCLK 配置为推挽输出，
把 DOUT 配置为上拉输入。写配置寄存器时，DOUT 会临时切到开漏输出，写完后再切回输入。

## 5. CS1238 底层驱动

核心文件：

- `Device/Inc/dev_cs1238.h`
- `Device/Src/dev_cs1238.c`

驱动提供的主要接口：

```c
void CS1238_Init(CS1238_Handle *h);
void CS1238_InitAll(void);
uint8_t CS1238_IsDataReady(CS1238_Handle *h);
HAL_StatusTypeDef CS1238_ReadRaw(CS1238_Handle *h, int32_t *raw, uint32_t timeout_ms);
HAL_StatusTypeDef CS1238_WriteConfig(CS1238_Handle *h, uint8_t cfg);
uint8_t CS1238_MakeConfig(uint8_t refo_off, uint8_t speed, uint8_t pga, uint8_t ch);
uint8_t CS1238_PgaGainFromSel(uint8_t pga);
float CS1238_RawToVoltage(int32_t raw, float vref, uint8_t pga);
```

配置枚举已经在头文件中明确列出：

```c
CS1238_SPEED_10HZ
CS1238_SPEED_40HZ
CS1238_SPEED_640HZ
CS1238_SPEED_1280HZ

CS1238_PGA_SEL_1
CS1238_PGA_SEL_2
CS1238_PGA_SEL_64
CS1238_PGA_SEL_128

CS1238_CH_A
CS1238_CH_B
CS1238_CH_TEMPERATURE
CS1238_CH_SHORT
```

`CS1238_ReadRaw()` 读取 24 位原始转换值，并做符号扩展。`CS1238_WriteConfig()`
按 CS1238 的时序先等待 DRDY/DOUT 拉低，再读掉当前转换数据，之后发送写配置命令
`0x65` 和 8 位配置寄存器值。

## 6. CS1238 应用层采样任务

核心文件：

- `App/Inc/app_cs1238_task.h`
- `App/Src/app_cs1238_task.c`

当前应用层常量：

```c
#define APP_CS1238_PRINT_INTERVAL_MS        1000U
#define APP_CS1238_PRINT_SAMPLE_WINDOW_MS   1200U
#define APP_CS1238_ENABLE_ADC1              1
#define APP_CS1238_ENABLE_ADC2              1
#define APP_CS1238_POLL_ADC2                0
#define APP_CS1238_PRINT_ADC2               0
#define APP_CS1238_REFO_OFF                 0U
#define APP_CS1238_SPEED_SEL                CS1238_SPEED_10HZ
#define APP_CS1238_PGA_SEL                  CS1238_PGA_SEL_2
#define APP_CS1238_CHANNEL_COUNT            2U
#define APP_CS1238_ACTIVE_CHANNEL_COUNT     2U
```

当前行为：

- ADC1 和 ADC2 都会创建设备对象。
- ADC1 会被轮询和打印。
- ADC2 会初始化配置，但 `APP_CS1238_POLL_ADC2` 和 `APP_CS1238_PRINT_ADC2` 当前为 0，
  所以不会进入周期轮询和打印。
- 每个 ADC 设备内部维护 A/B 两个数据流。
- 采到一个通道后，任务切到下一个通道，并丢弃若干次转换，减少切换通道后的旧数据或过渡数据。
- 打印时只统计最近约 1200 ms 内的样本，避免长时间窗口掩盖当前状态。

当前 USART1 调试输出格式较精简：

```text
[CS1238] init speed=10Hz pga=2 cfgA=0x04 cfgB=0x05 ADC1=OK ADC2=OK
[CS1238] t=12345ms
ADC1-A raw_avg=...
ADC1-B raw_avg=...
```

## 7. 串口和 ESP8266 模块当前状态

项目仍保留 3-UART router：

- `App/Src/app_serial_bridge.c`
- `App/Inc/app_serial_bridge.h`

它支持三种模式：

```c
APP_ROUTE_PC_CAR
APP_ROUTE_WIFI_CAR
APP_ROUTE_PC_ESP_DEBUG
```

也保留了本地 `#` 命令：

```text
#pc_car
#wifi_car
#esp_debug
#esp_on
#esp_off
#status
```

但在当前 CS1238 bring-up 配置下：

```c
#define APP_ENABLE_SERIAL_BRIDGE 0
#define APP_ENABLE_ESP8266       0
#define APP_ENABLE_UART2         0
#define APP_ENABLE_UART3         0
```

所以这些功能不会在开机时启动。需要回到车控/ESP8266 联调时，再从
`App/Inc/app_config.h` 打开对应宏。

## 8. 构建方式

工程使用 CMake preset：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

构建输出位于：

```text
build/Debug/
```

常见输出包括：

```text
SourceCar_HAL.elf
SourceCar_HAL.hex
```

## 9. 后续扩展建议

- 如果继续调 CS1238，建议先保持串口路由和 ESP8266 关闭，避免额外中断和串口任务干扰采样时序。
- ADC1 稳定后，再把 `APP_CS1238_POLL_ADC2` 和 `APP_CS1238_PRINT_ADC2` 打开，对比双 ADC 行为。
- 如果要恢复车控串口路由，需要同步打开 `APP_ENABLE_SERIAL_BRIDGE`、`APP_ENABLE_UART2`，
  如果涉及 ESP8266，还需要打开 `APP_ENABLE_ESP8266` 和 `APP_ENABLE_UART3`。
- 公开仓库目前还没有明确 `LICENSE` 文件，后续建议按实际开源意图补充许可证。
