# STM32L051 电源管理 EC

面向电池供电 Linux 主机的嵌入式控制器（Embedded Controller）固件。使用
**STM32L051C8T6** 独立管理电源按键、负载开关、电池状态和低功耗休眠，
并通过 I2C 向主机提供电池数据与延迟断电接口。

## 背景

电池供电的主机需要在操作系统尚未启动、已经关机或负载供电关闭时，仍能响应
电源按键和外部电源变化。本项目将这部分控制放在独立 MCU 上：EC 决定负载是否
供电，Linux 主机读取电池信息，并可在系统关机前安排延迟断电。

当前实现围绕 BQ27220 电量计、BQ25601 充电器的 PG 信号和 TPS22992S 负载开关
构建，支持可选的 SSD1306 OLED。仓库主要包含固件和主机工具；硬件连接与
电池参数需要按实际电路适配。

## 主要功能

- **按键开关机**：按键去抖后长按 3 秒切换负载供电；上电默认关闭负载。
  长按关断直接控制负载开关，不等待 Linux 确认。
- **电池数据采集**：通过 I2C1 读取电压、瞬时／平均电流、SOC、剩余／满充容量、
  电池状态及预计充满／放空时间。每个字段独立标记有效性。
- **主机 I2C 接口**：I2C2 使用 7 位地址 `0x42`，提供 28 字节电池快照；
  单次连续读取使用同一份快照，中断不访问电量计。
- **延迟断电**：主机可请求 `0～65535` 秒后关闭 PA1 控制的负载开关；
  新的有效命令替换旧倒计时，按键提前关断会取消倒计时。
- **状态显示**：RGB LED 用颜色表示电量，用单闪／双闪区分电池供电与外部电源，
  用蓝色短脉冲提示负载运行。可选 OLED 提供概览与电气数据两页，短按切换。
- **低功耗与恢复**：负载关闭、无外部电源且空闲 10 秒后进入 STOP；
  按键或 PG 边沿唤醒后恢复时钟和 I2C，并处理未完成的主机事务。
- **诊断与降级**：串口输出启动、电池、状态转换和 I2C2 恢复日志。
  电量计启动失败时进入 `NO_GAUGE`，仍保留按键与负载控制，电池字段标记为无效。

## 硬件与引脚

目标 MCU 为 STM32L051C8T6（64 KB Flash、8 KB RAM）。运行时使用 HSI16，
AHB 分频 2，HCLK／PCLK 为 8 MHz；毫秒计时由 SysTick 中断维护。

| 接口 | MCU 引脚 | 用途 |
|---|---|---|
| 电源按键 | PA0 | 内部上拉，低电平按下；STOP 唤醒源 |
| 负载开关 | PA1 | TPS22992S ON，高电平开启负载，上电默认低 |
| RGB LED | PA4 / PA5 / PA6 | 红／绿／蓝，共阳，低电平点亮 |
| 充电器 PG | PC13 | BQ25601 外部电源有效信号，低有效；STOP 唤醒源 |
| I2C1 | PB6 SCL / PB7 SDA | BQ27220（`0x55`）及可选 OLED（`0x3C`） |
| I2C2 | PB13 SCL / PB14 SDA | 主机接口，AF5，从机地址 `0x42` |
| 调试串口 | PA2 TX | USART2，115200、8N1 |
| SWD | PA13 / PA14 | SWDIO / SWCLK |

I2C 地址均为未左移的 7 位地址。主机总线按 **100 kHz** 配置，需要支持
clock stretching；SCL／SDA 使用外部上拉到 3.3 V，主机与 EC 共地。
固件仅读取 BQ25601 的 PG 引脚，不通过 I2C 配置充电器。

## 仓库结构

```text
Inc/                  驱动、平台层、状态机与主机协议头文件
Src/                  固件实现、启动代码与系统支持
ThirdParty/u8g2/      u8g2 C 核心、SSD1306 驱动与精简字体
cmake/                Arm GCC 工具链、编译参数与链接配置
Tests/                主机协议、状态机、STOP 与 Python 工具测试
tools/                Linux 主机读取与延迟断电脚本
docs/i2c_host_protocol.md  接线、寄存器表、命令格式及故障诊断
```

## 构建与烧录

需要以下工具和依赖：

- CMake 3.21 或更新版本（以下命令使用仓库的 CMake Presets）。
- Ninja。
- GNU Arm Embedded 工具链，包含 `arm-none-eabi-gcc`、`g++`、`objcopy`
  以及 newlib／newlib-nano；本地验证使用 GCC 14.3.1。
- [STM32CubeL0 v1.12.4](https://github.com/STMicroelectronics/STM32CubeL0/tree/v1.12.4)
  中的 CMSIS 和 STM32L0 LL 驱动。

当前 CMake 从仓库的同级目录 `../STM32CubeL0` 引用驱动。目录应为：

```text
workspace/
├── ec/                 本仓库
└── STM32CubeL0/         外部依赖，需包含 Drivers/
```

若尚未准备依赖，可在本仓库根目录执行：

```sh
git clone --recursive --depth 1 --branch v1.12.4 \
  https://github.com/STMicroelectronics/STM32CubeL0.git ../STM32CubeL0
```

将 Arm 工具链加入 `PATH`，再构建 Release 固件：

```sh
cmake --preset Release -DEC_ENABLE_OLED=OFF
cmake --build --preset Release
```

OLED 默认关闭。启用 0.91 英寸、128×32 SSD1306 显示时重新配置：

```sh
cmake --preset Release -DEC_ENABLE_OLED=ON
cmake --build --preset Release
```

已有构建缓存会保留 OLED 选项，切换时需显式设置。Debug 构建可使用
`cmake --preset Debug -DEC_ENABLE_OLED=OFF` 和 `cmake --build --preset Debug`。

Release 产物位于 `build/Release/`：`ec.elf`、自动导出的 `ec.bin` 和 `ec.map`。
通过 ST-Link／SWD 使用 STM32CubeProgrammer 等工具烧录；若使用 BIN，Flash 起始
地址为 `0x08000000`。串口日志从 PA2 输出，可用于检查启动和接口状态。

## Linux 主机使用

在可访问 `/dev/i2c-*`、已安装 `smbus2` 的 Python 环境中运行：

```sh
python3 tools/read_ec.py --bus 1 --address 0x42
```

脚本以 JSON 输出一次完整快照，无效字段为 `null`，负电流表示放电。
总线编号按主机实际配置选择，脚本不设置总线速率。

请求 15 秒后断开负载供电：

```sh
python3 tools/poweroff_ec.py --bus 1 --address 0x42 --delay-seconds 15
```

如果 Linux 主机由该负载供电，需要为操作系统完成关机预留时间。
计时从固件处理写事务的 STOP 中断开始，到期后由主循环执行关断；
阻塞式 I2C／串口／显示操作可能使实际动作稍晚于期限。

完整寄存器布局、命令格式、测试命令与主机兼容性说明见
[I2C 主机协议文档](docs/i2c_host_protocol.md)。

## 验证与适配边界

`Tests/` 覆盖电池数据编码、有效位、快照一致性、I2C 事务边界、延迟断电、
计时回绕、按键并发、STOP 唤醒证据和异常事务恢复。测试运行方式见
[协议文档的验证章节](docs/i2c_host_protocol.md#验证)。
Python 工具测试可直接运行：

```sh
python3 -B -m unittest discover -s Tests -p 'test_host_tools.py' -v
```

适配其他硬件时，需要关注以下现有约束：

- **电量计配置与电池绑定**：`Src/bq27220.c` 内嵌 `gm.fs` 配置，当前以
  3300 mAh 设计容量作为配置检查条件。容量不匹配时启动流程会写入该配置；
  更换电池需要重新生成并核对配置，不能只改容量常量。
- **STOP 期间接口不可读**：I2C2 不支持 STOP 地址匹配唤醒；需要按键或 PG 唤醒。
  主机读取本身不会重置空闲计时。主机控制器还需与 clock stretching 兼容。
- **低电量目前只提示**：当前固件没有按低 SOC 自动关断负载的逻辑。
- **实板验证仍必要**：主机测试模拟寄存器和中断，不覆盖电气时序；
  ACK、上拉、时钟拉伸、PA1 关断和 STOP 功耗需要在实际电路上验证。

## 第三方依赖与开源许可

u8g2 核心采用 **BSD-2-Clause**，允许使用、修改和分发，包括商用；
不会要求本项目采用 GPL 或同一种许可证。分发源码时需保留版权、许可条款和
免责声明；分发固件二进制时也需在随附文档或材料中提供这些声明。
详见仓库中的 [u8g2 LICENSE](ThirdParty/u8g2/LICENSE) 和
[上游许可证](https://github.com/olikraus/u8g2/blob/master/LICENSE)。

字体与其他依赖需要分别处理，不能统一按 u8g2 核心许可证理解：

| 组件 | 来源／许可 | 当前情况 |
|---|---|---|
| u8g2 C 核心与 SSD1306 驱动 | u8g2 2.36.19，BSD-2-Clause | 已保留上游 LICENSE 与源码声明；版本、提交及裁剪方式见 [集成说明](ThirdParty/u8g2/README.md) |
| ASCII 字体与 9×15 字体子集 | 源字体标注 Public Domain | 来源与声明保留在字体源文件中 |
| 中文 `u8g2_font_ec_ui_12` | Efont Biwidth `b12.bdf` 子集；efont 的 BSD 三条款许可 | 已保留字体版权与生成说明，发布前需补齐完整许可条款及免责声明；见 [上游字体许可说明](https://github.com/olikraus/u8g2/wiki/fntgrpefont#copyright) |
| 外部 STM32L0 LL 驱动 | 本地 STM32CubeL0 组件 LICENSE 为 BSD-3-Clause | 分发所用源码或固件时需保留对应声明 |
| 外部 CMSIS Core／Device 及 ST 系统模板 | 本地 CMSIS 组件 LICENSE 为 Apache-2.0 | `Src/system_stm32l0xx.c` 与所用 Device 模板一致；仓库还需随附对应许可证 |

STM32CubeL0 内不同组件、版本的许可可能不同，应以实际使用组件的 LICENSE 和
源码声明为准。`Src/startup_stm32l0x1xx.S`、`syscall.c`、`sysmem.c` 等文件也带有
ST 版权声明；公开分发前需核对其生成模板来源并补齐对应许可。
Apache-2.0 组件还要求保留相关声明、标明修改，并在适用时随附上游 NOTICE；
具体条件见 [Apache-2.0 正文](https://www.apache.org/licenses/LICENSE-2.0)。

**项目原创代码采用 MIT License，详见根目录 [LICENSE](LICENSE)。**
允许使用、修改、分发和商用，需保留版权声明与许可正文。
MIT 授权适用于本项目原创部分；u8g2、字体、ST／CMSIS 代码及其派生文件继续
适用各自的许可证。发布时还需补齐上表中指出的第三方许可文件。
