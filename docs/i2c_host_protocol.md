# EC I2C2 电池数据与延迟关断接口（协议 v1）

STM32L051C8T6 使用 PB13（SCL）和 PB14（SDA）的 AF5，作为 I2C2 从机。
I2C1（PB6/PB7）仍由 MCU 作为主机访问 BQ27220；I2C2 中断返回缓存或接收关断请求，
不访问电量计，也不直接切断负载。

## 接线和参数

- 默认 **7 位地址：`0x42`**，定义在 `Inc/ec_host_i2c.h`。
  总线上写／读地址字节分别是 `0x84`／`0x85`；主机 API 应传 `0x42`。
- 主机配置 **100 kHz**，需要支持 clock stretching。当前 HSI16/2 配置仅按 Standard mode 实现。
  HCLK、PCLK1 和 I2C2 内核时钟均为 **8 MHz**。ES0251 §2.12.3 要求在主机
  数据建立时间为 250 ns 时，I2C 内核时钟至少为 4 MHz；原 2.097 MHz 配置不能保证兼容。
- PB13 → 主机 SCL，PB14 → 主机 SDA，两端 GND 共地。
- SCL/SDA 使用外部上拉到主机 3.3 V；可从 4.7 kΩ 起步，按已有上拉、线长和电容确认。
  MCU 引脚为开漏，无内部上拉。主机已有上拉时不要重复叠加过强的上拉。
- MCU 运行时可读，包括负载开启或 PG 有效时。
  保留“负载关闭、无外部电源、空闲 10 秒进入 STOP”的现有逻辑。
  **I2C2 不支持 STOP 地址匹配唤醒**；STOP 前释放引脚，STOP 期间主机不能读取。
  按键或 PG 唤醒后恢复接口。若主机必须随时读取，需要另行调整休眠策略或增加唤醒信号。
  完成一次 I2C 读取本身不会重置 10 秒空闲计时；读事务结束后若已满足休眠条件，
  MCU 可以立即进入 STOP。因此负载关闭且无 PG 时，连续轮询不能保证接口一直在线。

## 读取过程

主机先写一个字节的寄存器偏移，再发起读事务，偏移随读取递增。
支持 repeated START，也支持写偏移后 STOP，再单独读。
推荐一次从 `0x00` 读取 **28 字节**：

```text
START → 0x42/W → 0x00 → repeated START → 0x42/R → 28 bytes → NACK → STOP
```

每次读地址匹配时冻结一份已发布的数据，整个连续读使用同一份快照。
不同读事务可能拿到不同采样轮次；需要一致数据时应一次读完整块。
所有多字节数值为 **小端**，电流为 16 位有符号补码。
电池数据寄存器只读，对这些偏移和未知偏移的额外写入数据均忽略。
`0x20` 是独立的延迟关断命令入口，格式见下文；命令不修改电量计。
越界读取返回 `0xFF`，持续越界读取不会绕回寄存器起点。

## 寄存器表

| 偏移 | 字节数 | 类型 | 内容 |
|---|---:|---|---|
| `0x00` | 1 | u8 | 协议版本，当前为 1 |
| `0x01` | 1 | u8 | 数据长度，当前为 28 |
| `0x02` | 2 | u16 | 数据有效位，见下表 |
| `0x04` | 1 | u8 | bit0：PG/VBUS 有效；bit1：负载开启；bit2：Gauge NORMAL；bit3：支持延迟关断；bit4：关断倒计时已启动；bit5–7 保留为 0 |
| `0x05` | 1 | u8 | 电池状态：0 未知；1 静置；2 充电；3 放电；4 已充满 |
| `0x06` | 2 | u16 | 电池电压，mV |
| `0x08` | 2 | i16 | 瞬时电池电流，mA，正值充电／负值放电 |
| `0x0A` | 2 | u16 | 电量百分比 SOC，0–100 |
| `0x0C` | 2 | u16 | 预计充满剩余时间，分钟 |
| `0x0E` | 2 | u16 | 预计放空剩余时间，分钟 |
| `0x10` | 2 | u16 | 剩余容量，mAh |
| `0x12` | 2 | u16 | 满充容量 FCC，mAh |
| `0x14` | 2 | i16 | 平均电池电流，mA |
| `0x16` | 2 | u16 | BQ27220 原始 BatteryStatus() |
| `0x18` | 4 | u32 | 采样轮次，每次运行期 Gauge 轮询完成后递增，包括读取失败的轮次 |

采样轮次从 MCU 启动时的 0 开始，32 位自然回绕；它不是时间戳。
采样保持约 1 秒的轮询调度，实际执行时间受阻塞式 I2C 和串口输出影响。
毫秒计时采用 SysTick 中断，读取电量计、串口输出和 OLED 操作期间仍正常计时。
主机可以低频轮询，例如每秒读一次。若多次读取轮次不变，说明还没有完成新采样。
开机 Gauge BOOT 期间和 NO_GAUGE 模式下，接口仍可响应，但电池字段无效。

### 有效位（`0x02`）

| bit | 含义 |
|---:|---|
| 0 | 电压有效 |
| 1 | 瞬时电流有效 |
| 2 | SOC 有效 |
| 3 | 预计充满时间有效 |
| 4 | 预计放空时间有效 |
| 5 | 剩余容量有效 |
| 6 | 满充容量有效 |
| 7 | 平均电流有效 |
| 8 | BatteryStatus 有效 |
| 9–15 | 保留，为 0 |

主机必须先检查有效位。每个字段独立处理读取失败：无效的无符号 16 位字段返回 `0xFFFF`，
无效电流返回 0；**无效电流的 0 不代表实际没有电流**。
TimeToFull()/TimeToEmpty() 返回 `0xFFFF` 时也清除相应有效位，不显示成 65535 分钟。
有效的时间值 0 表示电量计报告的 0 分钟，与“未知”不同。

时间直接来自 BQ27220 `0x18`／`0x16`，是随充放电条件变化的估计值。
TimeToFull 包含电量计对充电末端电流收敛的估计，无需 MCU 用 SOC 简单推算。
该时间对应电量计估计的满充／放空，**不是 EC 低电量断电阈值的倒计时**。

电池状态由电池电流和有效的 BatteryStatus FC 位共同判断：

1. Gauge 非 NORMAL 或瞬时电流读取失败 → 未知。
2. 电流为负 → 放电（即使 FC 尚未清除）。
3. 有效的 FC（bit9）置位 → 已充满。
4. 电流为正 → 充电。
5. 电流为 0 → 静置。

PG 只表示输入电源有效。外部电源存在而负载消耗大于充电电流时，电池仍可能放电。
这不是 BQ25601 充电器的 Charge Done 状态；固件继续不通过 I2C 访问 BQ25601。

## 主机写入延迟关断

关断对象为 **PA1 控制的 TPS22992S 负载开关**。主机在一次独立写事务中发送恰好 4 字节：

```text
START → 0x42/W → 0x20 → 0xA5 → delay_s_lo → delay_s_hi → STOP
```

| 字节 | 内容 |
|---:|---|
| 0 | 命令入口 `0x20` |
| 1 | 关断命令 `0xA5` |
| 2 | 延迟秒数低 8 位 |
| 3 | 延迟秒数高 8 位 |

延迟为 **16 位无符号小端整数，单位秒，范围 `0～65535`**。
例如 `20 A5 05 00` 延迟 5 秒；`20 A5 2C 01` 延迟 300 秒；
`20 A5 00 00` 在写事务结束、主循环处理请求时立即关断。
最大延迟 65535 秒，即 18 小时 12 分 15 秒。

- 从 STM32 处理该写事务的 **STOP 中断**起计时；到期后主循环将 PA1 拉低，
  状态切换为 ACTIVE_IDLE。阻塞式电量计／串口／显示操作的耗时计入倒计时；
  主循环处理和 I/O 耗时可能使实际动作稍晚于期限，接口不保证硬实时响应。
- 只有上述完整、无错误、以 STOP 结束的帧才产生请求。
  错误命令、缺少字节、多余字节、总线错误，以及用 repeated START 结束的命令均忽略。
  不附加 SMBus 长度字节或 PEC；读取 `0x20` 返回 `0xFF`。
- 后续有效命令替换旧倒计时，并从新命令的 STOP 重新计时。
  无效写入不取消已生效的倒计时。按键长按提前关断会清除倒计时。
- 负载已关闭时，关断请求保持关闭并清除旧倒计时，不影响下一次按键开机。
  与命令同时发生的长按不会立即重新开机；仍按住的按键需要释放后重新长按。
- 主循环接受倒计时后，`0x04` 的 bit4 置位；关断或按键取消后清零。
  状态发布存在主循环处理延迟；I2C 字节 ACK 本身不代表倒计时已启动。
  bit3 用于区分新固件与只支持读取的旧固件，28 字节读布局和版本号保持为 1。
- 关断后，无外部电源且无按键活动时，重新等待空闲 10 秒再进入 STOP。

## STOP 前后与异常事务恢复

- 正常读取以 NACK／STOP 结束时关闭 TX 中断并清空 TXDR；下一次写入从新的偏移字节开始。
  在同一中断中出现旧事务 STOP 和新事务 ADDR 时，先清理旧事务，再接收新事务。
  已收到的完整关断请求不会被随后的一次读取清除。
- STOP 前同时检查 BUSY 和尚未处理的 ADDR／RXNE／TXIS／NACK／STOP／错误事件。
  硬件收到总线 STOP 就会清除 BUSY；即使 BUSY 已清零，只要结束中断还没处理，
  就暂缓休眠，防止丢掉最后一个字节或关断请求。
- 最后一次唤醒条件检查到 WFI 之间屏蔽中断执行，检查按键、PG、软件唤醒标志和 EXTI PR。
  新到来的中断保持挂起并让 WFI 返回；恢复原中断屏蔽状态后处理。
  采集唤醒证据时同时读取软件标志和硬件 EXTI PR，避免遗漏尚未进入 ISR 的边沿。
- 若 PA1 关断时主机正在读取，主机失去供电可能来不及发 STOP，BUSY 因此保持置位。
  主循环在所有运行状态检查 **BUSY 或 SCL 为低**。从首次观察异常开始计时，连续
  **1 秒**没有中断处理进展、没有待处理中断或完整命令时，关闭并复位 I2C2，
  放弃未完成的事务并保留缓存。恢复不会改变 PA1、负载状态或已应用的关断倒计时。
  这样负载运行、PG 有效以及唤醒后 BUSY 已清零但 SCL 仍低时，也有恢复机会。
  正常中断进展会重新计时；新 START 不会因为上次中断发生很久以前而被立即复位。
  字节之间暂停超过 1 秒的主机事务可能被中止，主机应重新从写偏移开始读取。
  若外部仍持续拉低 SCL，同一次故障只尝试一次复位，直到总线释放或出现中断进展。
- 进入 STOP 前关闭 I2C1／I2C2（PE=0），释放 PB13/PB14。唤醒后先恢复 8 MHz 时钟，
  再恢复 I2C2，随后恢复可能阻塞的 I2C1。I2C2 在软件状态和 NVIC 就绪后才设置 PE，
  不会在开始应答后清除新的中断挂起。寄存器指针归零、残留事务状态清除，
  已发布的电池快照保留。每次读取仍应先写寄存器偏移。
  错误恢复时 PE 保持关闭至少 3 个 APB 时钟周期，再重新使能。

上述总线状态和低功耗约束见 [ST RM0377 §23.3／§23.7.7](https://www.st.com/resource/en/reference_manual/rm0377-ultralowpower-stm32l0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)，
PE 处理见 [ST ES0251 §2.12.2](https://www.st.com/resource/en/errata_sheet/es0251-stm32l05xxxl06xxx-device-errata-stmicroelectronics.pdf)，
PRIMASK 下的 WFI 唤醒行为见 [Arm CMSIS 的 WFI 说明](https://arm-software.github.io/CMSIS_6/v6.0.0/Core/group__intrinsic__CPU__gr.html)。

## Linux 主机示例

使用 `tools/read_ec.py`，在已启用 I2C 且可访问 `/dev/i2c-1` 的 Linux 主机上运行：

```sh
python3 -m pip install smbus2
python3 tools/read_ec.py --bus 1 --address 0x42
```

脚本一次读取完整快照，输出 JSON；无效字段输出 `null`，负电流保持为负数。
总线速率需在主机 I2C 控制器配置中设置为 100 kHz，脚本不修改总线速率。
核心读取操作使用 `i2c_rdwr`：

```python
from smbus2 import SMBus, i2c_msg

with SMBus(1) as bus:
    offset = i2c_msg.write(0x42, [0x00])
    response = i2c_msg.read(0x42, 28)
    bus.i2c_rdwr(offset, response)
    data = bytes(response)
```

普通 MCU 主机同样可用“写偏移 + repeated START + 连续读”实现。
不要添加 SMBus block read 的长度字节或 PEC，协议只包含表中的 28 个数据字节。

延迟关断脚本先读取 bit3 确认固件支持，再发送独立写事务。以下命令会实际断开负载供电：

```sh
python3 tools/poweroff_ec.py --bus 1 --address 0x42 --delay-seconds 5
```

如果主机由该负载供电，应选取足够时间让操作系统完成关机。核心写操作如下：

```python
from smbus2 import SMBus, i2c_msg
import struct

delay_seconds = 5
with SMBus(1) as bus:
    payload = struct.pack("<BBH", 0x20, 0xA5, delay_seconds)
    bus.i2c_rdwr(i2c_msg.write(0x42, payload))  # 单独写入，以 STOP 结束
```

`tools/read_ec.py` 的 JSON 增加 `power_off_supported` 和 `power_off_pending` 字段。

## 验证

协议测试在主机上模拟 LL 标志、地址匹配和中断：

```sh
cc -std=c11 -Wall -Wextra -Werror -IInc \
  -include Tests/stubs/stm32l0xx_conf.h \
  Tests/test_host_i2c.c Src/ec_host_i2c.c -o /tmp/ec-test-host-i2c
/tmp/ec-test-host-i2c
cc -std=c11 -Wall -Wextra -Werror -IInc -DEC_ENABLE_OLED=0 \
  -include Tests/stubs/stm32l0xx_conf.h \
  Tests/test_ec_telemetry.c Src/ec_state_machine.c Src/ec_host_i2c.c \
  -o /tmp/ec-test-telemetry
/tmp/ec-test-telemetry
cc -std=c11 -O1 -Wall -Wextra -Werror -DSTM32L051xx -IInc \
  -isystem ../STM32CubeL0/Drivers/CMSIS/Core/Include \
  -isystem ../STM32CubeL0/Drivers/CMSIS/Device/ST/STM32L0xx/Include \
  -isystem ../STM32CubeL0/Drivers/STM32L0xx_HAL_Driver/Inc \
  Tests/test_platform_stop.c -o /tmp/ec-test-platform-stop
/tmp/ec-test-platform-stop
python3 -B -m unittest discover -s Tests -p 'test_host_tools.py' -v
```

测试覆盖负电流、小端序、无效数据、FC bit9、完整快照、采样期间更新、
repeated START、RXNE 与 STOP/ADDR 同时出现、忽略额外写入、越界、NACK、错误后新事务、
STOP 忙碌检查及恢复、唤醒后先恢复时钟再打印日志，以及 CR1/OAR1/TIMINGR 初始化值。
关断测试覆盖精确写帧、STOP 后提交、非法帧及总线错误、重复命令替换、0／65535 秒、
32 位计时回绕、I/O 阻塞期间到期、并发按键、手动关断清除倒计时、已关闭时幂等、
关断后空闲 10 秒、慢速 BOOT 后的空闲计时，以及主机脚本独立写事务和能力位检查。
STOP 平台测试使用真实 CMSIS 寄存器类型和 ST LL 函数，在 RAM 中模拟外设寄存器，
验证实际平台代码的 PRIMASK、WFI 窗口、短脉冲、硬件／软件唤醒证据、SysTick 停用和恢复。
增加 BUSY 已清零但 STOP/RXNE 未处理、地址关闭窗口中的结束事件、断电时主机丢失 STOP、
读取后写关断、反复 3 次关断／STOP／恢复／开机，以及恢复后毫秒计数归零的回归测试。
恢复测试还覆盖负载运行／PG 有效时恢复、BUSY 清零但 SCL 低、外部持续拉低时避免重复复位、
新 START 的完整超时窗口，以及恢复后继续读取和写入关断命令。
这些测试不模拟电气时序；烧录后仍需用实际主机验证 ACK、
clock stretching、读数随充放电变化、延迟结束时 PA1 实际拉低，以及 STOP／唤醒后的响应。

### 实板无 ACK 的核对方法

运行期使用 HSI16 作为 SYSCLK，AHB 分频 2，APB1/APB2 不分频。
SysTick 和 USART2（115200 8N1）的参数按 8 MHz 设置。
STOP 继续使用 MSI 唤醒，返回后先恢复 HSI16/2，再恢复 I2C 和输出日志。
进入 STOP 前关闭 SysTick 中断并清除挂起的 tick，避免周期中断造成立即唤醒。
运行时功耗会高于原 2.097 MHz 配置；需要实测新的功耗。

新版在平台初始化完成及 STOP 恢复后输出 `[I2C2]` 三行诊断日志，
增加 PRIMASK、NVIC 使能／挂起和 APB1 时钟使能状态。
在主机没有正在读取、MCU 没有进入 STOP 时，应检查：

| 寄存器／条件 | 地址／掩码 | 预期值 |
|---|---|---|
| I2C2_CR1 | `0x40005800` | `0x000000BD`：PE、RXIE、ADDRIE、NACKIE、STOPIE、ERRIE 开启 |
| I2C2_OAR1 | `0x40005808` | `0x00008084`：OA1EN 开启，7 位地址 `0x42` |
| I2C2_TIMINGR | `0x40005810` | `0x00303F3F` |
| GPIOB_MODER | `0x50000400 & 0x3C000000` | `0x28000000`：PB13/PB14 为 AF |
| GPIOB_OTYPER | `0x50000404 & 0x00006000` | `0x00006000`：两脚开漏 |
| GPIOB_PUPDR | `0x5000040C & 0x3C000000` | 0：无内部上拉 |
| GPIOB_AFRH | `0x50000424 & 0x0FF00000` | `0x05500000`：两脚 AF5 |
| GPIOB_IDR | `0x50000410 & 0x00006000` | 主机 3.3 V 上拉有效且总线空闲时为 `0x00006000` |

读事务中 TXIE 临时开启，CR1 可为 `0xBF`；STOP 中外设和地址应答被关闭，不能用上述值判断失败。
确认寄存器启用后仍无 ACK，需要在 PB13/PB14 实际焊盘观察 SCL/SDA、地址字节和第九个时钟。
仅降低 SCL 频率不保证增加数据建立时间，软件主机也应明确设置 SDA 后延迟，再拉高 SCL。
本次时钟修改消除了已知配置缺口，不能替代无 ACK 根因的实板验证。

### SCL 持续为低时的恢复诊断

本次固件位于 `build/Release/ec.elf` 和 `build/Release/ec.bin`；
CMake 现在每次链接后自动导出 BIN，避免 ELF 已更新但 BIN 仍为旧版本。
下列恢复日志需要重新烧录本次固件才能看到。

一次超时恢复会输出：

```text
[I2C2] RECOVERY CR1=0x000000BD ISR=0x00000001 IRQ_EN=1 pins-released SCL=0 SDA=1
```

这行先记录复位前的状态，再记录 **PE=0、PB13/PB14 切为无上拉输入**时的电平，
随后恢复 AF5 和从机接口。测量期间 STM32 不驱动这两条线，也不会生成 SCL 脉冲。
`pins-released SCL=0` 表示释放 STM32 引脚后总线依然为低，需要检查主机是否释放 SCL、
上拉电源和线路；单靠复位 STM32 无法保证恢复。`SCL=1` 表示这次释放窗口中线路已回高，
主机可重试，但仍需用实际波形确认最初是谁拉低。

本次故障的只读实板采样：固件与修改前 `build/Release/ec.bin` 一致，
`CR1=0xBD`、`OAR1=0x8084`、`ISR=0x01`、NVIC ISER=`0x01000000`，
PB13 为低、PB14 为高，串口周期状态日志正常。没有观察到 ADDR/TXIS/BUSY 挂起；
这些采样不足以认定 MCU 的地址或发送中断卡住，也不能直接确定低电平的驱动方。

### 树莓派硬件 I2C 的兼容性

若主机使用 BCM2835 系列 I2C 控制器，Linux 驱动标记了 `I2C_AQ_NO_CLK_STRETCH`，
而本从机通过 clock stretching 等待地址处理和发送字节。
因此需要检查主机驱动和波形，避免把所有失败归因于 STM32 STOP 恢复。
依据：[树莓派 i2c-bcm2835 驱动](https://github.com/raspberrypi/linux/blob/rpi-6.18.y/drivers/i2c/busses/i2c-bcm2835.c)。

可用官方 `i2c-gpio` 软件控制器做对比测试。若原连接确实使用 GPIO2=SDA、GPIO3=SCL，
并且没有其他设备需要硬件 I2C1，可在主机启动配置中关闭原硬件 I2C，复用原引脚：

```ini
dtparam=i2c_arm=off
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=2,i2c_gpio_scl=3,i2c_gpio_delay_us=5
```

先移除或修改原来使能同一硬件总线的配置，避免两个控制器同时占用 GPIO2/3。
Raspberry Pi OS 当前配置路径为 `/boot/firmware/config.txt`，修改后重启生效。
参数依据：[官方 overlay README](https://github.com/raspberrypi/firmware/blob/master/boot/overlays/README)，
路径依据：[树莓派 config.txt 文档](https://www.raspberrypi.com/documentation/computers/config_txt.html)。
软件总线仍需要有效的 3.3 V 上拉，也不能在 STM32 STOP 期间唤醒 I2C2。
重启后先用 `i2cdetect -l` 确认总线枚举，再在 MCU 已唤醒时读取：

```bash
python3 tools/read_ec.py --bus 3
```

硬件依据：[STM32L051 数据手册](https://www.st.com/resource/en/datasheet/stm32l051c8.pdf)
（表 11、表 17）、[ST ES0251 勘误](https://www.st.com/resource/en/errata_sheet/es0251-stm32l05xxxl06xxx-device-errata-stmicroelectronics.pdf)
（§2.12.3）、[BQ27220 TRM](https://www.ti.com/lit/ug/sluubd4a/sluubd4a.pdf)
（表 2-6、§2.11／2.12）、[smbus2 文档](https://smbus2.readthedocs.io/en/latest/)。
