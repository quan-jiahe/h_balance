# STM32F407VET6 ↔ 亚博 K230 通讯与单脉冲点动

最新支持 **USB转TTL + VOFA+在线调参**：PA9 TX、PA10 RX，115200/8N1，接收协议JustFloat。接线、通道与命令见 [VOFA.md](VOFA.md)。用户默认kp=0.5已保留，修改命令如 `kp=0.3` 加换行，复位后恢复源码默认值。

## 扩展板按键和 LED

| 器件 | F407 引脚 | 逻辑 |
| --- | --- | --- |
| SW1、SW2、SW3、SW4、SW5 | PD3、PD4、PD5、PD6、PD7 | 板上电阻上拉，按下为低电平 |
| LED3、LED4、LED5 | PD10、PD11、PA8 | 高电平点亮，上电默认熄灭 |

主循环已调用 `board_buttons_poll()`，按下与释放均消抖 20 ms。包含 `board.h` 后，可用 `board_button_is_pressed(BOARD_SW1)` 读取消抖后的状态，用 `board_button_take_press(BOARD_SW1)` 读取并清除一次按下事件。长按只产生一次事件，每个按键最多保留一个待读事件。LED 用 `board_led_set(BOARD_LED3, true)` 点亮，传 `false` 熄灭。LED1 和 LED2 是固定供电指示灯。

## 毫米位置/速度自动稳球版（2026-11-20）

工作目录统一为 `C:\Users\LENOVO\Desktop\ultra\h_balance`。打开 `MDK-ARM/F407_K230_UART.uvprojx`，选择 **F407_K230_Center_Balance** Target。固件为 `Firmware/F407_K230_Center_Balance.hex`，复制 HEX 时需同时保留对应编译版本的 map 供调参使用。

- 输入是 K230 的 18 字节版本 1 毫米观测：x_mm、vx_mm_s、有符号 int16、小端、CRC16。速度直接使用 K230 估计值，不在 F407 再差分或叠加滤波。像素版本 2 不参与控制，避免单位混用。
- 原像素控制的 DIR 标定值固定为 -1；毫米坐标左正右负，与原像素右正左负相反，因此电机输出另乘轴方向 -1。方向选择菜单和 DIR 文本已移除。此值按用户指定写入固件，并非本次实测电机方向。
- 保留原上电行程检查 0→650→310，需先手动置于机械下端并取出球。310 作为该次上电的显示/IMU 参考；进入稳球后不作为电机回位目标。20 s 未完成检查会进入故障。
- 自检完成自动进入 **S1 WAIT BALL**。收到新的有效球观测后自动进入 **S2 ZERO**，控制目标为管道 0 mm。
- |x|≤2 mm 且 |v|≤4 mm/s持续约0.3 s、且电机已追上目标时进入 **S3 HOLD ZERO**，把完整保持输出平滑转入新的参考姿态，停止发脉冲。偏差达到4 mm或速度达到10 mm/s时继续从学到的姿态纠偏，不强制返回310。310自检结束位置不代表实际水平。
- 丢球或180 ms无新观测时锁住实际步数，回到S1，但保留已经学到的姿态补偿；重获后重置运动规划并逐步恢复。人工停止也保留补偿。重复时间戳不能刷新看门狗。
- **返回**或运行中**确认**进入 **S4 STOPPED**，新球观测不会自行重启；确认可重新允许自动检测。**S5 FAULT** 表示脉冲异常或上电超时，第二行显示 `STEP PULSE FAIL` 或 `STARTUP TIMEOUT`，需排查后复位。
- 保留用户已调的每帧目标变化最多12步、4 ms脉冲间隔以及0～650步硬行程限制。球在远处没有有效回中（朝目标速度≤4 mm/s）持续0.3 s后，按ki缓慢学习姿态补偿，正常回中和制动时保留补偿，过零也不清除。输出饱和或指令计步落后时阻止同向积分，允许反向退出。`ki=0`仅冻结学习，不删除补偿。控制结构、模型验证及适用边界见 [CONTROL.md](CONTROL.md)，在线命令见 [VOFA.md](VOFA.md)。超过±100 mm仍按相同控制律纠偏，不触发S5。KFF/KFF2接口保留。
- 0.96 寸四针 OLED 默认使用已在实机点亮的 SSD1306 配置。接线为 SCL→PB8、SDA→PB9、VCC→3.3 V、GND 共地。模块丝印 `0x78` 对应 7 位地址 `0x3C`，也支持 `0x3D`。固件在上电约 100 ms 后开始探测，失败每 250 ms 重试。正式固件显示相对停靠姿态的 IMU 角度。其他已确认型号可在 Keil 的 C/C++ Define 中择一添加 `F407_OLED_SH1106` 或 `F407_OLED_SSD1309`。

### 编译和烧录

在 VSCode 的 PowerShell 中，从仓库根目录运行：

```powershell
.\tools\build_center.ps1
.\stm32f407_uart_test\flash_center.ps1
```

当前下载器为沁恒WCH-Link的CMSIS-DAP接口，原CubeProgrammer/ST-Link命令不适用。脚本会重编译AXF和HEX，再通过Keil下载、校验并复位；仅烧录时直接运行第二条命令即可。工程共享的 `MDK-ARM/F407_K230_UART.uvoptx` 保存了CMSIS-DAP、SWD 100 kHz和512 KB Flash算法配置。模块SWDIO接PA13、SWCLK接PA14、GND共地；RX/TX接PA9/PA10仅用于VOFA。调试接口错误码28需安装官方 `WCHLinkDrv_WHQL_S.exe`，步骤见 [VOFA.md](VOFA.md)。烧录会立即开始原上电行程，开始前把机构放到底端并取出球。

### VSCode 源码调参（当前采用）

在 `Core/Src/center_balance.c` 顶部修改具名 `g_balance_tuning` 默认值，重点先看 `.kp`、`.kv`、`.ki`。保存后从仓库根目录运行：

```powershell
.\tools\build_center.ps1
.\stm32f407_uart_test\flash_center.ps1
```

编译成功后再烧录，参数随固件永久保存。无需运行在线调参工具。`magic` 和 `version` 保持不变。

### ST-Link 在线调参（备用）

调参程序仅使用 Python 标准库、现有 CubeProgrammer 与 ST-Link，不需要额外串口。先烧录本版，再在仓库根目录运行：

```powershell
py .\tools\tune_balance.py --show
py .\tools\tune_balance.py --set kp=0.20 kv=0.80
```

使用 HOTPLUG 读取 RAM，不复位。读取当前 map 的参数符号，并先核对固件标记及版本，再写入单个32位浮点参数，最后读回验证。错误固件标记会拒绝写入。默认 map 是 `MDK-ARM/Listings/CenterBalance/F407_K230_Center_Balance.map`，需与烧录的 HEX 同次编译。多个 ST-Link 时应只保留目标这一个。

| 参数 | 默认值 | 含义与调整 |
| --- | --- | --- |
| kp | 0.5 | 位置增益，单位步/mm，过冲时适当减小 |
| kv | 0.70 | 速度误差增益，同时影响回中驱动力和速度抑制 |
| ki | 0.875 | 远处不回中及中心低速学习率，设0仅冻结学习 |
| gain | 1.50 | 动态纠偏和学习速率倍率，不重新缩放已学姿态 |
| brake | 1.60 | 提前制动时的速度增益倍率 |
| max_speed | 72 | 目标球速上限 mm/s |
| accel | 88 | 制动距离规划加速度 mm/s² |
| ref_accel | 120 | 目标速度上升斜率 mm/s² |
| kff / kff2 | 0 / 0 | 线性/平方速度补偿 |
| max_offset | 150 | 单次纠偏相对锁存姿态的最大脉冲偏移 |

在线修改只保存在 RAM，复位恢复默认。确定合适参数后修改 `Core/Src/center_balance.c` 的 `g_balance_tuning` 初始化值，重新编译、烧录，使其永久生效。方向 -1 是编译常量，在线工具不能修改。

本版 Keil AC6.24编译0错误0警告，153项Cortex-M检查、23项Python检查通过，包含毫米帧/速度、自动启动、固定方向、非310位置稳定保持、丢球、停止和两端限位。调参工具用模拟CLI验证了签名检查、地址和写后校验。本次尚未实际烧录，在线SWD调参及实机稳球需烧录后验证。

## 拍照联调用往返固件

Keil 的 **F407_K230_UART** 目标用于拍照联调。该固件上电即运行 0–650 步往返，启动时关闭 IMU 找平流程。上电前把连杆手动放到底板，第 0 步以该位置为准。650 步是实测机械上界，步数来自已发出的脉冲。首次试机取出钢珠，核对运动方向和行程。

F407 通过 PE9 输出 STEP、PE7 输出 DIR。到第 0 或 650 步后直接换向，端点没有额外停顿。两端保留 12 ms 的减速脉冲间隔，中段间隔在 4–8 ms 中按伪随机序列逐趟变化，相邻两趟速度不同。每约 250 ms 从 USART3 PD8 输出四行状态，含 `MOTOR:FWD`、`MOTOR:REV`、`MOTOR:STOP` 或 `MOTOR:ERR` 和 `STEP:+123`。拍照联调只需接 PD8 → K230 IO10 和 GND，`115200 / 8N1`。

K230 的 `capture_training_photos.py` 独立每约 0.5 秒保存一张照片。IDE 停止拍照后，F407 仍继续运行。停止电机可切断驱动供电，或在双向串口已接线且 K230 运行 `touch_ui.py` 对应的平衡界面时按“返回”。按“返回”后需复位 F407 才会重启往返。

联调固件为 `Firmware/F407_K230_Photo_Pulse_Cycle.hex`。本地编译、73 项 Cortex-M 检查和 14 项 K230 Python 检查通过。双板联机和实物运动待验证。

下面的自动找平和单脉冲点动说明对应各自的历史 HEX。

## 开机自检和自动找平

找平版使用历史固件 `Firmware/F407_K230_Startup_Level.hex`。K230 已部署的界面能显示新状态，更新 `k230/touch_ui.py` 后还会在扫描和回平时显示停止提示。现有主程序、串口帧格式和 YOLO 模型继续使用。

| 接线 | F407 |
| --- | --- |
| ICM-45686 SCLK / MISO / MOSI | PB13 / PB14 / PB15（SPI2） |
| ICM-45686 CS / INT1 | PB0（低有效）/ PB1（本版通过 SPI 查询数据就绪） |
| STEP / DIR | PE9 / PE7（已验证正向脉冲抬升电机侧） |

### 上电操作

1. 第一次试机先取出钢珠。每次上电前手动把连杆放到底板，再给整机上电。这个位置是脉冲计数零点。IMU 能拒绝接近水平的起始姿态，触底仍由实际摆放保证。
2. `HOME: CHECK LOW` 表示先静置，再取 40 组稳定的三轴加速度数据。每轴去除最大和最小样本后计算重力方向。`HOME: SWEEP UP` 表示开始正向扫描，主段每 24 步停稳测角，STEP 间隔 4 ms。实测机械上界是 650 个脉冲，程序扫描到 640 步，预留 10 步。
3. ICM 的 Z 轴沿钢珠运动方向，固件用三轴加速度的重力投影确定水平并记录过零步数。
4. `HOME: RETURN LEVEL` 表示从上限快速反向回到水平附近，再用较长的滤波窗口测角并单步微调。连续三组稳定读数在 ±0.10° 内后显示 `HOME: READY`，保持位置等待指令。
5. 自动运行期间 K230 **返回**可以中止。待命后 **前进/后退** 每次有效点击走一步并切入 `HOME: MANUAL`，仍受 0–640 步范围限制。

`HOME: FAULT` 时停止发 STEP。右侧下一行显示原因，常见有 `IMU SPI` / `IMU DATA`、`PLACE AT LOW`、`NO PROGRESS`、`NO LEVEL IN RANGE` 或 `LEVEL TIMEOUT`。排查后回到底板再重启。水平精度取决于 Z 轴和管道的对齐以及传感器零偏，首次试机可用水平尺独立复核。每次上电自检所用的水平步数只保存在 RAM。

加速滤波版通过 Keil 编译、42 项 Cortex-M 模拟检查及 13 项 K230 Python 检查。HEX 校验和、向量和 Flash 地址范围已复核。2026-11-18 Keil Load 再次报 `Target DLL has been cancelled`，本机 `EnableFlashSeq` 已关闭，板上读回仍为前一版。改用 STM32CubeProgrammer 的 100 kHz SWD 烧录，工具校验通过，两次独立读回也都与 10,288 字节的新版 HEX 一致。实机运动与找平精度待验证。

若换电脑后 Keil Load 报 `Target DLL has been cancelled`，留意前面的弹窗。旧版下载时实机复现的具体报错为 `Connection refused due to device mismatch`，CubeProgrammer 读到芯片 ID 为 `0x413`、512 KB Flash。修复设置为 **Options for Target → Debug → ST-Link Settings → Pack**，取消 **Debug Description → Enable**。本机对应 `.uvoptx` 的 `EnableFlashSeq` 为 `0`。在 **Flash Download** 使用 `STM32F4xx 512kB Flash`，选 **Erase Sectors、Program、Verify**。在 **Utilities** 选 **Use Target Driver for Flash Programming** 和 **Use Debug Driver**。设置后直接点 **Load**。

以下为独立点动测试固件的接线与操作记录。

这是完整源码的独立 Keil MDK 工程，用于测试双向串口与 X42S 单脉冲点动。仓库根目录的 MSPM0 工程负责原有平衡控制。

## 接线

| K230 | STM32F407VET6 | 方向 |
| --- | --- | --- |
| IO9 / UART1 TX | PD9 / USART3 RX / AF7 | K230 → F407 |
| IO10 / UART1 RX | PD8 / USART3 TX / AF7 | F407 → K230 |
| GND | GND | 必须共地 |

两块板各自正常供电，串口使用 3.3 V TTL。这里只连 TX、RX、GND，不将 K230 的 5V 脚接到信号脚。参数为 `115200 / 8N1`，无硬件流控。

PD8/PD9 的 USART3 AF7 映射依据：[ST 数据手册](https://www.st.com/resource/en/datasheet/stm32f407ve.pdf)。

| F407 J8 | X42S |
| --- | --- |
| PE9 / TIM1_CH1 / AF1 | Stp |
| PE7 / GPIO | Dir |
| GND | 驱动信号参考地 |

按实际驱动版本配置接线图中的 COM 和脉冲输入模式。提供的 HEX 按共阳接线配置，STEP 空闲高、有效低，在 DIR 更新 100 µs 后输出 100 µs 脉冲。若实物输入需要空闲低，在 `Core/Src/board.c` 将 `STEP_POLARITY` 改为 `0u` 后重新编译。前进时 PE7 为高电平。驱动器设置决定电机实际转向和每脉冲角度。定时器配置依据 [ST RM0090](https://www.st.com/resource/en/reference_manual/rm0090-stm32f407-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)。

## 编译和烧录

1. 单脉冲点动使用预编译的历史固件 `Firmware/F407_K230_Stepper_Jog.hex`。`Firmware/F407_K230_UART.hex` 是旧通讯测试备份。
2. 当前 Keil 的 **F407_K230_UART** 目标生成拍照往返固件，输出为 `MDK-ARM/Objects/F407_K230_UART.hex`。
3. 通过 ST-Link / SWD 烧录。可以直接用 STM32CubeProgrammer 或 ST-Link Utility 加载 HEX，核对芯片为 F407VE，Flash 起始地址 `0x08000000`，烧录后复位，BOOT0 为 0。
4. 若直接用 Keil 下载：在 Options for Target → Debug 选择 **ST-Link Debugger / SW**；在 Flash Download 添加适用于 STM32F407 的 **STM32F4xx 512KB Flash** 算法。若列表没有该芯片/算法，先通过 Pack Installer 安装 `Keil::STM32F4xx_DFP`。工程已包含编译所需的 CMSIS 头文件和启动文件，编译本身不需要在线下载 HAL。
5. 在 K230 上同时更新 `main.py`、`touch_ui.py` 和 `balance_ui_20.fnt`。首次部署的完整文件清单见 [K230 README](../k230/README.md)。串口接线后，等待主控菜单和步数出现。

Flash 配置为 **512 KB**，使用普通 SRAM **128 KB**。内部 HSI **16 MHz** 提供系统时钟，SysTick 提供 1 ms 计时。USART3 用 RXNE 中断和接收缓冲，主循环解析、发送。TIM1 采用单脉冲模式，每次计满由硬件结束。

## K230 上应看到什么

K230 顶部显示“主控菜单已同步”，四行主控菜单示例：

```text
F407 RX:OK N:123
OBS:120 LOST
KEY:UP V:1
步数 +0
```

| 项目 | 含义 |
| --- | --- |
| `RX:OK` / `WAIT` / `LOST` | F407 最近 1.5 秒收到正确协议帧 / 尚未收到 / 接收超时 |
| `N` | F407 收到的校验正确且类型受支持的包总数，包括观测和触控/同步请求 |
| `OBS` | 观测帧计数；当前识别与标定未完成，因此 `LOST` 正常 |
| `KEY` / `V` | F407 实际收到的最近一次导航键 / 通讯测试值 |
| 步数 | F407 确认 TIM1 发完脉冲后的净计数，以 F407 上电位置为零。后缀 `ERR` 表示发脉冲异常，点动禁用 |
| `E` / `C` | 发生 UART 或协议校验错误时在 RX 行显示累计错误计数 |

显示计数 N、OBS 取模 100000，E、C 取模 1000；内部使用 32 位计数。每 250 ms 回传一次菜单，触控或菜单同步请求也触发回包。

| 点击屏幕 | F407 处理 | K230 反馈 |
| --- | --- | --- |
| 上移 | 测试值 +1，最大 9999 | `KEY:UP V:...` |
| 下移 | 测试值 -1，最小 -9999 | `KEY:DOWN V:...` |
| 确认 | 切换 START/STOP 并发送命令 | 顶部观测状态切换，预览继续 |
| 返回 | 测试值清零 | `KEY:BACK V:0` |
| 前进 | PE7 高，PE9 发 1 个 STEP 脉冲 | 步数 +1 |
| 后退 | PE7 低，PE9 发 1 个 STEP 脉冲 | 步数 −1 |

这是通讯测试时的按钮定义；根目录 MSPM0 平衡固件的正式菜单定义见 `../k230/README.md`。

从安全中位上电，先点一次前进、再点一次后退，核对步数依次为 `+0 → +1 → +0`。按住按钮只在一次有效点击完成后发送一次，两个点动按钮只在收到配套状态时生效。测试两端时保持同一次 F407 上电基准，接近机械端点前停下，记录两侧的带符号步数。步数是指令脉冲的累计量，实际位置由现场观察确认。软件限位留待实测数据确定。

F407 启动时把当前机械位置作为步数 `+0`，启动代码不发 STEP 脉冲。只重启 K230 时，F407 继续保留累计步数。驱动器自身的上电设置待现场确认。

## 源码位置

- `Core/Src/main.c`：主循环、动作分发和菜单回传。
- `Core/Src/board.c`：HSI、USART3、SysTick 和 PE7/PE9 单脉冲输出。
- `Core/Src/link_test.c`：CRC、观测/触控解析、计数和 95 字节菜单封装。
- `Startup/`：ST 官方 Keil 启动文件。
- `Drivers/CMSIS/`：随工程提供的 ST 和 Arm 头文件及许可证；来源见 `THIRD_PARTY.md`。
- `Tests/`：使用真实 K230 Python 协议生成输入，编译实际 C 源码，在 Cortex-M 指令模拟器中执行，再用 K230 解码器核对输出。

## 验证范围

Keil 正式固件编译通过。Cortex-M 模拟器通过 25 项检查，覆盖 K230 真实编解码、正反向事件、脉冲接口调用次数、步数回包、故障和原有通讯回归。可选 PC 检查依赖 Python `unicorn` 包和本机 Keil：

```powershell
py -m pip install unicorn
py -B Tests/run_protocol_tests.py --keil "你的Keil路径\UV4\UV4.exe"
```

2026-11-18 实机部署记录

- 点动版首次 Keil Load 报 `Target DLL has been cancelled`。当时通过 STM32CubeProgrammer 写入 Flash 第 0 扇区，复位后读回的 4424 字节全部匹配。Keil 下载修复见本文开头。
- K230 的 `main.py`、`touch_ui.py` 和 `balance_ui_20.fnt` 已写入 SD 卡，读回 SHA-256 均匹配。原采集脚本已备份，原钢球模型校验一致。写入后 USB 调试口掉线，点动版开机画面和 F407 步数回包尚未确认。
- 现场还需核对 PE9 波形、电机方向和两端净步数。
