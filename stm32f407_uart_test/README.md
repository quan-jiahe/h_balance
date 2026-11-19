# STM32F407VET6 ↔ 亚博 K230 通讯与单脉冲点动

## 开机自检和自动找平

找平版使用 `Firmware/F407_K230_Startup_Level.hex`，也可以打开下文的 Keil 工程重新编译并 Load。K230 已部署的界面能显示新状态，更新 `k230/touch_ui.py` 后还会在扫描和回平时显示停止提示。现有主程序、串口帧格式和 YOLO 模型继续使用。

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

1. 用 Keil MDK 5 打开 `MDK-ARM/F407_K230_UART.uvprojx`，选择 **Arm Compiler 6**。本机已用 MDK 5.43 / AC6 6.24 完整编译，0 错误、0 警告。
2. 直接烧录预编译的 `Firmware/F407_K230_Stepper_Jog.hex`。若按 F7 从源码重新编译，输出为 `MDK-ARM/Objects/F407_K230_UART.hex`。`Firmware/F407_K230_UART.hex` 是旧通讯测试备份。
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
