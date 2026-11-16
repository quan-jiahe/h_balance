# STM32F407VET6 ↔ 亚博 K230 通讯测试

这是完整源码的独立 Keil MDK 工程，用于先验证双向串口、K230 屏幕反馈和触控事件。原 MSPM0 平衡控制工程保留在仓库根目录，本工程没有移植 PID、舵机或电机控制。

## 接线

| K230 | STM32F407VET6 | 方向 |
| --- | --- | --- |
| IO9 / UART1 TX | PD9 / USART3 RX / AF7 | K230 → F407 |
| IO10 / UART1 RX | PD8 / USART3 TX / AF7 | F407 → K230 |
| GND | GND | 必须共地 |

两块板各自正常供电，串口使用 3.3 V TTL。这里只连 TX、RX、GND，不将 K230 的 5V 脚接到信号脚。参数为 `115200 / 8N1`，无硬件流控。

PD8/PD9 的 USART3 AF7 映射依据：[ST 数据手册](https://www.st.com/resource/en/datasheet/stm32f407ve.pdf)。

## 编译和烧录

1. 用 Keil MDK 5 打开 `MDK-ARM/F407_K230_UART.uvprojx`，选择 **Arm Compiler 6**。本机已用 MDK 5.43 / AC6 6.24 完整编译，0 错误、0 警告。
2. 按 F7 编译，输出为 `MDK-ARM/Objects/F407_K230_UART.hex`。`Firmware/F407_K230_UART.hex` 是随工程提供的已编译版本；修改代码后请烧录新编译的 Objects 版本。
3. 通过 ST-Link / SWD 烧录。可以直接用 STM32CubeProgrammer 或 ST-Link Utility 加载 HEX，核对芯片为 F407VE，Flash 起始地址 `0x08000000`，烧录后复位，BOOT0 为 0。
4. 若直接用 Keil 下载：在 Options for Target → Debug 选择 **ST-Link Debugger / SW**；在 Flash Download 添加适用于 STM32F407 的 **STM32F4xx 512KB Flash** 算法。若列表没有该芯片/算法，先通过 Pack Installer 安装 `Keil::STM32F4xx_DFP`。工程已包含编译所需的 CMSIS 头文件和启动文件，编译本身不需要在线下载 HAL。
5. K230 继续运行仓库现有的 `k230/main.py`、`touch_ui.py` 和字体即可，不需要替换原生系统。串口接线后，等待约 1 秒应收到测试菜单。

Flash 配置为 **512 KB**，使用普通 SRAM **128 KB**（未使用独立 64 KB CCM）。时钟为内部 HSI **16 MHz**，不依赖开发板外部晶振，不使用 Bootloader 偏移。USART3 用 RXNE 中断和接收缓冲，主循环解析、发送，SysTick 提供 1 ms 计时。只配置 PD8、PD9 和系统计时，不初始化电机或舵机。

## K230 上应看到什么

K230 顶部显示“主控菜单已同步”，四行主控菜单示例：

```text
F407 RX:OK N:123
OBS:120 LOST
KEY:UP V:1
RUN:ON E:0 C:0
```

| 项目 | 含义 |
| --- | --- |
| `RX:OK` / `WAIT` / `LOST` | F407 最近 1.5 秒收到正确协议帧 / 尚未收到 / 接收超时 |
| `N` | F407 收到的校验正确且类型受支持的包总数，包括观测和触控/同步请求 |
| `OBS` | 观测帧计数；当前识别与标定未完成，因此 `LOST` 正常 |
| `KEY` / `V` | F407 实际收到的最近一次按键 / 通讯测试值，并非小球坐标 |
| `RUN` | F407 请求 K230 开始/暂停视觉观测的状态 |
| `E` / `C` | UART 硬件/接收缓冲错误计数 / 协议 CRC 错误计数 |

显示计数 N、OBS 取模 100000，E、C 取模 1000；内部使用 32 位计数。每 250 ms 回传一次菜单，触控或菜单同步请求也触发回包。

| 点击屏幕 | F407 处理 | K230 反馈 |
| --- | --- | --- |
| 上移 | 测试值 +1，最大 9999 | `KEY:UP V:...` |
| 下移 | 测试值 -1，最小 -9999 | `KEY:DOWN V:...` |
| 确认 | 切换 START/STOP 并发送命令 | `RUN:ON/OFF`；顶部观测状态也随之切换 |
| 返回 | 测试值清零 | `KEY:BACK V:0` |

这是通讯测试时的按钮定义；根目录 MSPM0 平衡固件的正式菜单定义见 `../k230/README.md`。

双向通过的判断是：**RX:OK、N 持续增加，并且点按钮后 KEY/V 跟着变化**。顶部“主控菜单已同步”只证明 K230 收到了 F407 的菜单；若 K230→F407 断线，仍可能收到菜单，但会显示 `RX:LOST`。若 F407→K230 断线，K230 约 3 秒后显示“未收到主控回包”。

## 源码位置

- `Core/Src/main.c`：主循环、菜单刷新与 START/STOP 回传。
- `Core/Src/board.c`：HSI 时钟、USART3/PD8/PD9、中断接收和 SysTick。
- `Core/Src/link_test.c`：CRC、观测/触控解析、测试值和 95 字节菜单封装，不依赖具体 MCU。
- `Startup/`：ST 官方 Keil 启动文件。
- `Drivers/CMSIS/`：随工程提供的 ST 和 Arm 头文件及许可证；来源见 `THIRD_PARTY.md`。
- `Tests/`：使用真实 K230 Python 协议生成输入，编译实际 C 源码，在 Cortex-M 指令模拟器中执行，再用 K230 解码器核对输出。

## 验证范围

已完成 Keil 正式固件编译、15 项 C 协议检查、真实 K230 编解码互通的软件检查：覆盖 CRC、噪声、错误长度、截断/超时恢复、按键限幅、计时回绕、接收超时、START/STOP 与菜单回包。可选 PC 检查依赖 Python `unicorn` 包和本机 Keil，正常固件编译不依赖它们：

```powershell
py -m pip install unicorn
py -B Tests/run_protocol_tests.py --keil "你的Keil路径\UV4\UV4.exe"
```

当前电脑未检测到 ST-Link，因此尚未烧录 F407，也尚未实测两块板的连线、触控和断线反馈。请先完成以上通讯测试，再接入平衡控制、识别和标定。
