# STM32F103C8T6 钢球平衡迁移版

适配求真物联网开发板 QZ_AIOT v2.0 白银扩展板。Keil 项目为 [F103_K230_Center_Balance.uvprojx](MDK-ARM/F103_K230_Center_Balance.uvprojx)。请克隆整个 `h_balance`。F103 和 F407 从同一份 `main.c`、`center_balance.c`、K230 协议、OLED 界面和 VOFA 控制代码编译。上电行程、各模式状态机、调参默认值和显示方向沿用当前主工程。MODE4 仍是预留模式。

F103 使用内部 HSI 加 PLL 得到 64 MHz，Flash 和 SRAM 分别按 C8T6 的 64 KB 和 20 KB 链接。USART2 与 USART1 分别接 K230 和板载 CH340，均为 115200 8N1。X42S 保留原来的 STEP 低有效、100 μs DIR 建立和 100 μs 脉冲，换向保持 2 ms。[ST 芯片规格](https://www.st.com/en/microcontrollers-microprocessors/stm32f1-series/stm32f103c8.html) · [ST 参考手册](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101-stm32f102-stm32f103-stm32f105-and-stm32f107-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

## 接线

跳线排位置按白银扩展板原理图的 P3/P4 编号和信号名核对。K230、主控、步进驱动和 IMU 共地。

| 功能 | 扩展板连接 | 对端 |
| --- | --- | --- |
| K230 → F103 | P4.12 `PA3`（USART2 RX） | K230 `IO9` TX |
| F103 → K230 | P4.13 `PA2`（USART2 TX） | K230 `IO10` RX |
| 步进脉冲 | P3.5 `PA8`（TIM1_CH1） | X42S `STP` |
| 步进方向 | P4.14 `PA1` | X42S `DIR` |
| VOFA+ | 扩展板 `USB1`，板载 CH340 | PC，内部连 `PA9` TX 和 `PA10` RX |
| SSD1306 OLED | `U7`，依次为 GND、3.3V、`PB8` SCL、`PB9` SDA | 原 OLED |
| ICM-45686 | P3.1～4，`PB12` CS、`PB13` SCK、`PB14` MISO、`PB15` MOSI | 原 IMU，3.3V 和 GND 接扩展板对应电源 |
| SWD 下载 | 最小系统板 SWDIO `PA13`、SWCLK `PA14`、GND | WCH-Link CMSIS-DAP 或 ST-LINK |

KEY1 `PA11` 选下一模式，KEY2 `PA15` 选上一模式，KEY3 `PB3` 确认。KEY4 `PB4` 启动 MODE5 正弦跟踪，KEY5 `PB5` 保留按键检测。固件使用内部上拉、低电平有效和原 20 ms 消抖。JTAG 释放给按键用，SWD 保留。[MODE5 操作与测试](../stm32f407_uart_test/README.md#mode5-正弦位置跟踪)

- USART2 共用 `U9` ESP8266 插座。接 K230 时取下 ESP8266。
- `PA8` 共用 SPI TFT 插座的 DC，`PA1` 共用 `H1` DHT11 信号。将这两个插座留空。
- IMU 的 SPI2 信号与 `U10` TB6612 插座和 `MK1` NRF24L01 的部分信号共用。将两个插座留空。X42S 继续使用自己的动力电源，接线按原驱动器版本。
- 原 ICM-45686 用于 OLED 相对倾角显示。若未安装 IMU，OLED 显示 `ANGLE IMU ERR`，钢球闭环仍使用 K230 的毫米位置与速度。

## 编译和下载

安装 Keil 的 `STM32F1xx_DFP 2.4.1` 和 Arm Compiler 6。用上面的 Keil 项目 **Rebuild** 后选择 **Flash → Download**。项目默认配置 CMSIS-DAP、SWD 100 kHz，也可以在 `Options for Target` 中改用 ST-LINK。使用 F103 工程生成的 HEX。

PowerShell 编译并更新发布 HEX。

```powershell
.\stm32f103c8_balance\build.ps1
```

上电自检会按原流程自动走 `0 → 650 → 310`。烧录复位前取出钢球，并让机构从机械下端起步。

VOFA 连接扩展板 USB1 枚举出的 COM 口。通道和参数见 [VOFA 手册](../stm32f407_uart_test/VOFA.md)。调参可以直接用板载 CH340。沿用 SWD 脚本时需要指定此工程生成的 Keil map。

## 验证

- Keil AC6.24 编译 0 错误、0 警告，Flash 24,900 B，静态 SRAM 含链接分配的栈和堆共 3,368 B。F407 正式目标重编译所得 HEX 与仓库原文件逐字节相同。
- F103 寄存器仿真通过 545 项检查，覆盖 64 MHz 时钟配置、按键、USART1/USART2、STEP 和换向间隔。相同简化机械模型下，F103 与 F407 输出一致；两版均有一段 MODE2 耗时 7.128 s，MODE3 有一侧保持误差 8.33 mm，超出题目要求。
- F103 实物尚未烧录。K230 收发、OLED、IMU、X42S 的现场电气连接和运动效果待测。

CMSIS Device F1 头文件与启动文件来自 [ST 固定提交](https://github.com/STMicroelectronics/cmsis-device-f1/tree/c8e9a4a4f16b6d2cb2a2083cbe5161025280fb22)，许可证随文件保留。新增的 `core_cm3.h` 来自 [Arm CMSIS 6.3.0](https://github.com/ARM-software/CMSIS_6/tree/v6.3.0/CMSIS/Core/Include)，其余 CMSIS Core 文件复用 F407 工程。
