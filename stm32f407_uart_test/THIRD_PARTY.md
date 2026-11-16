# 随工程提供的第三方文件

| 文件 | 来源 | 版本 | 许可证 |
| --- | --- | --- | --- |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/`、`Startup/startup_stm32f407xx.s` | [STMicroelectronics/cmsis-device-f4](https://github.com/STMicroelectronics/cmsis-device-f4/tree/9192c7b9df75a142f2027ab266601fe061fc00b3) | 获取时 master 对应 `9192c7b9df75a142f2027ab266601fe061fc00b3` | Apache-2.0，见 `Drivers/CMSIS/Device/ST/STM32F4xx/LICENSE.md` |
| `Drivers/CMSIS/Core/Include/` | [Arm CMSIS](https://github.com/ARM-software/CMSIS_6/tree/v6.3.0/CMSIS/Core/Include) | ARM.CMSIS 6.3.0 | Apache-2.0，见 `Drivers/CMSIS/Core/LICENSE` |

以上文件保留原始版权声明。ST 文件在首次下载后与所列固定提交的文件逐一校验；Arm 文件来自本机已安装的官方 CMSIS 6.3.0 包。只带入 Cortex-M4 / Arm Compiler 6 所需的 CMSIS 文件；不包含 HAL、RTOS、Keil 编译器或设备包的安装文件。
