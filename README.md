# 当前联调入口

最新 [VOFA+调参说明](stm32f407_uart_test/VOFA.md)：USB转TTL接PA9/PA10，115200，JustFloat。

统一工作目录为 `C:\Users\LENOVO\Desktop\ultra\h_balance`。最新毫米稳球的编译、烧录与在线调参见 [F407说明](stm32f407_uart_test/README.md)。Keil选择 `F407_K230_Center_Balance`。固定DIR=-1，自检后S1自动检测，球稳定后锁住当时电机位置。旧版本说明保留在后面供参考。

# h_balance

钢球平衡系统：亚博带触摸屏 K230 负责视觉接口和触控菜单，通过串口与主控通信。

- K230 部署、串口协议和待完成项：[k230/README.md](k230/README.md)。
- 钢球 Model2 的下载、运行库依赖与标定说明：[k230/MODEL.md](k230/MODEL.md)。
- STM32F407VET6 已接入 ICM-45686。连杆触底上电后，固件静止采样并滤波，驱动 X42S 扫描到 640 步，再回到水平附近微调。加速滤波版已烧录并读回核对，实机运动与找平精度仍待验证。[F407 接线、固件与试机说明](stm32f407_uart_test/README.md)
- 两台电脑上的 Codex 接手前先阅读 [AGENTS.md](AGENTS.md)。
- 当前钢球平衡控制 C 工程为 TI MSPM0 工程。K230 已接入钢球 Model2，并在现场显示 F407 自检菜单。毫米标定、真实场景识别效果及 F407 钢球闭环控制仍待完成。
- K230 字体及许可证随代码提交。本机设备备份、调试配置和编译缓存保存在各自电脑上。

两台电脑各自克隆本仓库，开始任务前更新远程分支，每项任务使用独立的 `codex/` 分支。完成后提交并推送，再经审查合并到 `main`。Git 传递已提交的代码与文档，任务进度请记录到仓库中。

以下保留原 MSPM0 示例说明。

## Example Summary

Empty project using DriverLib.
This example shows a basic empty project using DriverLib with just main file
and SysConfig initialization.

## Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| SYSCTL |  |  |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) for LaunchPad information, including user guide and hardware files.

| Pin | Peripheral | Function | LaunchPad Pin | LaunchPad Settings |
| --- | --- | --- | --- | --- |
| PA20 | DEBUGSS | SWCLK | N/A | <ul><li>PA20 is used by SWD during debugging<br><ul><li>`J101 15:16 ON` Connect to XDS-110 SWCLK while debugging<br><li>`J101 15:16 OFF` Disconnect from XDS-110 SWCLK if using pin in application</ul></ul> |
| PA19 | DEBUGSS | SWDIO | N/A | <ul><li>PA19 is used by SWD during debugging<br><ul><li>`J101 13:14 ON` Connect to XDS-110 SWDIO while debugging<br><li>`J101 13:14 OFF` Disconnect from XDS-110 SWDIO if using pin in application</ul></ul> |

### Device Migration Recommendations
This project was developed for a superset device included in the LP_MSPM0G3507 LaunchPad. Please
visit the [CCS User's Guide](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/tools/ccs_ide_guide/doc_guide/doc_guide-srcs/ccs_ide_guide.html#sysconfig-project-migration)
for information about migrating to other MSPM0 devices.

### Low-Power Recommendations
TI recommends to terminate unused pins by setting the corresponding functions to
GPIO and configure the pins to output low or input with internal
pullup/pulldown resistor.

SysConfig allows developers to easily configure unused pins by selecting **Board**→**Configure Unused Pins**.

For more information about jumper configuration to achieve low-power using the
MSPM0 LaunchPad, please visit the [LP-MSPM0G3507 User's Guide](https://www.ti.com/lit/slau873).

## Example Usage

Compile, load and run the example.
