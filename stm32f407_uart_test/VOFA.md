# VOFA+ 串口调参

当前 OLED 模式菜单固件锁定在线写参。JustFloat 通道和发送周期沿用下表，I6 的 `-3` 表示有效写参指令已被锁定。实际固化值按 I7～I17 顺序为 `0.5, 0.7, 0.875, 1.5, 1.6, 30, 40, 120, 0, 0, 150`，取自实机进入 S3 时的 VOFA 读数。下文在线写参步骤适用于此前的调参版。

使用 USB 转 3.3V TTL 模块，电脑通过模块连接 F407 的独立 USART1。ST-Link 保留作烧录用途，K230 继续使用 USART3 PD8/PD9。

| USB转TTL | STM32F407VET6 |
| --- | --- |
| RX | PA9 / USART1 TX |
| TX | PA10 / USART1 RX |
| GND | GND |

板子正常供电时不用连接TTL模块的VCC。选择3.3V TTL电平。

## 烧录

在 `C:\Users\LENOVO\Desktop\ultra\h_balance` 的 PowerShell 中执行：

```powershell
.\stm32f407_uart_test\flash_center.ps1
```

该脚本使用本机Keil的CMSIS-DAP下载器：先重新编译居中目标，同时更新AXF和发布HEX，再下载并检查校验结果、复位开始原上电行程。默认Keil路径为 `D:\keil5\core\UV4\UV4.exe`，可用 `-Keil` 指定其他路径；不再调用仅面向ST-Link的CubeProgrammer命令。只检查模块和驱动可运行 `./stm32f407_uart_test/flash_center.ps1 -Check`，不会连接目标MCU或烧录。

沁恒一体模块的RX/TX只负责VOFA通信。烧录还需 **SWDIO→PA13、SWCLK→PA14、GND→GND**。工程已配置CMSIS-DAP、SWD 100 kHz和STM32F407VE的512 KB Flash算法。烧录前退出Keil调试，将机构置于机械下端并取出球。

共享uvoptx明确关闭 **Settings → Pack → Debug Description → Enable**，使用常规CMSIS-DAP驱动连接和原Flash算法。此前省略此设置时Keil启用了DFP的 `DebugDeviceUnlock → CheckID`，实机报 `0xE00FFFE8` 读取失败。修改后保留编程校验；若Keil已打开工程，需关闭再重开以加载新配置。配置已修正，尚未验证修正后的实机下载；若仍不能访问目标，检查SWD接线、供电和复位线，而不是继续改平衡参数。

若COM串口正常而“WCH CMSIS-DAP”显示错误码28，缺少的是调试接口驱动。到[沁恒官方WCH-LinkUtility页面](https://www.wch.cn/downloads/WCH-LinkUtility_ZIP.html)下载并解压，用管理员权限安装 `Drv_Link/WCHLinkDrv_WHQL_S.exe`，然后重新插拔模块。不要用CH340驱动替代，也不要更换串口接口的驱动。驱动安装后，Keil中Debug应显示CMSIS-DAP Debugger，目标选择F407_K230_Center_Balance。

## VOFA+ 设置

1. 接口选串口及TTL模块对应COM，115200、8位数据、无校验、1位停止位，无流控。
2. 接收协议选择 **JustFloat**，连接后添加波形控件，先显示通道0、1。
3. 参数指令用 **Str/ASCII** 模式，以换行结束，例如 `kp=0.30\n`。可启用发送窗口的自动追加换行，或者在命令模板中填写换行，不要重复手工输入字符反斜杠和n。
4. 滑块、输入框或按钮可绑定 `kp=%f\n`、`kv=%f\n` 等模板。小数使用点号。

设备每50 ms发送一帧，18个小端float32和四字节帧尾 `00 00 80 7F`。同一串口不混发ASCII回显，命令结果和参数当前值在数据通道中查看。

| 通道（从0开始） | 含义 |
| --- | --- |
| 0 | 最近收到的钢球位置，mm |
| 1 | 最近收到的钢球速度，mm/s |
| 2 | 当前电机累计步数 |
| 3 | 电机目标步数 |
| 4 | 状态：0上电/调平，1等球，2纠偏，3稳定保持，4人工停止，5故障 |
| 5 | 观测有效：1有效，0丢球/过期，只有为1时通道0/1才代表当前有效观测 |
| 6 | 最近命令结果：0尚无命令，1成功，-1命令/参数错误，-2串口接收错误 |
| 7～17 | kp、kv、ki、gain、brake、max_speed、accel、ref_accel、kff、kff2、max_offset |

## 参数命令

```text
kp=0.30
kv=0.80
ki=0
gain=1.50
brake=1.60
max_speed=30
accel=88
ref_accel=120
kff=0
kff2=0
max_offset=150
```

每行是一个独立命令，回车/换行后生效，也可一次发送多行。不支持通过该串口启动电机、修改方向、写Flash等命令。

| 参数 | 接收范围 |
| --- | --- |
| kp / ki / gain | 0～3 |
| kv | 0～4 |
| brake | 1～4 |
| max_speed | 1～150 mm/s |
| accel | 1～300 mm/s² |
| ref_accel | 1～500 mm/s² |
| kff | -3～3 |
| kff2 | -0.05～0.05 |
| max_offset | 1～300步 |

未知参数、NaN/Inf、超范围值和多余内容均拒绝修改。命令过长、接收错误或半条命令超时后，会丢弃到下一次换行，再接收完整的新命令。收发使用独立缓冲，发送队列满时跳过整帧，不阻塞稳球循环。

在线修改立即影响RAM参数，复位恢复 `Core/Src/center_balance.c` 的默认值。找到合适数值后把参数写回该文件、编译烧录以永久保存。本次保留用户已修改的默认 `kp=0.5`。

## 自适应姿态补偿

自检结束只把当前位置当成起始参考，不假定它水平。球距离目标至少10 mm，且朝目标的速度不超过4 mm/s（包括静止或向外滚），持续约0.3 s、且未处于制动时，开始慢速学习姿态补偿。球正常回中或制动时保留已有补偿；零点附近保留低速学习。过零不会缩小或清除补偿。

补偿以输出步数独立保存。`ki=0` 只冻结学习，不清除已有补偿；修改 `gain` 不会重新缩放已有补偿。人工停止、丢球和超时仍立即停止发脉冲、保持实际位置，同时保留所学补偿，恢复后重置速度规划与停滞计时，再以原目标变化限制逐步恢复控制。停止电机应使用K230返回/停止，不能把 `gain=0` 当作停机命令。

`ki` 控制累加速度。积分使用的误差最多取10 mm，因此远处静止时，每秒最多增加约 `10 × ki × gain` 步的输出。例如 `ki=0.3`、`gain=1.5` 时约4.5步/s；默认 `ki=0.875`、`gain=1.5` 时约13.1步/s。这些是未限幅时的指令变化率，不是实测倾角或电机反馈。

|x|≤2 mm、|v|≤4 mm/s持续约0.3 s，且电机已追上目标后，进入S3，把当时完整的实际保持输出转移到新的参考姿态，并将增量积分归零；转移前后目标步数相同。偏差达到4 mm或球速达到10 mm/s时恢复纠偏，参考仍是学到的姿态。不会强制返回310步。

`max_offset` 限制的是起始/最近稳定参考姿态周围的总纠偏量（动态输出加学习补偿），积分自身也有同样限幅。触及此幅度或0～650步机械边界时停止同向积分，反向积分仍可退出；电机指令计步明显落后时也不继续向落后方向增加积分。不存在靠移动参考来绕开饱和的逻辑。初始捕获时所需平衡姿态和瞬态制动余量必须在这些限幅允许的范围内，才可能完成回中。

观察 I0/I1、I2/I3和I4/I5：球远处停滞时I3应逐步变化，正常回中时制动，稳定后I2/I3保持学到的步数。I2仍只是已发STEP计数，不是编码器反馈。所有18个通道及11个参数命令的顺序保持不变。

| 参数 | 当前作用 |
| --- | --- |
| kp / kv | 位置纠偏与速度误差反馈；kv同时影响目标球速驱动力和阻尼 |
| ki | 学习速率，0冻结学习；不清除已学姿态 |
| gain | 放大动态纠偏和之后的学习速率，不缩放已学姿态 |
| brake | 制动阶段速度反馈倍率 |
| max_speed / accel / ref_accel | 球速包络上限、制动规划加速度、目标速度上升速率 |
| kff / kff2 | 原线性/平方速度补偿接口，先保留0 |
| max_offset | 相对起始/最近稳定姿态的总纠偏幅度，包括学习补偿 |

起始建议（不是已实测参数）：`kp=0.5`、`kv=0.7`、`ki=0.3`、`gain=1.5`、`brake=1.6`、`max_speed=30`、`accel=40`、`ref_accel=60`、`kff=0`、`kff2=0`、`max_offset=150`。每行以实际换行结束，发送后核对I6与I7～I17。源码默认值和用户12步/4 ms时序未被这些建议覆盖。原理、适用边界和可复现的验证结果见 [CONTROL.md](CONTROL.md)。

方向符号修正和取消球越界停机等既有行为保持。已在COM17实收过旧固件18通道数据；WCH-Link一体模块需关闭VOFA的Break，DTR/RTS也保持关闭。新控制未烧录、未在线写参，实机适应性仍待验证。

接口依据：[ST引脚说明](https://github.com/STMicroelectronics/STM32_open_pin_data/blob/master/mcu/STM32F407V%28E-G%29Tx.xml)、[JustFloat格式](https://www.vofa.plus/docs/learning/dataengines/justfloat/)、[VOFA命令绑定](https://www.vofa.plus/docs/learning/start/data_cmd_parameter/)。
