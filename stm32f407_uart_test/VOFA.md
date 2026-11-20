# VOFA+ 串口调参

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

该脚本烧录最新 `Firmware/F407_K230_Center_Balance.hex`，并复位开始原上电行程。修改源码后先运行 `.\tools\build_center.ps1`。

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

方向符号修正、稳定后锁存电机实际步数、取消球越界停机等既有行为保持。当前版已生成HEX，协议检查已执行；实际TTL连线和VOFA+实机交互尚未验证。

接口依据：[ST引脚说明](https://github.com/STMicroelectronics/STM32_open_pin_data/blob/master/mcu/STM32F407V%28E-G%29Tx.xml)、[JustFloat格式](https://www.vofa.plus/docs/learning/dataengines/justfloat/)、[VOFA命令绑定](https://www.vofa.plus/docs/learning/start/data_cmd_parameter/)。
