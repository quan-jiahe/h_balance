# 钢球平衡工程

K230 负责拍照、钢球识别和触控。STM32F407VET6 负责串口联调、X42S 点动和 ICM-45686 开机找平。钢球闭环控制的两个实现见下表。

| 用途 | 入口 | 当前状态 |
| --- | --- | --- |
| K230 独立采图 | [capture_training_photos.py](k230/capture_training_photos.py)、[部署说明](k230/README.md#独立采集训练照片) | 已实机验证 640×480 原始 JPEG、LCD 预览和每 10 张绿灯提示。串口状态显示可选。 |
| STM32F407VET6 | [Keil 工程](stm32f407_uart_test/MDK-ARM/F407_K230_UART.uvprojx)、[接线与烧录](stm32f407_uart_test/README.md) | 保留可重建的开机找平固件和已经烧录校验的独立点动固件。K230 已显示 F407 自检菜单。 |
| MSPM0 稳球 | [main.c](main.c)、[balance_pid.c](user_driver/balance_pid.c)、[pid.c](user_driver/pid.c) | CCS 工程。按毫米位置做 PID，保留线性和平方速度补偿 KFF/KFF2、定点和自动序列。 |
| 旧 F407 稳球参考 | [位置—速度控制](reference/f407_position_speed.c.txt) | 从 F407bujin 原工程摘录。以像素位置计算速度参考、制动和中心稳定判定，输出相对水平位置的有符号电机脉冲目标。完整旧工程和角度限幅版本保存在本机设备备份。 |
| K230 识别基线 | [main.py](k230/main.py)、[Model2 说明](k230/MODEL.md) | 保留 YOLO11 推理、相机预览、触控和二进制串口。像素到毫米的标定仍待完成，当前观测为 `LOST`。 |

旧 F407 控制器原本在收到新视觉帧时调用，输入是目标 X 与测量 X（像素）及毫秒时钟。旧协议的 `BASIC.cy` 可携带目标 X，见本机 `device_backups/stepper_reference_20261118/F407bujin__2/Core/Src/main.c`。当前 Keil 工程运行的是找平与点动，接入钢球闭环仍待完成。

## 下一阶段

等待 K230 完成采图。将 `/data/steel_ball_photos/batch_NNN/` 整批复制到电脑的 `datasets/steel_ball/raw/`，保留批次与 JPEG 原件。筛除损坏或重复帧，标注可见钢球并保留无球负样本。按采集批次划分训练、验证和测试集，避免相邻帧跨集合，再训练单类 `steel` YOLO 模型。训练结果放在 `runs/`。上板前以 Model2 为基线评估误检、漏检和定位误差，再导出与目标 K230 运行时兼容的 `.kmodel`。

## 协作与回归

两台电脑从同一 Git 仓库更新。使用 `codex/` 任务分支，经审查合入 `main`。采集准备的整合分支为 `codex/pre-yolo-cleanup`。设备备份、模型下载文件、原始照片、训练输出和编译缓存留在本机。

本次整理通过 13 项 K230 回归和 42 项 F407 Cortex-M 模拟检查。Keil 干净构建复现了找平固件，采图脚本和共用串口模块与 K230 读回文件一致。

- K230 回归：`py -B -m unittest discover -s tools -p 'test_*.py' -v`
- F407 编译和 Cortex-M 模拟：[F407 验证说明](stm32f407_uart_test/README.md#验证范围)
- 设备恢复与接手：[AGENTS.md](AGENTS.md)
