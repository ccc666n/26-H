# 2026 年电赛 H 题：车载平衡滚球运动控制系统

[中文](README.md) | [English](README_EN.md)

> 2026 年全国大学生电子设计竞赛广东赛区
> 题目：车载平衡滚球运动控制系统（H 题）
> 成绩：省一等奖

本仓库保存该项目当前可公开的控制固件与竞赛资料，用于技术交流和复现参考。仓库不包含求职信息、个人简历、联系方式或其他不必要的个人信息。

## 项目概述

本项目将循迹移动底盘、视觉钢球定位和水管倾角闭环控制整合为一个机器人运动控制系统：

- 底盘以 `MSPM0G3507` 为主控，使用 8 路灰度传感器、双 GMR 编码器、MG513 直流减速电机和 TB6612 电机驱动器。
- MaixCAM2 视觉端识别钢球，并通过 UART 向主控发送位置与速度数据。
- 主控以固定周期运行预测误差 PID，控制 X42S 闭环步进驱动器，带动水管倾角机构调节钢球运动。
- OLED 与按键用于模式选择、状态显示和计时。

技术报告记录了循迹、定点停车、静态滚球定位、短距离动态平衡、整圈平衡和任意指定位置平衡等测试。

## 仓库结构

```text
.
|-- docs/
|   |-- design-report.pdf
|   `-- problem-statement.pdf
|-- firmware/
|   `-- mspm0g3507-ccs/
|       |-- AnalogGray/       # 灰度传感器采集与标定
|       |-- Camera/           # 视觉结果的 UART 接收与校验
|       |-- Encoder/          # 双轮编码器计数与速度采样
|       |-- GetYaw/           # IMU 航向角更新与归零
|       |-- ICM42688/         # ICM42688 驱动
|       |-- IMU/              # 姿态解算
|       |-- Motor/            # TB6612 电机控制
|       |-- Stepper/          # X42S 闭环步进电机指令
|       |-- Track/            # 循迹与竞赛任务状态机
|       |-- empty.c           # 固件入口
|       |-- empty.syscfg      # SysConfig 配置源文件
|       `-- targetConfigs/    # CCS 目标配置
|-- .gitattributes
|-- .gitignore
`-- THIRD_PARTY_NOTICES.md
```

## 系统架构

```text
8 路灰度传感器 ─┐
双 GMR 编码器 ──┼─> MSPM0G3507 ─> TB6612 ─> MG513 底盘电机
ICM42688 IMU ───┤
MaixCAM2 UART ──┤
                └────────────> X42S ─> 闭环步进电机 ─> 水管倾角机构
```

固件采用主循环调度与周期控制任务结合的方式：

- `TIMER_0` 负责 IMU 通路的姿态更新。
- `TIMER_ENCODER` 每 50 ms 采样一次双轮编码器计数。
- `Track` 模块实现任务状态机、循迹、速度反馈、路段切换和停车逻辑。
- `Camera` 模块校验并解析视觉 UART 数据帧。
- `Stepper` 模块将目标倾角转换为 X42S 的位置控制指令。

## 控制方法

### 循迹与速度闭环

8 路灰度传感器用于估计车辆相对黑线的横向偏差，并据此生成左右轮差速目标。双 GMR 编码器提供速度与累计里程反馈，用于速度闭环、分段减速和停车判断。

### 钢球预测误差 PID

技术报告中的钢球控制周期为 20 ms，位置误差与速度前馈组成预测误差：

```text
预测误差 = 位置误差 + kv × 钢球速度
```

控制器将预测误差映射为水管目标倾角。报告记录的角度限幅为 `+/-50°`；视觉目标丢失时，控制器清零状态并使水管回到零角度。

### 视觉 UART 协议

当前 MSPM0 固件中的 `Camera/camera.c` 采用 XOR 校验，并期望接收以下数据帧：

```text
$BALL,<valid>,<position_error_cm>,<absolute_position_cm>,<speed_cm_s>,<confidence>,<timestamp_ms>*<checksum>\n
```

其中 `valid` 非零表示本帧检测到钢球。接收端保存位置误差、绝对位置、速度、置信度与时间戳，供控制任务读取。

## 报告中的测试结果

| 测试项目 | 报告记录结果 |
| --- | --- |
| 单圈循迹并停车 | 平均 18.35 s；最大停车偏差 1.7 cm |
| 静态滚球定位 | 最长 3.83 s；最大误差 0.9 cm |
| AB 段动态平衡 | 最长 6.91 s |
| 整圈中心位置平衡 | 最长 24.90 s |
| 整圈指定位置平衡 | 最长 25.12 s；最大误差 0.9 cm |

完整题目要求、测试记录、系统框图和报告源代码摘录见 [docs/problem-statement.pdf](docs/problem-statement.pdf) 与 [docs/design-report.pdf](docs/design-report.pdf)。

## CCS 构建与烧录

本工程面向 `MSPM0G3507`，使用 Code Composer Studio（CCS）与 TI MSPM0 SDK。

1. 安装 CCS 和 TI MSPM0 SDK。
2. `empty.syscfg` 记录的 SDK 版本为 `mspm0_sdk@2.11.00.07`，SysConfig 版本为 `1.26.2+4477`；复现竞赛构建时优先使用这一组合。
3. 在 CCS 中从 `firmware/mspm0g3507-ccs` 导入工程。
4. 若 CCS 要求选择目标配置，使用 `targetConfigs/MSPM0G3507.ccxml`。
5. 打开 `empty.syscfg`，确认目标为 LQFP-64 封装的 MSPM0G3507，然后构建 `Debug` 配置。
6. 通过 SWD 连接兼容的 MSPM0G3507 调试硬件，在 CCS 中下载固件。

仓库刻意忽略 `Debug/` 等构建产物；导入工程后由本地 CCS 重新生成即可。

## 当前公开范围

当前包含：

- MSPM0G3507 CCS 固件快照
- SysConfig 与 CCS 目标配置
- 技术报告
- H 题原始赛题

当前未包含：

- MaixCAM2 端 Python 源码
- YOLO 模型和训练数据
- WebRTC 配置
- 原理图、PCB 与机械图
- 实物照片、测试曲线与演示视频

报告附录中的视觉端示例与当前 `Camera/camera.c` 的 UART 字段数量并不完全一致。视觉端源码补充后，将统一协议版本，再声明端到端复现流程完整可用。

## 版权与第三方代码

当前不声明覆盖全仓库的许可证。现有快照包含带有第三方版权声明的文件，包括 TI 的版权和再分发条件；使用、再分发或重新授权前，请先阅读 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

后续完成模块来源审计后，将把原创代码与第三方组件的授权范围分别说明。
