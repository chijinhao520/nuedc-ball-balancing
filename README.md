# 车载平衡滚球 · 电赛代码参考

2026 年 TI 杯浙江省大学生电子设计竞赛 H 题项目的部分核心实现。平台为 **MSPM0G3507 + RDK X5**，包括滚球控制、循迹与轮速控制、视觉球位估计及串口通信。

**获奖：2026 年 TI 杯浙江省大学生电子设计竞赛省一等奖。**

本仓库提供代码阅读参考。未包含整机入口、硬件驱动、板卡资料、模型权重、训练数据或详细调试文档，不能直接编译烧录或一键运行整机；参数保留原样机取值，移植时需要重新标定。

## 平衡控制思路

系统通过视觉估计槽内球的位置与速度，再调整摆杆角度控制滚球；小车行驶时，将底盘运动带来的扰动一起纳入补偿。

- **感知与估计**：YOLO 检测结合管带 ROI 筛选和跳变门控，将球心像素坐标标定为实际位置，再由 α-β 观测器估计球位与球速。
- **静止滚球**：针对静摩擦死区和不同位置的平衡角差异，采用平衡角前馈与分段状态机，依次组织推动、反向制动、低速松手和末端镇定；制动同时考虑剩余距离、球速及运动方向。
- **行驶稳球**：在球位反馈基础上，结合底盘加减速规划提供前馈，起动阶段辅以受限的 IMU 信号补偿。该信号受安装姿态和俯仰影响，需要按样机校准。
- **协同控制**：循迹与轮速闭环负责底盘行驶，滚球状态机负责目标位置和摆杆动作，通过任务接口与视觉通信交换状态。

核心实现见 [ball_ctrl.c](firmware/control/ball_ctrl.c) 和 [ball_yolo_1d.py](vision/processors/ball_yolo_1d.py)。这里只概述控制结构，具体参数不代表其他机械结构下的通用整定值。

## 代码范围

| 目录 | 内容 |
|---|---|
| `firmware/control/` | 滚球状态机、循迹、轮速 PI、航向与里程控制 |
| `firmware/common/` | 视觉结果帧解析：AA55 帧头、小端载荷、XOR 校验 |
| `vision/processors/` | 管内目标筛选、球位观测器、YOLO 检测后处理 |
| `vision/serial_out.py` | 视觉结果串口打包与状态接收 |

固件原环境为 Keil MDK5、Arm Compiler 6.22、TI MSPM0 SDK 2.10.00.04。控制模块调用的姿态、电机、灰度、调度和整车服务接口需由接入工程提供；这些应用相关头文件与实现未随仓提供。

视觉代码使用 Python、NumPy、OpenCV；串口模块需要 pyserial，YOLO 球位处理器需要 RDK X5 的 `hobot_dnn`（或 `hobot_dnn_rdkx5`）及匹配的六输出头 BPU 模型。这里没有启动服务或训练入口。

IMU 标定流程另见 **[衡准 IMU](https://github.com/chijinhao520/hengzhun-imu/tree/d014f7fbed5cebdb95114626efb0886373338420)**（固定引用首版）。

## 软件基础与引用

| 来源 | 在原项目中的用途 |
|---|---|
| [逐飞科技 MSPM0G3507 库](https://gitee.com/seekfree/MSPM0G3507_Library) | IMU660RB 驱动移植基础，GPL-3.0-or-later |
| [x-io Fusion](https://github.com/xioTechnologies/Fusion) | 姿态融合与在线偏置估计，MIT |
| [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) | 钢球检测模型的训练基础 |
| [D-Robotics RDK](https://github.com/D-Robotics) | X5 BPU 推理运行时 |
| [TI MSPM0 SDK](https://www.ti.com/tool/MSPM0-SDK) | MCU 外设与固件开发基础 |

本项目的工作包括控制逻辑、视觉处理链路、标定与系统集成。上述第三方源码、SDK 和模型权重未随本参考仓库分发，使用时应遵守各自许可；详细范围见 [NOTICE](NOTICE)。

作者：**池金壕**。自有代码采用 [PolyForm Noncommercial 1.0.0](LICENSE)，本 README 自有说明采用 [CC BY-NC 4.0](LICENSES/CC-BY-NC-4.0.txt)。**禁止未经许可的商业使用**；允许用途、署名要求与第三方范围见 [NOTICE](NOTICE)，具体以完整许可为准。PolyForm 对教育、公益、公共研究等机构有明确许可条款；本仓库的非商业条款不覆盖第三方代码。
