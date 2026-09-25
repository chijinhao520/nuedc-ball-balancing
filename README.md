# 车载平衡滚球 · 电赛代码参考

2026 年 TI 杯浙江省大学生电子设计竞赛 H 题项目的部分核心实现。平台为 **MSPM0G3507 + RDK X5**，包括滚球控制、循迹与轮速控制、视觉球位估计及串口通信。

**获奖：2026 年 TI 杯浙江省大学生电子设计竞赛省一等奖。**

本仓库提供代码阅读参考。未包含整机入口、硬件驱动、板卡资料、模型权重、训练数据或详细调试文档，不能直接编译烧录或一键运行整机；参数保留原样机取值，移植时需要重新标定。

| 目录 | 内容 |
|---|---|
| `firmware/control/` | 滚球状态机、循迹、轮速 PI、航向与里程控制 |
| `firmware/common/` | 视觉结果帧解析：AA55 帧头、小端载荷、XOR 校验 |
| `vision/processors/` | 管内目标筛选、球位观测器、YOLO 检测后处理 |
| `vision/serial_out.py` | 视觉结果串口打包与状态接收 |

固件原环境为 Keil MDK5、Arm Compiler 6.22、TI MSPM0 SDK 2.10.00.04。控制模块调用的姿态、电机、灰度、调度和整车服务接口需由接入工程提供；这些应用相关头文件与实现未随仓提供。

视觉代码使用 Python、NumPy、OpenCV；串口模块需要 pyserial，YOLO 球位处理器需要 RDK X5 的 `hobot_dnn`（或 `hobot_dnn_rdkx5`）及匹配的六输出头 BPU 模型。这里没有启动服务或训练入口。

IMU 标定流程另见 **[衡准 IMU](https://github.com/chijinhao520/hengzhun-imu/tree/d014f7fbed5cebdb95114626efb0886373338420)**（固定引用首版）。

作者：**池金壕**。自有代码采用 [PolyForm Noncommercial 1.0.0](LICENSE)，**未经另行授权禁止商业使用**；转载、修改须遵守许可并保留署名。第三方基础及范围见 [NOTICE](NOTICE)，本仓库的非商业条款不覆盖第三方代码。
