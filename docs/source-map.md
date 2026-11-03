# 源码导航

[返回首页](../README.md) · [上手与工程集成](getting-started.md)

这里按用途索引根目录的全部 C / H 文件。文件维持原始名称与位置，便于对照设计报告和历史实验记录。

## 姿态与控制入口

以下文件均包含自己的 `main()`，属于不同实验版本，不能作为同一目标的多个入口一起编译。

| 文件 | 阅读重点 |
| --- | --- |
| [`main.c`](../main.c) | MPU6050 采样、Mahony IMU 解算、三轴控制函数、编码器测速、PWM 输出与 LoRa / VOFA 数据发送 |
| [`main_newpid.c`](../main_newpid.c) | 控制与 PWM 设置的另一版本，加入串口 FIFO 接收及指令实验；磁力计初始化和采样调用处于注释状态 |
| [`main_teacher.c`](../main_teacher.c) | 保留的控制参考版本，包含姿态解算、编码器读取与三轴 PID / PWM 逻辑 |
| [`main_teacher2.c`](../main_teacher2.c) | 另一控制参考版本，包含陀螺仪反馈、单轴 PWM 控制与 GPIO 脉冲计数 |
| [`scan.c`](../scan.c) | 串口接收触发的扫描 / 电机控制实验，结合定时器启停、姿态解算与数据输出 |
| [`scan2.c`](../scan2.c) | 扫描实验的简化版本，依据接收数据长度切换 PWM 和定时器状态 |

## 通信、定位与载荷入口

| 文件 | 阅读重点 |
| --- | --- |
| [`main_loraTEST.c`](../main_loraTEST.c) | MPU6050 与 Mahony IMU 数据采集，经 UART3 输出 VOFA JustFloat 格式数据 |
| [`main_lora_and_camera.c`](../main_lora_and_camera.c) | VC0706 摄像头指令、UART2 接口、LoRa 接收触发与图像回传实验流程 |
| [`GPS.c`](../GPS.c) | 独立 GNSS 测试入口，调用底层库的初始化、解析和接收回调；文件并非可直接并入其他入口的通用 GPS 驱动 |

这些文件同样包含 `main()`。摄像头与 GNSS 文件保留了实验中的未完成处理，阅读时应区分函数流程和已验证的数据链路。

## 姿态算法与磁力计

| 文件 | 作用 |
| --- | --- |
| [`MahonyAHRS.c`](../MahonyAHRS.c) / [`MahonyAHRS.h`](../MahonyAHRS.h) | 四元数状态与 Mahony 更新接口；提供带磁力计的 `MahonyAHRSupdate()` 和不带磁力计的 `MahonyAHRSupdateIMU()` |
| [`hmc5883.c`](../hmc5883.c) / [`hmc5883.h`](../hmc5883.h) | HMC5883 软件 I²C、寄存器配置、磁场数据读取及角度计算 |
| [`qmc5883.c`](../qmc5883.c) / [`qmc5883.h`](../qmc5883.h) | QMC5883 软件 I²C、寄存器配置、磁场数据读取及角度计算 |
| [`main_HMC5883test.c`](../main_HMC5883test.c) | HMC5883 初始化和周期读取的独立测试入口 |
| [`main_QMC5883test.c`](../main_QMC5883test.c) | QMC5883 初始化和周期读取的独立测试入口 |

主控制版本目前使用 IMU 更新接口；两个磁力计测试入口用于分别验证各自驱动，不代表它们已同时集成进主控制程序。

## 中断与调试接口

| 文件 | 作用 |
| --- | --- |
| [`isr.c`](../isr.c) / [`isr.h`](../isr.h) | CH32V307VCT6 中断框架与回调连接，保留 SEEKFREE 文件头和许可声明 |
| [`Vofa.h`](../Vofa.h) | 浮点数 / 字节转换联合体及 VOFA 函数声明；相关实现分散在各实验入口中，仓库没有独立的 `Vofa.c` |

## 阅读路线

1. 先从 [`main.c`](../main.c) 的初始化和主循环了解数据流。
2. 对照 [`MahonyAHRS.c`](../MahonyAHRS.c) 理解姿态更新，再查看控制函数与编码器反馈。
3. 用 [`main_loraTEST.c`](../main_loraTEST.c) 阅读遥测数据打包与串口输出。
4. 根据实验目标进入对应传感器、扫描、GNSS 或摄像头文件。
5. 集成前核对[环境、入口选择和参数差异](getting-started.md)，尤其是中断回调与采样周期。
