# STM32 FreeRTOS 两轮自平衡车

![MCU](https://img.shields.io/badge/MCU-STM32F103C8-blue)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green)
![Language](https://img.shields.io/badge/Language-C-orange)
![Control](https://img.shields.io/badge/Control-Cascade%20PI%2FPD-purple)

基于 STM32F103C8 的两轮自平衡车项目。系统使用 MPU6050 获取车身姿态，使用左右轮编码器获取运动反馈，通过速度外环 PI 和直立内环 PD 计算电机输出，并支持蓝牙遥控、超声波测距和 OLED 状态显示。

软件使用 FreeRTOS 将控制、蓝牙、测距和显示划分为四个任务，并通过任务通知和队列连接中断与任务。

## 核心功能

| 功能 | 实现方式 |
| --- | --- |
| 车身平衡 | MPU6050 提供姿态和角速度数据，直立环计算平衡输出 |
| 速度控制 | 左右编码器提供轮速反馈，速度环 PI 产生目标倾角修正量 |
| 转向控制 | 根据转向输入和 Z 轴角速度计算转向输出，左右轮差速驱动 |
| 蓝牙遥控 | USART3 接收单字节命令，支持前进、后退、左转、右转和刹车 |
| 超声波测距 | PA3 触发测距，PA2 接收回波，TIM3 计时并换算距离 |
| 状态显示 | OLED 显示 Pitch、测距结果、左右编码器值和目标速度 |
| 倾倒保护 | 车身角度偏离机械平衡角超过阈值时停止电机输出 |

## 软件架构

### FreeRTOS 任务

| 任务 | 优先级 | 栈深度 | 职责 |
| --- | ---: | ---: | --- |
| Control | 4 | 256 | 等待 MPU6050 通知，采集姿态和编码器数据，运行控制算法并更新电机 |
| Bluetooth | 2 | 128 | 从蓝牙队列读取命令并更新目标速度和转向量 |
| Ultrasonic | 2 | 128 | 周期调用超声波测距函数 |
| OLED | 1 | 256 | 周期刷新运行状态 |

任务优先级数字越大，优先级越高。栈深度单位是 StackType_t；在 Cortex-M3 上每个栈单元为 4 字节。

### 中断与任务间通信

- MPU6050 数据就绪后触发外部中断。中断清除标志并通过 vTaskNotifyGiveFromISR() 唤醒控制任务；姿态读取和控制计算在任务中执行。
- USART3 接收中断读取蓝牙字节，通过 xQueueSendFromISR() 写入队列；蓝牙任务通过 xQueueReceive() 取出并解析命令。
- 控制任务通过 ulTaskNotifyTake() 等待传感器数据就绪通知；周期任务使用 vTaskDelayUntil() 等待下一次运行。

## 控制流程

MPU6050 数据就绪中断 → 任务通知 → 控制任务读取姿态与编码器 → 速度外环 PI 计算目标倾角修正量 → 直立内环 PD 计算平衡输出 → 转向控制生成转向输出 → 左右轮差速、限幅和 PWM 更新。

速度环输出与机械平衡角相加，作为直立环的目标角度。直立环计算出的平衡输出分别与转向输出相减、相加，得到左右电机控制量。PWM 由 TIM1 输出，电机方向由 GPIO 控制。

## 主要模块

| 模块 | 文件 | 职责 |
| --- | --- | --- |
| 应用入口与任务 | User/main.c | 外设初始化、队列和任务创建、调度器启动及任务主体 |
| 平衡与 PID | HARDWARE/control.c、HARDWARE/control.h | 速度 PI、直立 PD、转向计算和 MPU6050 外部中断通知 |
| MPU6050 与 DMP | HARDWARE/mpu6050.c、HARDWARE/inv_mpu.c | 传感器寄存器访问、DMP 初始化和姿态数据读取 |
| 蓝牙串口 | HARDWARE/usart3.c、HARDWARE/usart3.h | USART3 初始化与接收中断 |
| 编码器 | HARDWARE/encoder.c、HARDWARE/encoder.h | TIM2、TIM4 编码器接口及计数读取 |
| 电机与 PWM | HARDWARE/motor.c、HARDWARE/pwm.c | 电机方向、输出限幅及 TIM1 PWM |
| 超声波 | HARDWARE/sensor.c、HARDWARE/sensor.h | PA2/PA3 测距接口及 TIM3 计时 |
| OLED | HARDWARE/oled.c、HARDWARE/oled.h | OLED 初始化、字符和数值显示 |
| FreeRTOS 配置 | FreeRTOS/include/FreeRTOSConfig.h | 时钟、节拍、堆空间、中断优先级边界及栈溢出检查配置 |

## 编译与下载

本项目使用 Keil MDK 工程：

1. 使用 Keil MDK 打开 Project.uvprojx。
2. 选择工程目标并执行 Build。
3. 通过 ST-Link 或兼容下载器将生成的固件下载到 STM32F103C8。
4. 按实际硬件连接 MPU6050、编码器、电机驱动、蓝牙模块、超声波模块和 OLED。

工程基于 STM32F10x 标准外设库和 FreeRTOS Cortex-M3 移植。具体引脚、供电和电机驱动连接应以对应硬件板和原理图为准。

## 当前实现说明

- 超声波等待回波包含超时和任务延时，但等待粒度会影响测距精度，需结合实际模块和硬件验证。
- Delay_us() 直接操作 SysTick；FreeRTOS 也使用系统节拍。若在调度器运行期间调用该延时，应进一步检查其对系统节拍的影响。
- PID 参数、机械平衡角、传感器安装方向和电机极性与具体车辆相关，上车前需要校准。
- 工程编译通过不代表已完成整车稳定性、测距精度和长时间运行验证。

## 目录结构

项目目录包含 FreeRTOS 内核和移植层、HARDWARE 外设驱动、Library 标准外设库、Start 启动文件、System 系统支持代码、User 应用入口与 Keil 工程文件。