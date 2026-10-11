#include "control.h"
#include "stm32f10x.h"
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "inv_mpu.h"
extern TaskHandle_t ControlTask_Handle;  // 这些变量在 usart3.c 中定义

// ============ 全局变量定义 ============
// 编码器数据
int Encoder_Left = 0;
int Encoder_Right = 0;
// 在 #include 之后添加

// 电机PWM输出
int MOTO1 = 0;
int MOTO2 = 0;

// MPU6050角度数据
float Pitch = 0, Roll = 0, Yaw = 0;

// 陀螺仪原始数据
short gyrox = 0, gyroy = 0, gyroz = 0;

// 加速度原始数据
short aacx = 0, aacy = 0, aacz = 0;

// 超声波距离
int length = 0;

// 安全停车标志
char stop = 0;

// ============ 原有变量定义 ============
float Med_Angle = 0;        // 机械中值
float Target_Speed = 0;     // 期望速度（俯仰）
float Turn_Speed = 0;       // 期望速度（偏航）

// PID参数
float Vertical_Kp = 250;    // 直立环KP
float Vertical_Kd = 0.6;    // 直立环KD
float Velocity_Kp = 0.65;   // 速度环KP
float Velocity_Ki = 0.00325;// 速度环KI
float Turn_Kd = 0.6;        // 转向环KD
float Turn_Kp = 20;         // 转向环KP

// PID输出变量
int Vertical_out = 0;
int Velocity_out = 0;
int Turn_out = 0;

// 速度限幅宏定义
#define SPEED_Y 15  // 俯仰(前后)最大设定速度
#define SPEED_Z 80  // 偏航(左右)最大设定速度

// ============ 函数声明 ============
int Vertical(float Med, float Angle, float gyro_Y);
int Velocity(int Target, int encoder_left, int encoder_right);
int Turn(int gyro_Z, int RC);

// ============ 外部中断处理函数 ============
// MPU6050 data-ready ISR only wakes the control task.
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5) != 0)
    {
        if(PBin(5) == 0)
        {
            BaseType_t higherPriorityTaskWoken = pdFALSE;
            EXTI_ClearITPendingBit(EXTI_Line5);
            if (ControlTask_Handle != NULL) vTaskNotifyGiveFromISR(ControlTask_Handle, &higherPriorityTaskWoken);
            portYIELD_FROM_ISR(higherPriorityTaskWoken);
        }
    }
}
int Vertical(float Med, float Angle, float gyro_Y)
{
    int PWM_out;
    PWM_out = Vertical_Kp * (Angle - Med) + Vertical_Kd * (gyro_Y - 0);
    return PWM_out;
}

/*********************
速度环PI：Kp*Ek + Ki*Ek_S
入口：目标速度、左右编码器值
出口：速度环输出
*********************/
int Velocity(int Target, int encoder_left, int encoder_right)
{
    static int Encoder_S = 0;
    static int EnC_Err_Lowout_last = 0;
    static int EnC_Err_Lowout = 0;
    int PWM_out, Encoder_Err;
    float a = 0.7;
    
    // 1. 计算速度偏差
    Encoder_Err = ((encoder_left + encoder_right) - Target);
    
    // 2. 对速度偏差进行低通滤波
    EnC_Err_Lowout = (1 - a) * Encoder_Err + a * EnC_Err_Lowout_last;
    EnC_Err_Lowout_last = EnC_Err_Lowout;
    
    // 3. 对速度偏差积分
    Encoder_S += EnC_Err_Lowout;
    
    // 4. 积分限幅
    if(Encoder_S > 10000) Encoder_S = 10000;
    if(Encoder_S < -10000) Encoder_S = -10000;
    
    // 5. 停车时清零积分
    if(stop == 1)
    {
        Encoder_S = 0;
        stop = 0;
    }
    
    // 6. 速度环控制输出计算
    PWM_out = Velocity_Kp * EnC_Err_Lowout + Velocity_Ki * Encoder_S;
    return PWM_out;
}

/*********************
转向环：系数*Z轴角速度 + 系数*遥控数据
入口：Z轴陀螺仪数据、遥控数据
出口：转向环输出
*********************/
int Turn(int gyro_Z, int RC)
{
    int PWM_out;
    PWM_out = Turn_Kd * gyro_Z + Turn_Kp * RC;
    return PWM_out;
}
