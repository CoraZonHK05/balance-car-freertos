#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "usart3.h"          
#include "oled.h"
#include "mpu6050.h"
#include "encoder.h"
#include "motor.h"
#include "pwm.h"
#include "sensor.h"
#include "exti.h"
#include "control.h"
#include "inv_mpu.h"
// FreeRTOS头文�?
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

// ==================== 任务句柄 ====================
TaskHandle_t ControlTask_Handle;
TaskHandle_t BluetoothTask_Handle;
TaskHandle_t UltrasonicTask_Handle;
TaskHandle_t OLEDTask_Handle;

// ==================== 队列句柄 ====================
QueueHandle_t bluetoothQueue;    // 蓝牙指令队列

// ==================== 外部变量声明 ====================
extern int length;
extern int Encoder_Left, Encoder_Right;
extern float Pitch, Roll, Yaw;
extern short gyrox, gyroy, gyroz;
extern u8 Fore, Back, Left, Right;
extern float Target_Speed, Turn_Speed;
extern int MOTO1, MOTO2;
extern float Med_Angle;           // 机械中�?
extern int Vertical_out, Velocity_out, Turn_out;  // PID输出
extern float Turn_Kd;              // 转向环Kd

// ==================== 函数声明 ====================
void vTaskControl(void *pvParameters);
void vTaskBluetooth(void *pvParameters);
void vTaskUltrasonic(void *pvParameters);
void vTaskOLED(void *pvParameters);

// ==================== main函数 ====================
int main(void)
{
    // 1. 硬件初始化（所有外设）
Delay_ms(100);
    NVIC_Config();              // 中断分组（在FreeRTOS启动前）
    uart3_init(9600);           // 串口3（蓝牙）- 修正�?usart3_init
    
    OLED_Init();                // OLED初始�?
    OLED_Clear();               // 清屏
    
    MPU_Init();
    mpu_dmp_init();                 // MPU6050初始�?
    MPU6050_EXTI_Init();        // MPU6050外部中断初始�?
    
    Encoder_TIM2_Init();        // 左电机编码器
    Encoder_TIM4_Init();        // 右电机编码器
    
    Motor_Init();               // 电机IO初始�?
    PWM_Init_TIM1(0, 7199);     // PWM初始化（10KHz�?
    
    TIM3_Int_Init();            // 超声波定时器初始�?
    
    // 2. 创建FreeRTOS队列
    bluetoothQueue = xQueueCreate(5, sizeof(u8));  // 蓝牙指令队列，深�?
    
    // 3. 创建FreeRTOS任务
    xTaskCreate(vTaskControl,     "Control",     256, NULL, 4, &ControlTask_Handle);     // 优先�?（最高）
    xTaskCreate(vTaskBluetooth,   "Bluetooth",   128, NULL, 2, &BluetoothTask_Handle);   // 优先�?
    xTaskCreate(vTaskUltrasonic,  "Ultrasonic",  128, NULL, 2, &UltrasonicTask_Handle);  // 优先�?
    xTaskCreate(vTaskOLED,        "OLED",        256, NULL, 1, &OLEDTask_Handle);        // 优先�?（最低）
    
    // 4. 启动FreeRTOS调度�?
    vTaskStartScheduler();
    
    // 5. 正常情况下不会运行到这里
    while(1);
}

// ==================== 控制任务（最重要�?====================
void vTaskControl(void *pvParameters)
{
    int PWM_out;
    
    while(1)
    {
        // 等待MPU6050数据就绪中断的通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // 1. 采集编码器数�?
        Encoder_Left = -Read_Speed(2);      // 电机相对安装，取�?
        Encoder_Right = Read_Speed(4);
        
        // 2. 读取MPU6050角度和陀螺仪数据
			  mpu_dmp_get_data(&Roll,&Pitch,&Yaw);
        MPU_Get_Gyroscope(&gyroy, &gyrox, &gyroz);
        
        // 3. 速度环PI控制
        Velocity_out = Velocity(Target_Speed, Encoder_Left, Encoder_Right);
        
        // 4. 直立环PD控制
        Vertical_out = Vertical(Velocity_out + Med_Angle, Pitch, gyroy);
        
        // 5. 转向环控�?
        Turn_out = Turn(gyroz, Turn_Speed);
        
        // 6. 最终PWM输出
        PWM_out = Vertical_out;
        MOTO1 = PWM_out - Turn_out;     // 左电�?
        MOTO2 = PWM_out + Turn_out;     // 右电�?
        Limit(&MOTO1, &MOTO2);           // 限幅
        Load(MOTO1, MOTO2);              // 加载到电�?
        
        // 7. 安全检测（角度过大时停车）
        Stop(&Med_Angle, &Pitch);
    }
}

// ==================== 蓝牙任务 ====================
void vTaskBluetooth(void *pvParameters)
{
    u8 data;
    
    while(1)
    {
        // 从队列接收蓝牙数据（阻塞100ms�?
        if(xQueueReceive(bluetoothQueue, &data, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // 解析遥控指令
            switch(data)
            {
                case 0x00:  // 刹车
                    Fore = 0; Back = 0; Left = 0; Right = 0;
                    Target_Speed = 0;
                    Turn_Speed = 0;
                    break;
                    
                case 0x05:  // 前进
                    Fore = 1; Back = 0;
                    break;
                    
                case 0x01:  // 后退
                    Fore = 0; Back = 1;
                    break;
                    
                case 0x03:  // 右转
                    Right = 1; Left = 0;
                    break;
                    
                case 0x07:  // 左转
                    Left = 1; Right = 0;
                    break;
                    
                default:    // 其他指令视为刹车
                    Fore = 0; Back = 0; Left = 0; Right = 0;
                    Target_Speed = 0;
                    Turn_Speed = 0;
                    break;
            }
        }
        
        // 根据方向标志位计算目标速度
        if(Fore) Target_Speed -= 1;      // 前进
        if(Back) Target_Speed += 1;      // 后退
        if(Left) Turn_Speed += 30;       // 左转
        if(Right) Turn_Speed -= 30;      // 右转
        
        // 限幅
        if(Target_Speed > SPEED_Y) Target_Speed = SPEED_Y;
        if(Target_Speed < -SPEED_Y) Target_Speed = -SPEED_Y;
        
        if(Turn_Speed > SPEED_Z) Turn_Speed = SPEED_Z;
        if(Turn_Speed < -SPEED_Z) Turn_Speed = -SPEED_Z;
        
        // 转向约束（无转向指令时开启）
        if((Left == 0) && (Right == 0))
            Turn_Kd = -0.6;
        else
            Turn_Kd = 0;
    }
}

// ==================== 超声波任�?====================
void vTaskUltrasonic(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        // 调用非阻塞超声波测距
        length = Senor_Using();
        
        // 20ms执行一�?
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

// ==================== OLED显示任务 ====================
void vTaskOLED(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        // 刷新显示
        OLED_Clear();
        
        OLED_ShowString(0, 0, "Pitch:", 12);
        OLED_Float(0, 48, Pitch, 2);
        
        OLED_ShowString(0, 2, "Dist:", 12);
        OLED_ShowNumber(48, 2, length, 3, 16);
        
        OLED_ShowString(0, 4, "L:", 12);
        OLED_Num3(2, 4, Encoder_Left);
        
        OLED_ShowString(8, 4, "R:", 12);
        OLED_Num3(10, 4, Encoder_Right);
        
        OLED_ShowString(0, 6, "Target:", 12);
        OLED_Num3(7, 6, Target_Speed);
        
        // 100ms刷新一�?
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

// ==================== FreeRTOS钩子函数 ====================
void vApplicationIdleHook(void)
{
    // 空闲任务时CPU进入睡眠（省电）
    // __WFI();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // 堆栈溢出时死循环（调试用�?
    while(1);
}

void vApplicationMallocFailedHook(void)
{
    // 内存分配失败时死循环
    while(1);
}
