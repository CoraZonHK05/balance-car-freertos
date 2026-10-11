#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f10x.h"
#include "sys.h"

// ============ 外部变量声明 ============
extern int Encoder_Left, Encoder_Right;
extern int MOTO1, MOTO2;
extern float Pitch, Roll, Yaw;
extern short gyrox, gyroy, gyroz;
extern short aacx, aacy, aacz;
extern int length;

extern float Med_Angle;
extern float Target_Speed, Turn_Speed;
extern float Vertical_Kp, Vertical_Kd;
extern float Velocity_Kp, Velocity_Ki;
extern float Turn_Kd, Turn_Kp;
extern int Vertical_out, Velocity_out, Turn_out;

// ============ 宏定义 ============
#define SPEED_Y 15
#define SPEED_Z 80

// ============ 函数声明 ============
int Vertical(float Med, float Angle, float gyro_Y);
int Velocity(int Target, int encoder_left, int encoder_right);
int Turn(int gyro_Z, int RC);

#endif
