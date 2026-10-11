#ifndef  _ENCODER_H
#define  _ENCODER_H

#include "sys.h" 
#include "stm32f10x.h"

//Encoder_TIM2_Init()	初始化TIM2编码器	main函数初始化阶段
//Encoder_TIM4_Init()	初始化TIM4编码器	main函数初始化阶段


void TIM2_IRQHandler(void);
void TIM4_IRQHandler(void);
int Read_Speed(int TIMx);
void Encoder_TIM2_Init(void);
void Encoder_TIM4_Init(void);
#endif


