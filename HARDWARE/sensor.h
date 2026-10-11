#ifndef __SENSOR_H_
#define __SENSOR_H_
#include "stm32f10x.h"
#include "sys.h"

#define TRIG PAout(3) //输出端口
#define ECHO PAin(2)  //输入端口
void TIM3_IRQHandler(void);
void TIM3_Int_Init(void);
int Senor_Using(void); //单精度数据类型

#endif
