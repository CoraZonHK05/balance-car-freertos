#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"
#include "stm32f10x.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK MiniSTM32F103开发板
//MPU6050 IIC驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/4/18
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
	   		   
//IO方向设置
//这是直接操作STM32的端口配置寄存器低寄存器（CRL），把PB3在输入和输出模式间来回切换。

/*
GPIOB->CRL：控制PB0-PB7的配置寄存器（每4位控制一个引脚）

0XFFFF0FFF：这是一个掩码，作用是“清零”PB3对应的4位（第12-15位），但保持其他引脚不变。

3<<12：值3代表推挽输出模式，50MHz，左移12位后写入PB3的配置位，设置为输出。

8<<12：值8（二进制1000）代表输入模式（浮空输入），左移12位后写入，设置为输入。

这个过程就像是在极短时间内把PB3这个“角色”从发送者切换到接收者。


*/

#define MPU_SDA_IN()   {GPIOB->CRL&=0XFFFF0FFF;GPIOB->CRL|=8<<12;}//PB3切到浮空输入，准备读取MPU6050发来的应答信号或数据。
#define MPU_SDA_OUT()  {GPIOB->CRL&=0XFFFF0FFF;GPIOB->CRL|=3<<12;}//PB3切到推挽输出，准备向MPU6050发送数据（如地址、寄存器号）。


//#define SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<4;}
//#define SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<4;}

//IO操作函数	 
#define MPU_IIC_SCL     PBout(4) //SCL SCL（时钟线）：接在 PB4，只用作输出
#define MPU_IIC_SDA     PBout(3) //SDA	SDA（数据线）：接在 PB3，双向复用（输出时用PBout(3)，输入时用PBin(3)）
#define MPU_READ_SDA   	PBin(3) //SDA	

//#define IIC_SCL    PBout(8) //SCL 
//#define IIC_SDA    PBout(9) //SDA	 
//#define READ_SDA   PBin(9)  //输入SDA 



//IIC所有操作函数
void MPU_IIC_Delay(void);				//MPU IIC延时函数
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);				//发送IIC开始信号
void MPU_IIC_Stop(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_IIC_Ack(void);					//IIC发送ACK信号
void MPU_IIC_NAck(void);				//IIC不发送ACK信号

void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















