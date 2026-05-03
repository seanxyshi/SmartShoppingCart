#ifndef _SMARTCART_UART4_PROCESS_H
#define _SMARTCART_UART4_PROCESS_H

#include "stm32f10x.h"

#define FOLLOW_DISTANCE  (300)   // 跟随距离阈值（cm）

// 目标数据结构
typedef struct
{
    u32 Distance;      // 距离（cm）
    s16 Azimuth;       // 方位角（度）
    u32 GetTime;       // 最后接收时间
} TAG_TRANS_DATA;

extern TAG_TRANS_DATA TagTransData;

// 数据有效标志位：1-数据有效且校验通过，0-无效或超时
extern u8 Uart4_Data_Valid;

// 电机数据结构
typedef struct
{
    u16 PWML;          // 左轮 PWM
    u16 PWMR;          // 右轮 PWM
    u16 Velocity;      // 基准速度
    u16 Dis;           // 转向系数
    float Vcc;         // 电池电压
} MOTOR_DATA;

extern MOTOR_DATA MotorData;

// 函数声明
void Uart4Process(void);
void Uart4ReceiveInit(void);
void DriveProcess_New(void);
u8 GetFollowingStatus(void);  // 获取跟随状态：0-待机，1-跟随
void DoCmd_Uart4(u16 cmd);    // 命令处理函数
void Uart4Receive(void);      // UART4 接收数据处理
void Uart4RxStatusClr(void);  // 清除接收状态
void Uart4RxTimeOverCheck(void);  // 超时检查（在 SysTick 中调用）
void UART4_IRQHandler_Wrapper(void);  // UART4 中断处理包装函数

#endif  // _SMARTCART_UART4_PROCESS_H