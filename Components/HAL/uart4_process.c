/*! ----------------------------------------------------------------------------
 * @fileuart4_process.c
 * @briefUART4 接收目标距离和方位角数据，实现自主跟随功能
 *
 * @attention
 * Copyright 2025 (c) Suzhou Ailanxin electronic Technology Co., LTD.
 * All rights reserved.
 *
 * @author Yang
 ----------------------------------------------------------------------------*/

#include "uart4_process.h"
#include "hal_usart.h"
#include "motor.h"
#include "OSAL_Clock.h"
#include "dataoperation.h"
#include <string.h>
#include "Periph_init.h"

// 全局变量
TAG_TRANS_DATA TagTransData = {0};
MOTOR_DATA MotorData = {0};
u8 Uart4_Data_Valid = 0;  // 数据有效标志位：初始为 0（无效）

// UART4 接收缓冲区状态 - 完全参考 Followingcar1.2 的 USART1RX 结构
typedef union
{
    u8 ch[64];
    struct
    {
        u32 Frameheader;      // 帧头 0xFFFFFFFF
        u16 PacketLength;     // 包长度
        u16 SequenceID;       // 序列号
        u16 RequestCommand;   // 请求命令
        u16 VersionID;        // 版本 ID
        u32 AnchorID;         // 锚点 ID
        u32 TagID;            // 标签 ID
        u32 Distance;         // 距离 (需要字节交换)
        s16 Azimuth;          // 方位角 (需要字节交换)
        s16 Elevation;        // 仰角
        u16 TagStatus;        // 标签状态
        u16 BatchSn;          // 批次序列号
        u32 Reserve;          // 保留
        u8  Xor;              // 异或校验
    } item;
} UART4_RX_STRUCT;

typedef struct
{
    u8   rxstatus;        // 接收状态：0-空闲，1-接收中，2-接收完成
    u8   getcheck;        // 获取校验标志
    u8   incnum;          // 计数
    u8   inclimit;        // 限制值
    u8   end_flag;        // 结束标志
    u8   cls;             // 清屏标志
    u16  commond;         // 命令
    u16  rxnum;           // 接收数量
    u32  rxtime;          // 接收时间
    UART4_RX_STRUCT rxbuf;
} UART4_State_Struct;

UART4_State_Struct Uart4 = {0};

#define UART4_IDLE      0
#define UART4_RECVING   2
#define UART4_RECED     3
#define UART4_RX_DLY_MAX 50

/*
 *******************************************************************************
 * 驱动处理函数 - 根据距离和方位角计算电机速度值（使用闭环控制）
 *******************************************************************************
 */
void DriveProcess_New(void)
{
    // 首先检查数据有效性：只有数据有效时才执行跟随逻辑
    if(Uart4_Data_Valid == 0)
    {
        // 数据无效 - 停车
        ALL_motor_control(0, 0, 0, 0);
        return;
    }

    // 距离判断：大于跟随距离时进入待机，否则正常工作
    if(TagTransData.Distance > FOLLOW_DISTANCE || TagTransData.Distance <= 50)
    {
        // 待机模式 - 停止电机（距离太远或太近）
        //ALL_motor_control(0, 0, 0, 0);
				CAR_STOP();
			
        return;
    }

    // 跟随模式 - 计算左右轮速度
    int16_t base_speed = 50;  // 基准速度（闭环控制的目标速度值）
    int16_t azim_abs = (TagTransData.Azimuth >= 0) ? TagTransData.Azimuth : -TagTransData.Azimuth;

    // 根据方位角计算差速：角度越大，转向越明显
    // 系数 10 可根据实际转向灵敏度调整
    int16_t speed_diff = azim_abs * 10;

    int16_t left_speed = base_speed - speed_diff;
    int16_t right_speed = base_speed + speed_diff;

    // 如果方位角为负，反转差速方向
    if(TagTransData.Azimuth < 0)
    {
        left_speed = base_speed + speed_diff;
        right_speed = base_speed - speed_diff;
    }

    // 限幅保护：速度不能为负
    if(left_speed < 0) left_speed = 0;
    if(right_speed < 0) right_speed = 0;

    // 使用 SmartCart 的 ALL_motor_control 函数（闭环 PID 控制）
    // 左前=左后=left_speed，右前=右后=right_speed
    ALL_motor_control(left_speed, left_speed, right_speed, right_speed);
}

/*
 *******************************************************************************
 * 获取跟随状态
 * 返回值：0-待机（距离>150cm），1-跟随（距离<=150cm）
 *******************************************************************************
 */
u8 GetFollowingStatus(void)
{
    if(TagTransData.Distance > FOLLOW_DISTANCE)
    {
        return 0;  // 待机
    }
    else
    {
        return 1;  // 跟随
    }
}

/*
 *******************************************************************************
 * UART4 接收数据处理 - 完全参考 Followingcar1.2 的 Usart1Receive 实现
 *******************************************************************************
 */
void Uart4Receive(void)
{
    if(Uart4.rxstatus != UART4_RECED)
        return;

    // 处理命令（沿用 Followingcar1.2 的逻辑）
    DoCmd_Uart4(U16HighLowByteSwap(Uart4.rxbuf.item.RequestCommand));

    // 清除接收状态
    Uart4RxStatusClr();
}

/*
 *******************************************************************************
 * 清除接收状态
 *******************************************************************************
 */
void Uart4RxStatusClr(void)
{
    Uart4.rxstatus = UART4_IDLE;
    Uart4.getcheck = 0;
    Uart4.incnum = 0;
    Uart4.inclimit = 0;
    Uart4.commond = 0;
    Uart4.rxnum = 0;
    Uart4.end_flag = 0;
}

/*
 *******************************************************************************
 * 超时检查 - 在 SysTick 中调用
 *******************************************************************************
 */
void Uart4RxTimeOverCheck(void)
{
    if(Uart4.rxtime > 0)
    {
        Uart4.rxtime--;
        if(Uart4.rxtime == 0)
        {
            // 超时，重置接收状态
            if(Uart4.rxstatus == UART4_RECVING)
            {
                Uart4RxStatusClr();
            }
        }
    }
}

/*
 *******************************************************************************
 * 命令处理函数 - 完全参考 Followingcar1.2 的 DoCmd 实现
 *******************************************************************************
 */
void DoCmd_Uart4(u16 cmd)
{
    Uart4.end_flag = 0;

    switch(cmd)
    {
        case 0x2001:  // 距离和方位角数据命令
            // 沿用 Followingcar1.2 的字节交换方法
            TagTransData.Distance = U32HighLowByteSwap(Uart4.rxbuf.item.Distance);
            TagTransData.Azimuth = U16HighLowByteSwap(Uart4.rxbuf.item.Azimuth);

            // 更新接收时间
            TagTransData.GetTime = portGetTickCnt();

            // 【关键】只有校验通过才置有效标志
            Uart4_Data_Valid = 1;
            break;

        default:
            break;
    }
}

/*
 *******************************************************************************
 * UART4 中断服务函数 - 完全参考 Followingcar1.2 的 USART1_IRQHandler 实现
 * 需要在 stm32f10x_it.c 中调用此函数
 *******************************************************************************
 */
void UART4_IRQHandler_Wrapper(void)
{
    u16 udata;

    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        udata = (u16)(UART4->DR & (u16)0x01FF);

        if((Uart4.rxstatus == UART4_IDLE) || (Uart4.rxstatus == UART4_RECVING))
        {
            Uart4.rxbuf.ch[Uart4.rxnum++] = udata;
            Uart4.rxtime = UART4_RX_DLY_MAX;
            Uart4.rxstatus = UART4_RECVING;

            if(Uart4.getcheck)
            {
                Uart4.incnum++;
                if(Uart4.incnum == 5)
                {
                    Uart4.commond = udata;
                }

                if(Uart4.incnum == 6)
                {
                    Uart4.commond = (Uart4.commond << 8) + udata;
                    if(Uart4.commond == 0x2001)
                    {
                        Uart4.inclimit = 33;  // 0x2001 命令的总长度为 33 字节
                    }
                }

                if(Uart4.incnum == Uart4.inclimit)
                {
                    Uart4.getcheck = 0;
                    Uart4.incnum = 0;
                    Uart4.inclimit = 0;
                    Uart4.commond = 0;
                    Uart4.rxstatus = UART4_RECED;
                }
            }

            if(udata == 0xFF)
            {
                Uart4.end_flag++;
            }
            else
            {
                Uart4.end_flag = 0;
            }

            if(Uart4.end_flag >= 4)
            {
                Uart4.end_flag = 0;
                Uart4.getcheck = 1;
                Uart4.rxnum = 4;  // 帧头已接收 4 字节
            }

            if(Uart4.rxnum >= 100)
            {
                Uart4.rxstatus = UART4_RECED;
            }
        }

        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
}

/*
 *******************************************************************************
 * UART4 初始化
 *******************************************************************************
 */
void Uart4ReceiveInit(void)
{
    // UART4 已在 hal_usart.c 中初始化
    // 这里只需要使能中断
    NVIC_EnableIRQ(UART4_IRQn);
}

/*
 *******************************************************************************
 * UART4 处理主函数 - 在 NO_FULL 模式下周期性调用
 *******************************************************************************
 */
void Uart4Process(void)
{
    // 处理接收到的数据
    Uart4Receive();

    // 检查超时（200ms 无数据则清零并置无效标志）
    if((portGetTickCnt() - TagTransData.GetTime) > 200)
    {
        TagTransData.Distance = 0;
        TagTransData.Azimuth = 0;
        Uart4_Data_Valid = 0;  // 【关键】超时将标志位置 0
    }

    // 执行驱动控制
    DriveProcess_New();
}