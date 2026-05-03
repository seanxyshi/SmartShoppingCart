#include "stm32f10x.h"
#include "delay.h"
#include "Periph_init.h"
#include "timer.h"
#include "hal_usart.h"
#include "control.h"
#include "uwb.h"
#include "hal_iic.h"
#include "oled_i2c.h"
#include "motor.h"


/*******************************************************************************
*******************************************************************************/
void RCC_Configuration_part(void)
{
        ErrorStatus HSEStartUpStatus;
        RCC_ClocksTypeDef RCC_ClockFreq;

        /* 将 RCC 寄存器重新设置为默认值 */
        RCC_DeInit();

        /* 打开外部高速时钟晶振 HSE */
        RCC_HSEConfig(RCC_HSE_ON);

        /* 等待外部高速时钟晶振工作 */
        HSEStartUpStatus = RCC_WaitForHSEStartUp();

        if(HSEStartUpStatus != ERROR)
        {
                /* 开启 Flash 预读缓冲功能，时钟起振后使用 */
                FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

                /* 48~72Mhz 推荐 Latency 为 2 */
                FLASH_SetLatency(FLASH_Latency_2);

                /* 设置 AHB 时钟，72MHz HCLK = SYSCLK */
                RCC_HCLKConfig(RCC_SYSCLK_Div1);
                /* 设置告诉 APB2 时钟，1 分频 72MHz PCLK2 = HCLK */
                RCC_PCLK2Config(RCC_HCLK_Div1);
                /* 设置低速 APB1 时钟，2 分频 36MHz PCLK1 = HCLK/2 */
                RCC_PCLK1Config(RCC_HCLK_Div2);
                /*  设置 ADC 时钟 ADCCLK = PCLK2/4 */
                RCC_ADCCLKConfig(RCC_PCLK2_Div6);

                //设置 PLL 时钟源及倍频系数 不分频：RCC_PLLSource_HSE_Div1 9 倍频：RCC_PLLMul_9
                RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
                /* 打开 PLL */
                RCC_PLLCmd(ENABLE);
                /* 等待 PLL 稳定工作 */
                while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}

                /* 选择 PLL 时钟作为时钟源 */
                RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

                /* 等待时钟源切换，进入稳定状态 */
                while (RCC_GetSYSCLKSource() != 0x08){}
        }

        RCC_GetClocksFreq(&RCC_ClockFreq);
}

/*******************************************************************************
* 函数名  : init
* 描述    : 初始化函数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
*******************************************************************************/
void init(void)
{
        //初始化设置
        SystemInit();
        RCC_Configuration_part();
        SysTick_Init();      //嘀嗒定时
        Periph_init();       //指示灯 + 蜂鸣器初始化
        Timer_Init();        //定时器初始化设置
        Uart_Queue_Init();   //PDOA 数据队列初始化
        HalUARTInit();       //串口 1+ 串口 2 设置
        OLED_Configuration();//屏幕初始化与初始打印
        EXTI_ALL_Init();     //编码器 IO 初始化
        Motor_Gpio_init();   //马达电机 IO 口初始化
}


/*******************************************************************************
* 函数名  : main
* 描述    : 主函数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
********************************************************************************/
int main(void)
{
        init();
        while(1)
        {
                Read_AoA_Control();
        }
}