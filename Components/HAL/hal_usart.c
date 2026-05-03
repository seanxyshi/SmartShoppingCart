#include "hal_usart.h"

#define DMA1_MEM_LEN 256
char _dbg_TXBuff[DMA1_MEM_LEN];

uint16_t USART1_SendBuffer(const char* buffer, uint16_t length)
{
if( (buffer==NULL) || (length==0) )
{
return 0;
}

DMA_Cmd(DMA1_Channel4, DISABLE);
DMA_SetCurrDataCounter(DMA1_Channel4, length);
DMA_Cmd(DMA1_Channel4, ENABLE);
while(1)
{
if(DMA_GetITStatus(DMA1_IT_TC4)!=RESET)
{
DMA_ClearFlag(DMA1_IT_TC4);
break;
}
}
return length;
}

void _dbg_printf(const char *format,...)
{
uint32_t length;
va_list args;

va_start(args, format);
length = vsnprintf((char*)_dbg_TXBuff, sizeof(_dbg_TXBuff), (char*)format, args);
va_end(args);
USART1_SendBuffer((const char*)_dbg_TXBuff,length);
}

void HalUASRT1_DMA_Send_init(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar)
{
DMA_InitTypeDef DMA_InitStructure;

RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  DMA_DeInit(DMA_CHx);
DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;
DMA_InitStructure.DMA_MemoryBaseAddr = cmar;
DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
DMA_InitStructure.DMA_BufferSize = DMA1_MEM_LEN;
DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
DMA_InitStructure.DMA_Priority = DMA_Priority_High;
DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
DMA_Init(DMA_CHx, &DMA_InitStructure);
USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
}

void HalUASRT1_DMA_Config(void)
{
USART_ITConfig(USART1, USART_IT_TC,DISABLE);
USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

HalUASRT1_DMA_Send_init(DMA1_Channel4,(u32)&USART1->DR,(u32)_dbg_TXBuff);
}

void HalUSART1_Init(u32 bound)
{
USART_InitTypeDef USART_InitStructure;

USART_InitStructure.USART_BaudRate = bound;
USART_InitStructure.USART_WordLength = USART_WordLength_8b;
USART_InitStructure.USART_StopBits = USART_StopBits_1;
USART_InitStructure.USART_Parity = USART_Parity_No;
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

USART_Init(USART1, &USART_InitStructure);
USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
USART_Cmd(USART1, ENABLE);
}

void HalUSART1_IO_Init(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void HalUASRT2_NVIC_Config(void)
{
NVIC_InitTypeDef NVIC_InitStructure;
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
}

void HalUSART2_Init(u32 bound)
{
USART_InitTypeDef USART_InitStructure;

USART_InitStructure.USART_BaudRate = bound;
USART_InitStructure.USART_WordLength = USART_WordLength_8b;
USART_InitStructure.USART_StopBits = USART_StopBits_1;
USART_InitStructure.USART_Parity = USART_Parity_No;
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

USART_Init(USART2, &USART_InitStructure);
USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
USART_Cmd(USART2, ENABLE);
}

void HalUSART2_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void HalUASRT4_NVIC_Config(void)
{
NVIC_InitTypeDef NVIC_InitStructure;
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
}

void HalUSART4_Init(u32 bound)
{
USART_InitTypeDef USART_InitStructure;

USART_InitStructure.USART_BaudRate = bound;
USART_InitStructure.USART_WordLength = USART_WordLength_8b;
USART_InitStructure.USART_StopBits = USART_StopBits_1;
USART_InitStructure.USART_Parity = USART_Parity_No;
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

USART_Init(UART4, &USART_InitStructure);
USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
USART_Cmd(UART4, ENABLE);
}

void HalUSART4_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void HalUARTInit ( void )
{
HalUSART1_IO_Init();
HalUSART1_Init(115200);
HalUASRT1_DMA_Config();

HalUSART2_IO_Init();
HalUSART2_Init(115200);
HalUASRT2_NVIC_Config();

HalUSART4_IO_Init();
HalUSART4_Init(115200);
HalUASRT4_NVIC_Config();
}