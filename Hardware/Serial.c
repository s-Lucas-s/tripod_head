#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include "PID.h"

//uint8_t Cx,Cy,Cz;
//uint8_t Serial_RxData;
//uint8_t Serial_RxFlag;


 
uint8_t center_x,center_y,z;         
                           
void Serial_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	//USART3_RX	  PB11
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//USART3_TX	  PB10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &USART_InitStructure);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART3, ENABLE);
}
void USART3_IRQHandler(void)			 
{
		u8 com_data;				  //用于读取STM32串口收到的数据，这个数据会被下一个数据掩盖，所以要将它用一个数组储存起来。	
 		u8 i;
		static u8 RxCounter1=0;
		static u16 RxBuffer1[6]={0};  //定义一个6个成员的数组，可以存放6个数据，刚好放下一个数据包。
		static u8 RxState = 0;        //接收状态，判断程序应该接收第一个帧头、第二个帧头、数据或帧尾。	
 
		if( USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)  	   //接收中断  
		{
			
				USART_ClearITPendingBit(USART3,USART_IT_RXNE);   //清除中断标志
				com_data = USART_ReceiveData(USART3);
// 当RXState处于0时，为接收帧头1模式。若接收到帧头1（0x2C），将RXState置1，切换到接收帧头2模式，并将帧头1存入RxBuffer1[0]的位置，RxCounter1加一。
				if(RxState==0&&com_data==0x2C)  //0x2c帧头
				{
					
					RxState=1;
					RxBuffer1[RxCounter1++]=com_data;
				}
		
// 当RXState处于1时，为接收帧头2模式。若接收到帧头2（0x12），将RXState置2，切换到保存数据模式，并将帧头2存入RxBuffer1[1]的位置，RxCounter1加一。
				else if(RxState==1&&com_data==0x12)  //0x12帧头
				{
					RxState=2;
					RxBuffer1[RxCounter1++]=com_data;
				}
//当RXState处于2时，为保存数据模式。RxBuffer1[]将接收到的数据依次存入RxBuffer1[2]、RxBuffer1[3]、RxBuffer1[4]、RxBuffer1[5]中。当接收到第六位数据时，进行判断是否为帧尾（0x5B），若是帧尾分别保存数据RxBuffer1[2]、RxBuffer1[3]、RxBuffer1[4]到x、y、z中
				else if(RxState==2)
				{
					RxBuffer1[RxCounter1++]=com_data;
 
					if(RxCounter1==6 && com_data == 0x5B)       //RxBuffer1接受满了,接收数据结束
					{
						
						center_x=RxBuffer1[RxCounter1-4];
						center_y=RxBuffer1[RxCounter1-3];
						z=RxBuffer1[RxCounter1-2];
						RxCounter1 = 0;
						RxState = 0;	
					}
//若是不是帧尾帧尾将会把RxState、RxCounter1和RxBuffer1[]全部置零做接收异常处理。
					else if(RxCounter1 > 6)            //接收异常
					{
						RxState = 0;
						RxCounter1=0;
						for(i=0;i<6;i++)
						{
								RxBuffer1[i]=0x00;      //将存放数据数组清零
						}
					
					}
				}
				else   //接收异常
				{
						RxState = 0;
						RxCounter1=0;
						for(i=0;i<6;i++)
						{
								RxBuffer1[i]=0x00;      //将存放数据数组清零
						}
				}
 
		}
	PID_Control(center_x,center_y);
}
