#include "stm32f10x.h"
#include "Timer.h"

void Timer_Init(void)															//TIM3用以定时中断
{                                                                   		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);							//打开时钟
	                                                                		
	TIM_InternalClockConfig(TIM3);												//选择内部时钟
		
	TIM_TimeBaseInitTypeDef TIM_TimerBaseInitStructure;
	TIM_TimerBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;					//选择1分频（不分频）
	TIM_TimerBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;				//计数模式：向上计数
	TIM_TimerBaseInitStructure.TIM_Period=1000-1;								//ARR（自动重装载器值）100ms
	TIM_TimerBaseInitStructure.TIM_Prescaler=7200-1;							//PSC（预分频器）
	TIM_TimerBaseInitStructure.TIM_RepetitionCounter=0;							//重复计数器（高级定时器使用）
	TIM_TimeBaseInit(TIM3,&TIM_TimerBaseInitStructure);				
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);										//清除中断标志位
			
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);									//开启TIM3的更新中断
	                                                                
	NVIC_InitTypeDef NVIC_InitStructure;                           
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;                				//配置NVIC的TIM3线
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;                				//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;      				//指定NVIC线路的抢占优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;             				//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);                              				//将结构体变量交给NVIC_Init，配置NVIC外设
	                                                                
	TIM_Cmd(TIM3,ENABLE);                                        				//使能TIM3
	                                                                
}                                                                   
                                                                    
/*中断函数*/
void TIM3_IRQHandler(void)                                          
{                                                                   
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
