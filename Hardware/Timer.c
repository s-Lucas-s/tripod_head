#include "Timer.h"

uint32_t g_timer3_count = 0;													//全局变量：累计即使次数（中断100ms，计数+1）

void Timer_Init(void)															//TIM3用以定时中断
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);						// 打开时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 						// 打开时钟

    TIM_InternalClockConfig(TIM2); 												// 选择内部时钟
    TIM_InternalClockConfig(TIM3); 												// 选择内部时钟

    TIM_TimeBaseInitTypeDef TIM_TimerBaseInitStructure;
	TIM_TimerBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;					//选择1分频（不分频）
	TIM_TimerBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;				//计数模式：向上计数
	TIM_TimerBaseInitStructure.TIM_Period=1000-1;								//ARR（自动重装载器值）100ms
	TIM_TimerBaseInitStructure.TIM_Prescaler=7200-1;							//PSC（预分频器）
	TIM_TimerBaseInitStructure.TIM_RepetitionCounter=0;							//重复计数器（高级定时器使用）
	TIM_TimeBaseInit(TIM2,&TIM_TimerBaseInitStructure);

    TIM_TimerBaseInitStructure.TIM_Period = 1000 - 1;    						// ARR（自动重装载器值）100ms
    TIM_TimerBaseInitStructure.TIM_Prescaler = 72 - 1; 							// PSC（预分频器）
    TIM_TimeBaseInit(TIM3, &TIM_TimerBaseInitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);                						// 清除中断标志位
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);										// 清除中断标志位

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);									// 开启TIM3的更新中断

    NVIC_InitTypeDef NVIC_InitStructure;                           
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;                				//配置NVIC的TIM3线
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;                				//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;      				//指定NVIC线路的抢占优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;             				//指定NVIC线路的响应优先级为2
    NVIC_Init(&NVIC_InitStructure);                                             // 将结构体变量交给NVIC_Init，配置NVIC外设
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;           					// 配置NVIC的TIM3线
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 					// 指定NVIC线路的抢占优先级为2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        					// 指定NVIC线路的响应优先级为1
    NVIC_Init(&NVIC_InitStructure);                           					// 将结构体变量交给NVIC_Init，配置NVIC外设

    TIM_Cmd(TIM2,ENABLE);                                        				//使能TIM2
	                                                                
}

/*中断函数*/
void TIM3_IRQHandler(void)                                          
{                                                                   
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		g_timer3_count++;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

/**
 * @brief  启动TIM3计时（主函数发送“开始计时”信号）
 * @param  无
 * @retval 无
 */
void Timer3_Start(void)
{
	g_timer3_count = 0;				//启动前计时清零
	TIM_SetCounter(TIM3,0);			//重置TIM3硬件计数器
	TIM_Cmd(TIM3,ENABLE);			//使能TIM3，开始计时
}

/**
 * @brief  读取当前累计计时时间（单位：100ms）
 * @param  无
 * @retval 无
 */
uint32_t Timer3_Read(void)
{
	return g_timer3_count;			
}

/**
 * @brief  清除TIM3计时时间
 * @param  无
 * @retval 无
 */
void Timer3_Clear(void)
{
	TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE); // 关中断防止清零被打断
	g_timer3_count = 0;				// 清零软件计时值
	TIM_SetCounter(TIM3, 0);       	// 清零硬件计数器
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);  // 重新开启中断
}

