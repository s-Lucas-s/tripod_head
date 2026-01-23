#ifndef __Timer_H
#define __Timer_H
#include "stm32f10x.h"

extern uint32_t g_timer3_count;

void Timer_Init(void);
void Timer3_Start(void);
uint32_t Timer3_Read(void);
void Timer3_Clear(void);

#endif

