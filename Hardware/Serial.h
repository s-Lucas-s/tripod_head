#include "stm32f10x.h"
#include <stdio.h>

#ifndef __SERIAL_H
#define __SERIAL_H

extern uint8_t center_x,center_y,z;

void Serial_Init(void);
void USART1_IRQHandler(void);			 

#endif
