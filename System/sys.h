#ifndef __SYS_H
#define __SYS_H

/* ——————————include—————————— */
#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "Emm_V5.h"
#include "OLED.h"
#include "PID.h"
#include "Timer.h"
#include "board.h"
#include "usart.h"
#include "Serial.h"
#include "Serial1.h"
#include "Key.h"

/* ————————————变量———————————— */
extern bool Stop_flag;
extern int8_t Questionx;
extern bool Power_on_flag;

/* —————————————宏————————————— */
#define ABS(x)      ((x) >= 0 ? (x) : -(x))
#define Max_x_angle 180

/* ————————————函数———————————— */
void nvic_init(void);
float Check_angle(uint8_t addr);

#endif
