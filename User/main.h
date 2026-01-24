#ifndef __MAIN_H
#define __MAIN_H

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

/* ————————————变量———————————— */
extern bool Stop_flag;

/* —————————————宏————————————— */
#define Wait(x)                       \
    do {                              \
        while (rxFrameFlag == false); \
        rxFrameFlag = false;          \
        delay_ms(x);                  \
    } while (0)
#define ABS(x)      ((x) >= 0 ? (x) : -(x))
#define Max_x_angle 180

/* ————————————函数———————————— */
float Check_angle(uint8_t addr);

#endif
