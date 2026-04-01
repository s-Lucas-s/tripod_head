#ifndef __SERIAL1_H
#define __SERIAL1_H

#include "sys.h"

// 函数声明
void Serial1_Init(void);
void Serial1_SendByte(uint8_t Byte);
void Serial1_SendArray(uint8_t *Array, uint16_t Length);
void Serial1_SendString(char *String);
void VOFA_Send3Ch(float ch1, float ch2, float ch3);

#endif
