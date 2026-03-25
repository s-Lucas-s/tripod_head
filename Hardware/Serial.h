#ifndef __SERIAL_H
#define __SERIAL_H

#include "sys.h"

extern float center_x, center_y; // 外部声明视觉解析的中心坐标

void Serial_Init(void);                                                                              // 串口3初始化函数
void Serial_SendByte(uint8_t Byte);                                                                  // 串口发送一个字节
void Serial_SendArray(uint8_t *Array, uint16_t Length);                                              // 串口发送数组
void Serial_SendString(char *String);                                                                // 串口发送字符串
void Serial_SendNumber(uint32_t Number, uint8_t Length);                                             // 串口发送数字
void Serial_SendPacket(uint8_t packet_header, uint8_t packet_tail, uint8_t *Array, uint16_t Length); // 串口发送数据包

#endif
