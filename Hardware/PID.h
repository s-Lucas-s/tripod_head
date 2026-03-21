#ifndef __PID_H
#define __PID_H

#include "sys.h"

// 坐标结构体定义
typedef struct
{
    float x; // 横坐标
    float y; // 纵坐标
} Point2D;

extern uint32_t Target_Vertical_x; // X轴目标速度
extern uint32_t Target_Vertical_y; // Y轴目标速度

void PID_Init(void);
void PID_Control(int32_t xerr, int32_t yerr);
int calcHomography(Point2D src[4], Point2D dst[4], float H[3][3]);
void visualToReal(float H[3][3], Point2D pixelPt, Point2D *screenPt);

#endif
