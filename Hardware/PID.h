#ifndef __PID_H
#define __PID_H

#include "main.h"
void PID_Control(int16_t xerr, int16_t yerr);
void Vertical_out(int32_t *x_out, int32_t *y_out);

#endif

