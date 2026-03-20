#ifndef __PID_H
#define __PID_H

#include "sys.h"

extern uint32_t Target_Vertical_x;
extern uint32_t Target_Vertical_y;
void PID_Control(int32_t xerr, int32_t yerr);

#endif

