#ifndef __KEY_H
#define __KEY_H
#include "sys.h"

void Key_Init(void);
unsigned char Key_GetCode(void);
void Key_LoopDetect(void);

#endif
