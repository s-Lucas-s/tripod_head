#include "sys.h"

bool Stop_flag = 0;
bool Power_on_flag = 0;
int8_t Questionx=0;



/**
 *	@brief		MAIN函数
 *	@param		无
 *	@retval		无
 */

int main(void)
{
    // float x_angle = 0;
    // float y_angle = 0;
    nvic_init();
    PID_Init();
    board_init();
    Key_Init();
    OLED_Init();
    Serial_Init();
    Timer_Init();
    OLED_ShowString(0, 0, "Holle!", OLED_8X16);
    OLED_Update();

    while (1)
    {
        /*  x_angle = Check_angle(1);
         y_angle = Check_angle(2);

         if (x_angle > ABS(Max_x_angle) || y_angle > ABS(Max_x_angle) || Key_GetCode() == 1)
         {
             Emm_V5_Stop_Now(0, 0);
             Stop_flag = 1;
         } */
        /* OLED_ShowFloatNum(0, 0, x_angle, 3, 3, OLED_8X16);
        OLED_ShowFloatNum(0, 16, y_angle, 3, 3, OLED_8X16); */
        OLED_ShowFloatNum(0, 0, center_x, 3, 3, OLED_8X16);
        OLED_ShowFloatNum(0, 16, center_y, 3, 3, OLED_8X16);
        OLED_Update();
    }
}
/*
#ifdef __ARMCC_VERSION
#pragma diag_suppress = 69
#endif*/
/*中断函数*/
/* void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
} */

/*
#ifdef __ARMCC_VERSION
#pragma diag_default = 69
#endif*/

float Check_angle(uint8_t addr)
{
    float Check_pos = 0.0f, Check_Motor_Cur_Pos = 0.0f;

    Emm_V5_Read_Sys_Params(addr, S_CPOS);
    delay_ms(20);
    if (rxCmd[0] == addr && rxCmd[1] == 0x36 && rxCount == 8)
    {
        // 拼接成uint32_t类型
        Check_pos = (uint32_t)(((uint32_t)rxCmd[3] << 24) |
                               ((uint32_t)rxCmd[4] << 16) |
                               ((uint32_t)rxCmd[5] << 8) |
                               ((uint32_t)rxCmd[6] << 0));

        // 转换成角度
        Check_Motor_Cur_Pos = (float)Check_pos * 360.0f / 65536.0f;

        // 符号
        if (rxCmd[2])
        {
            Check_Motor_Cur_Pos = -Check_Motor_Cur_Pos;
        }
    }
    return Check_Motor_Cur_Pos;
}
