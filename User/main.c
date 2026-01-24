#include "main.h"
/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/
bool Stop_flag = 0;

/**
 *	@brief		MAIN函数
 *	@param		无
 *	@retval		无
 */

int main(void)
{
    float x_angle = 0;
    float y_angle = 0;
    board_init();
    OLED_Init();
    Timer_Init();
    Timer3_Start();
    OLED_ShowString(0, 0, "Holle!", OLED_8X16);
    OLED_Update();

    while (1)
    {
        x_angle = Check_angle(1);
        y_angle = Check_angle(2);

        if (x_angle > ABS(Max_x_angle) || y_angle > ABS(Max_x_angle))
        {
            Emm_V5_Stop_Now(0, 0);
            // Wait(20);
            Stop_flag = 1;
        }
        OLED_ShowFloatNum(0, 0, x_angle, 3, 3, OLED_8X16);
        OLED_ShowFloatNum(0, 16, y_angle, 3, 3, OLED_8X16);
        OLED_Update();
    }
}

#ifdef __ARMCC_VERSION
#pragma diag_suppress = 69
#endif
static int32_t x_out = 0;
static int32_t y_out = 0;
/*中断函数*/
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        if (!Stop_flag)
        {
            Vertical_out(&x_out, &y_out);
            if (x_out >= 0)
            {
                Emm_V5_Pos_Control(1, 0, (uint16_t)x_out, 0, 16000, 0, 1);
                Wait(0);
            }
            else if (x_out < 0)
            {
                Emm_V5_Pos_Control(1, 1, (uint16_t)(-x_out), 0, 16000, 0, 1);
                Wait(0);
            }
            if (y_out >= 0)
            {
                Emm_V5_Pos_Control(2, 0, (uint16_t)y_out, 0, 14000, 0, 1);
                Wait(0);
            }
            else if (y_out < 0)
            {
                Emm_V5_Pos_Control(2, 1, (uint16_t)(-y_out), 0, 14000, 0, 1);
                Wait(0);
            }
            Emm_V5_Synchronous_motion(0);
        }

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
#ifdef __ARMCC_VERSION
#pragma diag_default = 69
#endif

float Check_angle(uint8_t addr)
{
    float Check_pos = 0.0f, Check_Motor_Cur_Pos = 0.0f;

    Emm_V5_Read_Sys_Params(addr, S_CPOS);
    // Wait(20);
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
