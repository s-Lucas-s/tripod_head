#include "sys.h"

bool Stop_flag = 0;
bool Power_on_flag = 0;
int8_t Questionx = 0;

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
    Serial_SendPacket(0xA5,0x5A,(uint8_t*)&RESET_KEY,1); // 串口发送数据包
    OLED_ShowString(0, 0, "Holle!", OLED_8X16);
    OLED_Update();
    

    while (1)
    {
			
			//Delay_ms(100);
        Emm_V5_Vel_Control(1, 0, 400, 200, 0);
			//Delay_ms(100);
        Emm_V5_Vel_Control(2, 0, 400, 200, 0);
			Delay_s(5);
			while(1);
        uint8_t keyValue = 0;
        Key_LoopDetect();
        keyValue = Key_GetCode();// 获取单击按键值

          // KEY1按下：mode递增，1-4循环
            if(keyValue == 1)
            {
                Questionx++;
                if(Questionx > 4)  // 超过4则重置为1
                {
                    Questionx = 1;
                }
            }
            // KEY2按下：Stop_Flag翻转(0变1，1变0)
            else if(keyValue == 2)
            {
                Stop_flag = !Stop_flag;
            }
            

        /*  x_angle = Check_angle(1);
         y_angle = Check_angle(2);

         if (x_angle > ABS(Max_x_angle) || y_angle > ABS(Max_x_angle) || Key_GetCode() == 1)
         {
             Emm_V5_Stop_Now(0, 0);
             Stop_flag = 1;
         } */
        /* OLED_ShowFloatNum(0, 0, x_angle, 3, 3, OLED_8X16);
        OLED_ShowFloatNum(0, 16, y_angle, 3, 3, OLED_8X16); */
        //OLED_ShowFloatNum(0,16, center_x, 3, 3, OLED_8X16);
        //OLED_ShowFloatNum(0, 32, center_y, 3, 3, OLED_8X16);
        OLED_ShowString(0, 0, "Mode:", OLED_8X16);
        OLED_ShowNum(48, 0, Questionx, 1, OLED_8X16);
        OLED_ShowString(0, 16, "Stop:", OLED_8X16);
        OLED_ShowNum(48, 16, Stop_flag, 1, OLED_8X16);
        OLED_Update();
    }
}
/*
#ifdef __ARMCC_VERSION
#pragma diag_suppress = 69
#endif*/
/*中断函数*/
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

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
