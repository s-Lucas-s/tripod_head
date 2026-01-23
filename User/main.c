#include "Emm_V5.h"
#include "OLED.h"
#include "Timer.h"
#include "board.h"
#include "Delay.h"
#include "stm32f10x.h" // Device header
#include "usart.h"

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

// 定义实时位置全局变量
float pos = 0.0f, Motor_Cur_Pos = 0.0f;//pos存储从驱动器读取的原始位置数值，Motor_Cur_Pos转换后的电机当前位置（角度值）

/**
 *	@brief		MAIN函数
 *	@param		无
 *	@retval		无
 */

int main(void)
{
    OLED_Init();
    //Timer_Init();
    // 初始化板载外设
    board_init();

    // 位置模式：方向CW，速度1000RPM，加速度0（不使用加减速直接启动），脉冲数3200（16细分下发送3200个脉冲电机转一圈），相对运动
    Emm_V5_Vel_Control(1, 0, 1000, 0, 0);

    // 等待返回命令，命令数据缓存在数组rxCmd上，长度为rxCount
    while (rxFrameFlag == false)
        ;
    rxFrameFlag = false;

    // 延时2秒，等待运动完成
    delay_ms(2000);
    // 设置当前位置为单圈回零的零点位置，并存储到芯片上
    Emm_V5_Origin_Set_O(1, 1);

    // 读取电机实时位置
    Emm_V5_Read_Sys_Params(1, S_CPOS);

    // 等待返回命令，命令数据缓存在数组rxCmd上，长度为rxCount
    while (rxFrameFlag == false)
        ;
    rxFrameFlag = false;

    // 校验地址、功能码、返回数据长度，校验成功则计算当前位置角度
    if (rxCmd[0] == 1 && rxCmd[1] == 0x36 && rxCount == 8)
    {
        // 拼接成uint32_t类型
        pos = (uint32_t)(((uint32_t)rxCmd[3] << 24) |
                         ((uint32_t)rxCmd[4] << 16) |
                         ((uint32_t)rxCmd[5] << 8) |
                         ((uint32_t)rxCmd[6] << 0));

        // 转换成角度
        Motor_Cur_Pos = (float)pos * 360.0f / 65536.0f;

        // 符号
        if (rxCmd[2])
        {
            Motor_Cur_Pos = -Motor_Cur_Pos;
        }
    }

    while (1)
    {
    }
}
