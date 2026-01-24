#include "PID.h"
#include "Timer.h"
#include "stm32f10x.h"

#define x_Kp 1
#define x_Kd 1

#define y_Kp 1
#define y_Kd 1

#define L 1//调参
#define Get_square(x) x*x

static int64_t Vertical_x_last = 0;
static int64_t Vertical_y_last = 0;

/**
 * @brief 位置环
 *
 * @param xerr x轴位置偏差
 * @param yerr y轴位置偏差
 * @param x_out x轴输出
 * @param y_out y轴输出
 */

void PID_Control(int16_t xerr, int16_t yerr, int64_t *x_out, int64_t *y_out)
{

    static int64_t v_x_out = 0;
    int64_t v_x_temp = 0;

    static int64_t v_y_out = 0;
    int64_t v_y_temp = 0;

    v_x_temp = v_x_out;
    v_x_out = x_Kp * xerr + x_Kd * Vertical_x_last;
    Vertical_x_last = v_x_temp;


    v_y_temp = v_y_out;
    v_y_out = y_Kp * xerr + y_Kd * Vertical_y_last;
    Vertical_y_last = v_y_temp;

    *x_out = v_x_out;
    *y_out = v_y_out;
    Timer3_Clear();
}

/**
 * @brief 速度校准
 *
 * @param x_out x轴输出
 * @param y_outy轴输出
 * 
 * @note 为了让光标速度在幕布上是匀速的，需要对角速度做函数变化，遵循
 *        w(t)=(v_0*L)/(L^2+v_0^2*t^2 )
 */

void Vertical_out(int32_t *x_out, int32_t *y_out)
{
    uint32_t time_interval = 0;
    time_interval=Timer3_Read();
    *x_out = (Vertical_x_last * L) / (Get_square(L) + Get_square(Vertical_x_last * time_interval));

    *y_out = (Vertical_y_last * L) / (Get_square(L) + Get_square(Vertical_x_last * time_interval));
}


