#include "PID.h"
#include "Timer.h"
#include "stm32f10x.h"

typedef struct parameter_pid
{
    int32_t kp;
    int32_t ki;
    int32_t kd;
} pid_t;

struct dimensionality
{
    pid_t x;
    pid_t y;
};

#define Get_square(x) x *x

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

void PID_Control(int16_t xerr, int16_t yerr)
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

    Timer3_Clear();
}
