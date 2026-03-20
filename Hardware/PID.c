#include "PID.h"

#define Initial_time_threshold 500
#define Integral_MAX 500000

#define ratio_y 0.8
#define ratio_x ratio_y

typedef enum
{
    x,
    y
} Dimension_t;

typedef struct parameter_pid
{
    int32_t kp;
    int32_t ki;
    int32_t kd;
} pid_t;

typedef struct Cascade_Control_Loop
{
    pid_t Speed_Loop;
    pid_t Position_Loop;
} CCL_t;

CCL_t Dimension_x;
CCL_t Dimension_y;
void PID_Init(void)
{
    Dimension_x.Position_Loop.kp = 0;
    Dimension_x.Position_Loop.ki = 0;
    Dimension_x.Position_Loop.kd = 0;

    Dimension_x.Speed_Loop.kp = 0;
    Dimension_x.Speed_Loop.ki = 0;
    Dimension_x.Speed_Loop.kd = 0;

    Dimension_y.Position_Loop.kp = 0;
    Dimension_y.Position_Loop.ki = 0;
    Dimension_y.Position_Loop.kd = 0;

    Dimension_y.Speed_Loop.kp = 0;
    Dimension_y.Speed_Loop.ki = 0;
    Dimension_y.Speed_Loop.kd = 0;
}

uint32_t Vertical_y = 0;
static uint32_t Vertical_xerr_last = 0; // 上次X轴速度位置误差
static uint32_t Vertical_yerr_last = 0; // 上次Y轴速度位置误差
uint32_t Target_Vertical_x = 0;         // X轴目标速度
uint32_t Target_Vertical_y = 0;         // Y轴目标速度

static uint32_t xerr_last = 0; // 上次X轴位置误差
static uint32_t yerr_last = 0; // 上次Y轴位置误差

static uint32_t time_count_last = 0; // 上次时间戳

static int32_t Vertical_Xerr_S = 0; // 速度环X轴积分
static int32_t Vertical_Yerr_S = 0; // 速度环Y轴积分
static int32_t Position_Xerr_S = 0; // 位置环X轴积分
static int32_t Position_Yerr_S = 0; // 位置环Y轴积分

#define Get_square(x) ((x) * (x))
#define LIMIT_VALUE_SYMMETRIC(value, max_val) \
    do                                        \
    {                                         \
        if ((value) > (max_val))              \
        {                                     \
            (value) = (max_val);              \
        }                                     \
        else if ((value) < -(max_val))        \
        {                                     \
            (value) = -(max_val);             \
        }                                     \
    } while (0)
#define LIMIT_SYMMETRIC(value) LIMIT_VALUE_SYMMETRIC(value, Integral_MAX)

int32_t Speed_Loop_Control(Dimension_t Dimension, int32_t err, uint32_t interval_time)
{
    int32_t Speed_out = 0;

    if (Dimension == x)
    {
        int32_t Vertical_Err = 0;
        int32_t Vertical_x = 0;
        Vertical_x = (err - xerr_last) / interval_time;                             // 获取速度
        Vertical_Err = Target_Vertical_x - Vertical_x;                              // 获取速度误差
        Vertical_Err = (1 - ratio_x) * Vertical_Err + ratio_x * Vertical_xerr_last; // 低通滤波
        Vertical_Xerr_S += Vertical_Err;                                            // 速度积分
        LIMIT_VALUE_SYMMETRIC(Vertical_Xerr_S);
        Speed_out = Dimension_x.Speed_Loop.kp * Vertical_Err + Dimension_x.Speed_Loop.ki * (Vertical_Xerr_S) + Dimension_x.Speed_Loop.kd * (Vertical_xerr_last - Vertical_Err); // PID计算
        Vertical_xerr_last = Vertical_Err;
    }
    else if (Dimension == y)
    {
        int32_t Vertical_Err = 0;
        int32_t Vertical_y = 0;
        Vertical_y = (err - yerr_last) / interval_time;                                                                                                                         // 获取速度
        Vertical_Err = Target_Vertical_y - Vertical_y;                                                                                                                          // 获取速度误差
        Vertical_Err = (1 - ratio_y) * Vertical_Err + ratio_y * Vertical_yerr_last;                                                                                             // 低通滤波
        Vertical_Yerr_S += Vertical_Err;                                                                                                                                        // 速度积分
        Speed_out = Dimension_y.Speed_Loop.kp * Vertical_Err + Dimension_y.Speed_Loop.ki * (Vertical_Yerr_S) + Dimension_y.Speed_Loop.kd * (Vertical_yerr_last - Vertical_Err); // PID计算
        Vertical_yerr_last = Vertical_Err;
    }

    return Speed_out;
}

int32_t Position_Loop_Control(Dimension_t Dimension, int32_t err, uint32_t interval_time)
{
    int32_t Speed_out = 0;

    if (Dimension == x)
    {
        Position_Xerr_S += err;
        Speed_out = Dimension_x.Position_Loop.kp * err + Dimension_x.Position_Loop.ki * (Vertical_Xerr_S) + Dimension_x.Position_Loop.kd * (err - xerr_last) / interval_time; // PID计算
    }
    else if (Dimension == y)
    {
        Position_Yerr_S += err;
        Speed_out = Dimension_y.Position_Loop.kp * err + Dimension_y.Position_Loop.ki * (Vertical_Yerr_S) + Dimension_y.Position_Loop.kd * (err - yerr_last) / interval_time; // PID计算
    }
    return Speed_out;
}

int32_t PID_x_Control(int32_t x_err, uint32_t interval_time)
{
    return Position_Loop_Control(x, x_err, Speed_Loop_Control(x, x_err, interval_time));
}

int32_t PID_y_Control(int32_t y_err, uint32_t interval_time)
{
    return Position_Loop_Control(y, y_err, Speed_Loop_Control(y, y_err, interval_time));
}

/**
 * @brief 位置环
 *
 * @param xerr x轴位置偏差
 * @param yerr y轴位置偏差
 * @param x_out x轴输出
 * @param y_out y轴输出
 */

void PID_Control(int32_t xerr, int32_t yerr)
{
    int32_t x_out = 0;
    int32_t y_out = 0;
    uint32_t time_count = 0;
    uint32_t interval_time = 0;
    time_count = g_timer3_count;
    interval_time = (time_count_last <= Initial_time_threshold ? time_count_last : (time_count - time_count_last)); // 获取间隔时间
    time_count_last = time_count;
    if (Stop_flag)
    {
        xerr_last = 0;
        yerr_last = 0;
        Vertical_Xerr_S = 0;
        Vertical_Yerr_S = 0;
        return;
    }

    x_out = PID_x_Control(xerr, interval_time);
    y_out = PID_y_Control(yerr, interval_time);

    if (x_out >= 0)
    {
        Emm_V5_Pos_Control(1, 0, (uint16_t)x_out, 0, 16000, 0, 1);
    }
    else if (x_out < 0)
    {
        Emm_V5_Pos_Control(1, 1, (uint16_t)(-x_out), 0, 16000, 0, 1);
    }
    if (y_out >= 0)
    {
        Emm_V5_Pos_Control(2, 0, (uint16_t)y_out, 0, 14000, 0, 1);
    }
    else if (y_out < 0)
    {
        Emm_V5_Pos_Control(2, 1, (uint16_t)(-y_out), 0, 14000, 0, 1);
    }
    Emm_V5_Synchronous_motion(0);
    xerr_last = xerr;
    yerr_last = yerr;
}
