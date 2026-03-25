#include "PID.h"
#include <math.h>

#define Initial_time_threshold 500 // 初始时间阈值
#define Integral_MAX 500000        // 积分限幅最大值

#define ratio_y 0.8     // Y轴低通滤波系数
#define ratio_x ratio_y // X轴低通滤波系数

// 维度枚举类型（X轴和Y轴）
typedef enum
{
    x,
    y
} Dimension_t;

// PID参数结构体
typedef struct parameter_pid
{
    int32_t kp; // 比例系数
    int32_t ki; // 积分系数
    int32_t kd; // 微分系数
} pid_t;

// 串级控制环结构体（包含速度环和位置环）
typedef struct Cascade_Control_Loop
{
    pid_t Speed_Loop;    // 速度环PID参数
    pid_t Position_Loop; // 位置环PID参数
} CCL_t;

CCL_t Dimension_x; // X轴控制环
CCL_t Dimension_y; // Y轴控制环

/**
 * @brief 初始化PID参数
 */
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

/**
 * @brief 速度环控制计算
 *
 * @param Dimension 控制维度（x 或 y）
 * @param err 当前位置偏差
 * @param interval_time 离上次计算的间隔时间
 * @return int32_t 速度环的输出值
 */
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
        LIMIT_SYMMETRIC(Vertical_Xerr_S);
        Speed_out = Dimension_x.Speed_Loop.kp * Vertical_Err + Dimension_x.Speed_Loop.ki * (Vertical_Xerr_S) + Dimension_x.Speed_Loop.kd * (Vertical_xerr_last - Vertical_Err); // PID计算
        Vertical_xerr_last = Vertical_Err;
    }
    else if (Dimension == y)
    {
        int32_t Vertical_Err = 0;
        int32_t Vertical_y = 0;
        Vertical_y = (err - yerr_last) / interval_time;                             // 获取速度
        Vertical_Err = Target_Vertical_y - Vertical_y;                              // 获取速度误差
        Vertical_Err = (1 - ratio_y) * Vertical_Err + ratio_y * Vertical_yerr_last; // 低通滤波
        Vertical_Yerr_S += Vertical_Err;                                            // 速度积分
        LIMIT_SYMMETRIC(Vertical_Yerr_S);
        Speed_out = Dimension_y.Speed_Loop.kp * Vertical_Err + Dimension_y.Speed_Loop.ki * (Vertical_Yerr_S) + Dimension_y.Speed_Loop.kd * (Vertical_yerr_last - Vertical_Err); // PID计算
        Vertical_yerr_last = Vertical_Err;
    }

    return Speed_out;
}

/**
 * @brief 位置环控制计算
 *
 * @param Dimension 控制维度（x 或 y）
 * @param err 当前位置偏差
 * @param interval_time 离上次计算的间隔时间
 * @return int32_t 位置环的输出值
 */
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

/**
 * @brief X轴串级PID控制
 *
 * @param x_err X轴位置偏差
 * @param interval_time 间隔时间
 * @return int32_t X轴最终控制输出
 */
int32_t PID_x_Control(int32_t x_err, uint32_t interval_time)
{
    return Position_Loop_Control(x, x_err+Speed_Loop_Control(x, x_err, interval_time),interval_time );
}

/**
 * @brief Y轴串级PID控制
 *
 * @param y_err Y轴位置偏差
 * @param interval_time 间隔时间
 * @return int32_t Y轴最终控制输出
 */
int32_t PID_y_Control(int32_t y_err, uint32_t interval_time)
{
    return Position_Loop_Control(y, y_err+Speed_Loop_Control(y, y_err, interval_time),interval_time );
}

/**
 * @brief 整体PID控制函数（串级控制），计算各轴控制输出并控制电机运动
 *
 * @param xerr X轴位置偏差
 * @param yerr Y轴位置偏差
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

// ====================== 核心函数1：计算透视变换矩阵H ======================
/**
 * @brief 计算透视变换矩阵（单应性矩阵H） - 针对 STM32F103 的 Hartley 归一化优化版
 * @param src 输入：照片中屏幕四个角的光电坐标（必须是顺时针闭环）
 * @param dst 输入：屏幕实际四个角的相对坐标（必须和src的顺序完全一致）
 * @param H   输出：3x3的透视变换矩阵（H[2][2]=1.0f，尺度归一化）
 * @return 0:计算成功, -1:计算失败（四点共线/坐标错误/奇异矩阵）
 */
int calcHomography(Point2D src[4], Point2D dst[4], float H[3][3])
{
    static float A[8][9];
    int i, j;
    float cx_src = 0.0f, cy_src = 0.0f, cx_dst = 0.0f, cy_dst = 0.0f;
    float avg_dist_src = 0.0f, avg_dist_dst = 0.0f;
    float s_src, s_dst;
    const float SQRT2 = 1.41421356f;

    // 1. 静态数组清零 (使用循环以策安全，也可用 memset(A, 0, sizeof(A)))
    for (i = 0; i < 8; i++) {
        for (j = 0; j < 9; j++) {
            A[i][j] = 0.0f;
        }
    }

    // 2. Hartley 归一化：求质心
    for (i = 0; i < 4; i++) {
        cx_src += src[i].x; cy_src += src[i].y;
        cx_dst += dst[i].x; cy_dst += dst[i].y;
    }
    cx_src *= 0.25f; cy_src *= 0.25f;
    cx_dst *= 0.25f; cy_dst *= 0.25f;

    // 3. Hartley 归一化：求平均绝对距离并计算缩放因子
    for (i = 0; i < 4; i++) {
        float dx_s = src[i].x - cx_src; float dy_s = src[i].y - cy_src;
        float dx_d = dst[i].x - cx_dst; float dy_d = dst[i].y - cy_dst;
        avg_dist_src += sqrtf(dx_s * dx_s + dy_s * dy_s);
        avg_dist_dst += sqrtf(dx_d * dx_d + dy_d * dy_d);
    }
    avg_dist_src *= 0.25f;
    avg_dist_dst *= 0.25f;

    // 放宽阈值匹配单精度浮点特征
    if (avg_dist_src < 1e-6f || avg_dist_dst < 1e-6f) return -1;

    s_src = SQRT2 / avg_dist_src;
    s_dst = SQRT2 / avg_dist_dst;

    // 4. 用归一化后的点构建 8x9 矩阵 A
    for (i = 0; i < 4; i++) {
        float x = (src[i].x - cx_src) * s_src;
        float y = (src[i].y - cy_src) * s_src;
        float u = (dst[i].x - cx_dst) * s_dst;
        float v = (dst[i].y - cy_dst) * s_dst;

        A[2 * i][0] = x;     A[2 * i][1] = y;     A[2 * i][2] = 1.0f;
        A[2 * i][6] = -u * x;A[2 * i][7] = -u * y;A[2 * i][8] = -u;

        A[2 * i + 1][3] = x; A[2 * i + 1][4] = y; A[2 * i + 1][5] = 1.0f;
        A[2 * i + 1][6] = -v * x;A[2 * i + 1][7] = -v * y;A[2 * i + 1][8] = -v;
    }

    // 5. 高斯-约当消元提取 H_norm
    for (int col = 0; col < 8; col++) {
        float maxVal = fabs(A[col][col]);
        int maxRow = col;
        for (i = col + 1; i < 8; i++) {
            if (fabs(A[i][col]) > maxVal) {
                maxVal = fabs(A[i][col]);
                maxRow = i;
            }
        }

        if (maxRow != col) {
            float temp;
            for (j = col; j < 9; j++) {
                temp = A[col][j]; A[col][j] = A[maxRow][j]; A[maxRow][j] = temp;
            }
        }

        float div = A[col][col];
        if (fabs(div) < 1e-6f) return -1;

        float inv_div = 1.0f / div;
        for (j = col; j < 9; j++) A[col][j] *= inv_div;

        for (i = 0; i < 8; i++) {
            if (i != col && fabs(A[i][col]) > 1e-6f) {
                float factor = A[i][col];
                for (j = col; j < 9; j++) A[i][j] -= factor * A[col][j];
            }
        }
    }

    // 6. 稀疏矩阵连乘展开反归一化: H = T_dst^{-1} * H_norm * T_src
    float h00 = -A[0][8], h01 = -A[1][8], h02 = -A[2][8];
    float h10 = -A[3][8], h11 = -A[4][8], h12 = -A[5][8];
    float h20 = -A[6][8], h21 = -A[7][8]; // h22 隐含等于 1.0f

    // M = H_norm * T_src
    float M00 = h00 * s_src;
    float M01 = h01 * s_src;
    float M02 = h02 - M00 * cx_src - M01 * cy_src;

    float M10 = h10 * s_src;
    float M11 = h11 * s_src;
    float M12 = h12 - M10 * cx_src - M11 * cy_src;

    float M20 = h20 * s_src;
    float M21 = h21 * s_src;
    float M22 = 1.0f - M20 * cx_src - M21 * cy_src;

    // H = T_dst^{-1} * M
    if (fabs(M22) < 1e-6f) return -1;
    // 提前计算总归一化综合因子：(1/s_dst) 与 M22 的倒数
    float inv_s_dst = 1.0f / s_dst;
    float inv_H22 = 1.0f / M22;

    H[0][0] = (inv_s_dst * M00 + cx_dst * M20) * inv_H22;
    H[0][1] = (inv_s_dst * M01 + cx_dst * M21) * inv_H22;
    H[0][2] = (inv_s_dst * M02 + cx_dst * M22) * inv_H22;

    H[1][0] = (inv_s_dst * M10 + cy_dst * M20) * inv_H22;
    H[1][1] = (inv_s_dst * M11 + cy_dst * M21) * inv_H22;
    H[1][2] = (inv_s_dst * M12 + cy_dst * M22) * inv_H22;

    H[2][0] = M20 * inv_H22;
    H[2][1] = M21 * inv_H22;
    H[2][2] = 1.0f; // 已强制尺度归一

    return 0;
}

// ====================== 核心函数2：视觉坐标转屏幕坐标 ======================
/**
 * @brief 视觉坐标转屏幕相对坐标（严格左乘H矩阵）
 * @param H       输入：calcHomography算出的3x3变换矩阵
 * @param pixelPt 输入：照片中的光电坐标
 * @param screenPt 输出：转换后的屏幕相对坐标
 */
void visualToReal(float H[3][3], Point2D pixelPt, Point2D *screenPt)
{
    float x = pixelPt.x, y = pixelPt.y;

    float u_prime = H[0][0] * x + H[0][1] * y + H[0][2];
    float v_prime = H[1][0] * x + H[1][1] * y + H[1][2];
    float w       = H[2][0] * x + H[2][1] * y + H[2][2];

    if (fabs(w) < 1e-5f)
    {
        screenPt->x = 0.0f;
        screenPt->y = 0.0f;
        return;
    }

    // 变除法为乘法提速
    float inv_w = 1.0f / w;
    screenPt->x = u_prime * inv_w;
    screenPt->y = v_prime * inv_w;
}
