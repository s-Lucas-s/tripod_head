#include "PID.h"
#include <math.h>

/************************ 全局宏定义 ************************/
#define Integral_MAX 500000        // PID积分限幅最大值，防止积分饱和失控
#define MAX_SPEED 16000            // 电机最大输出限制，保护电机与机械结构
#define PI 3.1415926f              // 圆周率，用于圆形轨迹计算，备用参数

/************************ 透视变换单应性矩阵 ************************/
float H[3][3];                     // 3x3透视变换矩阵，用于视觉坐标→实际坐标转换

/************************ 枚举与结构体定义 ************************/
// 控制维度枚举：区分X轴/Y轴控制
typedef enum
{
    x,                              // X轴控制
    y                               // Y轴控制
} Dimension_t;

// PID参数结构体：封装位置环PID的比例/积分/微分系数
typedef struct parameter_pid
{
    int32_t kp;                     // 比例系数：快速减小误差
    int32_t ki;                     // 积分系数：消除静态误差
    int32_t kd;                     // 微分系数：抑制震荡、提高响应速度
} pid_t;

/************************ 全局PID参数实体 ************************/
pid_t Position_PID_x;               // X轴位置环PID参数
pid_t Position_PID_y;               // Y轴位置环PID参数

/**
 * @brief  PID参数初始化函数
 * @note   上电默认清零PID参数，调试时再赋值
 * @retval 无
 */
void PID_Init(void)
{
    // Y轴PID初始化为0，X轴同理可添加初始化
    Position_PID_y.kp=0;
    Position_PID_y.ki=0;
    Position_PID_y.kd=0;
}

/************************ 轨迹与前馈控制参数 ************************/
// 外部可直接赋值，控制电机运动速度
uint32_t Target_Vertical_x = 0;     // X轴目标运动速度（像素/帧）
uint32_t Target_Vertical_y = 0;     // Y轴目标运动速度（像素/帧）

// 轨迹目标坐标：视觉跟踪的目标点
float_t target_x = 0;              // X轴目标坐标
float_t target_y = 0;              // Y轴目标坐标

// 速度前馈系数：开环前馈，提升轨迹跟随平滑度
const int32_t Kf_x = 10;            // X轴前馈系数
const int32_t Kf_y = 10;            // Y轴前馈系数

/************************ PID内部静态变量 ************************/
static int32_t xerr_last = 0;       // X轴上一帧误差，用于微分计算
static int32_t yerr_last = 0;       // Y轴上一帧误差，用于微分计算
static uint32_t time_count_last = 0;// 上一帧控制时间戳，计算时间间隔dt
static int32_t integral_x = 0;      // X轴积分累加值
static int32_t integral_y = 0;      // Y轴积分累加值

/************************ 工具宏定义 ************************/
#define Get_square(x) ((x) * (x))                           // 计算平方
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
    } while (0)                               // 对称限幅宏：限制数值在±max_val之间
#define LIMIT_SYMMETRIC(value) LIMIT_VALUE_SYMMETRIC(value, Integral_MAX)  // 积分专用限幅

/**
 * @brief  单轴位置环PID控制计算
 * @param  Dimension: 控制维度 x/y
 * @param  err: 当前位置偏差（目标值-实际值）
 * @param  dt: 控制周期时间间隔（单位：ms）
 * @retval pid_out: PID计算输出值
 */
int32_t Position_PID_Control(Dimension_t Dimension, float err, uint32_t dt)
{
    float pid_out = 0;
    if(dt == 0) dt = 1;              // 防止除0错误

    if (Dimension == x)              // X轴PID计算
    {
        integral_x += err;           // 误差积分累加
        LIMIT_VALUE_SYMMETRIC(integral_x, Integral_MAX);  // 积分限幅

        // 位置环PID公式：输出 = Kp*误差 + Ki*积分 + Kd*微分
        pid_out = Position_PID_x.kp * err
                + Position_PID_x.ki * integral_x
                + Position_PID_x.kd * (err - xerr_last) / dt;
    }
    else                             // Y轴PID计算
    {
        integral_y += err;           // 误差积分累加
        LIMIT_VALUE_SYMMETRIC(integral_y, Integral_MAX);  // 积分限幅

        // 位置环PID公式
        pid_out = Position_PID_y.kp * err
                + Position_PID_y.ki * integral_y
                + Position_PID_y.kd * (err - yerr_last) / dt;
    }

    LIMIT_VALUE_SYMMETRIC(pid_out, MAX_SPEED);  // PID输出限幅
    return (int32_t)(pid_out + 0.5f);
}

/**
 * @brief  轨迹生成模块：更新目标点坐标
 * @param  dt: 时间间隔
 * @retval 无
 * @note   独立模块，修改轨迹仅需改动此函数
 */
void Trajectory_Update(uint32_t dt)
{
    // 默认：匀速直线运动轨迹
    target_x += Target_Vertical_x * dt / 1000;
    target_y += Target_Vertical_y * dt / 1000;

/*
    【圆形绕圈轨迹示例】
    static float angle = 0;
    float radius = 100;     // 绕圈半径（像素）
    float speed = 0.05f;    // 绕圈角速度
    angle += speed;
    if(angle > 2*PI) angle = 0;
    target_x = 320 + radius * cos(angle);  // 屏幕中心X
    target_y = 240 + radius * sin(angle);  // 屏幕中心Y
*/
}

/**
 * @brief  主控制函数：视觉位置环+速度前馈总控
 * @param  now_x: 视觉识别的当前激光X坐标
 * @param  now_y: 视觉识别的当前激光Y坐标
 * @retval 无
 * @note   电赛核心控制逻辑：单闭环+前馈，稳定不耦合
 */
void PID_Control(float now_x, float now_y)
{
    int32_t x_out = 0;               // X轴最终电机输出
    int32_t y_out = 0;               // Y轴最终电机输出
    uint32_t time_count = 0;         // 当前时间戳
    uint32_t interval_time = 0;      // 控制周期间隔时间

    // 获取定时器计时，计算时间间隔dt
    time_count = g_timer3_count;
    interval_time = time_count - time_count_last;

    // 时间容错：防止异常大值/0值导致计算错误
    if(interval_time > 100) interval_time = 100;
    if(interval_time == 0) interval_time = 1;
    time_count_last = time_count;

    // 停机标志：停机时清零所有状态，防止重启漂移
    if (!Stop_flag)
    {
        xerr_last = 0; yerr_last = 0;
        integral_x = 0; integral_y = 0;
        target_x = 0; target_y = 0;
        return;
    }

    // ===================== 核心控制流程 =====================
    // 步骤1：更新轨迹目标点
    Trajectory_Update(interval_time);

    // 步骤2：计算视觉位置误差 = 目标坐标 - 当前视觉坐标
    float err_x = target_x - now_x;
    float err_y = target_y - now_y;

    // 步骤3：位置环PID计算误差修正量
    int32_t pid_x = Position_PID_Control(x, err_x, interval_time);
    int32_t pid_y = Position_PID_Control(y, err_y, interval_time);

    // 步骤4：速度前馈计算：开环基础速度，提升跟随性
    int32_t feed_x = Target_Vertical_x * Kf_x;
    int32_t feed_y = Target_Vertical_y * Kf_y;

    // 步骤5：最终输出 = PID修正 + 速度前馈
    x_out = pid_x + feed_x;
    y_out = pid_y + feed_y;

    // 输出限幅，保护电机
    LIMIT_VALUE_SYMMETRIC(x_out, MAX_SPEED);
    LIMIT_VALUE_SYMMETRIC(y_out, MAX_SPEED);

    // ===================== 电机驱动控制 =====================
    // X轴电机：根据输出正负判断方向
    if(x_out >= 0)
        Emm_V5_Pos_Control(1, 0, (uint16_t)x_out, 0, 16000, 0, 1);
    else
        Emm_V5_Pos_Control(1, 1, (uint16_t)-x_out, 0, 16000, 0, 1);

    // Y轴电机：根据输出正负判断方向
    if(y_out >= 0)
        Emm_V5_Pos_Control(2, 0, (uint16_t)y_out, 0, 14000, 0, 1);
    else
        Emm_V5_Pos_Control(2, 1, (uint16_t)-y_out, 0, 14000, 0, 1);

    Emm_V5_Synchronous_motion(0);  // 电机同步运动执行

    // 保存当前误差，用于下一帧微分计算
    xerr_last = err_x;
    yerr_last = err_y;
}

// ====================== 透视变换（视觉标定）核心函数 ======================
/**
 * @brief  计算透视变换单应性矩阵H（Hartley归一化优化，适配F103）
 * @param  src[4]: 视觉采集的4个角点像素坐标
 * @param  dst[4]: 实际物理4个角点坐标
 * @param  H[3][3]: 输出3x3透视变换矩阵
 * @retval 0=成功, -1=失败
 * @note   上电仅标定一次，不占用实时算力
 */
int calcHomography(Point2D src[4], Point2D dst[4],float H[3][3])
{
    static float A[8][9];
    int i, j;
    float cx_src = 0.0f, cy_src = 0.0f, cx_dst = 0.0f, cy_dst = 0.0f;
    float avg_dist_src = 0.0f, avg_dist_dst = 0.0f;
    float s_src, s_dst;
    const float SQRT2 = 1.41421356f;

    // 1. 矩阵A清零初始化
    for (i = 0; i < 8; i++) {
        for (j = 0; j < 9; j++) {
            A[i][j] = 0.0f;
        }
    }

    // 2. Hartley归一化：计算坐标点质心，提升计算精度
    for (i = 0; i < 4; i++) {
        cx_src += src[i].x; cy_src += src[i].y;
        cx_dst += dst[i].x; cy_dst += dst[i].y;
    }
    cx_src *= 0.25f; cy_src *= 0.25f;
    cx_dst *= 0.25f; cy_dst *= 0.25f;

    // 3. 计算归一化缩放因子
    for (i = 0; i < 4; i++) {
        float dx_s = src[i].x - cx_src; float dy_s = src[i].y - cy_src;
        float dx_d = dst[i].x - cx_dst; float dy_d = dst[i].y - cy_dst;
        avg_dist_src += sqrtf(dx_s * dx_s + dy_s * dy_s);
        avg_dist_dst += sqrtf(dx_d * dx_d + dy_d * dy_d);
    }
    avg_dist_src *= 0.25f;
    avg_dist_dst *= 0.25f;

    // 非法坐标判断
    if (avg_dist_src < 1e-6f || avg_dist_dst < 1e-6f) return -1;

    s_src = SQRT2 / avg_dist_src;
    s_dst = SQRT2 / avg_dist_dst;

    // 4. 构建8×9线性方程组矩阵A
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

    // 5. 高斯-约当消元求解矩阵
    for (int col = 0; col < 8; col++) {
        float maxVal = fabs(A[col][col]);
        int maxRow = col;
        for (i = col + 1; i < 8; i++) {
            if (fabs(A[i][col]) > maxVal) {
                maxVal = fabs(A[i][col]);
                maxRow = i;
            }
        }

        // 行交换，避免除0
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

    // 6. 反归一化，计算最终H矩阵
    float h00 = -A[0][8], h01 = -A[1][8], h02 = -A[2][8];
    float h10 = -A[3][8], h11 = -A[4][8], h12 = -A[5][8];
    float h20 = -A[6][8], h21 = -A[7][8];

    // 矩阵运算：归一化逆变换
    float M00 = h00 * s_src;
    float M01 = h01 * s_src;
    float M02 = h02 - M00 * cx_src - M01 * cy_src;

    float M10 = h10 * s_src;
    float M11 = h11 * s_src;
    float M12 = h12 - M10 * cx_src - M11 * cy_src;

    float M20 = h20 * s_src;
    float M21 = h21 * s_src;
    float M22 = 1.0f - M20 * cx_src - M21 * cy_src;

    if (fabs(M22) < 1e-6f) return -1;
    float inv_s_dst = 1.0f / s_dst;
    float inv_H22 = 1.0f / M22;

    // 最终赋值3x3单应性矩阵H
    H[0][0] = (inv_s_dst * M00 + cx_dst * M20) * inv_H22;
    H[0][1] = (inv_s_dst * M01 + cx_dst * M21) * inv_H22;
    H[0][2] = (inv_s_dst * M02 + cx_dst * M22) * inv_H22;

    H[1][0] = (inv_s_dst * M10 + cy_dst * M20) * inv_H22;
    H[1][1] = (inv_s_dst * M11 + cy_dst * M21) * inv_H22;
    H[1][2] = (inv_s_dst * M12 + cy_dst * M22) * inv_H22;

    H[2][0] = M20 * inv_H22;
    H[2][1] = M21 * inv_H22;
    H[2][2] = 1.0f;

    return 0;
}

/**
 * @brief  视觉像素坐标 → 实际物理坐标转换
 * @param  H[3][3]: 已标定好的透视矩阵
 * @param  pixelPt: 视觉识别的激光像素点
 * @param  screenPt: 输出转换后的实际坐标
 * @retval 无
 */
void visualToReal(float H[3][3], Point2D pixelPt, Point2D *screenPt)
{
    float x = pixelPt.x, y = pixelPt.y;

    // 矩阵左乘运算
    float u_prime = H[0][0] * x + H[0][1] * y + H[0][2];
    float v_prime = H[1][0] * x + H[1][1] * y + H[1][2];
    float w       = H[2][0] * x + H[2][1] * y + H[2][2];

    // 除0保护
    if (fabs(w) < 1e-5f)
    {
        screenPt->x = 0.0f;
        screenPt->y = 0.0f;
        return;
    }

    // 齐次坐标归一化（乘法提速，替代除法）
    float inv_w = 1.0f / w;
    screenPt->x = u_prime * inv_w;
    screenPt->y = v_prime * inv_w;
}
