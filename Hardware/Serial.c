#include "Serial.h" // Device header
#include <stdarg.h>
#include <stdio.h>

const uint8_t RESET_KEY = 0xFF; // 定义一个全局变量，用于接收串口命令，控制系统重置

// 共用体：用于float和4字节数组的互转，方便解析串口收到的浮点数据
typedef union UnionFloat {
    uint8_t Array[4]; // 字节数组形式，用于逐字节接收
    float FloatNum;   // 浮点数形式，用于直接读取解析后的坐标
} UnionFloat_t;
float center_x, center_y; // 解析后的中心坐标

// 函数指针类型定义：用于串口命令处理函数的跳转表
typedef void (*cmd_handler_USART_t)(void);

#define Get_square(x) ((x) * (x)) // 计算平方
// 处理函数1：第一题的串口数据包解析与处理
void handle_USART_BasicQuestion1(void)
{
    u8 com_data;                        // 用于读取STM32串口收到的数据，这个数据会被下一个数据掩盖，所以要将它用一个数组储存起来。
    static bool data_packet_count = 0;  // 数据包计数：0=A5包，1=B6包
    static u8 RxCounter = 0;            // 共用体数组索引计数器（0-3循环）
    static u8 RxArrayCounter = 0;       // 全局字节计数器（0-9）
    static UnionFloat_t RxBuffer = {0}; // 定义一个6个成员的数组，可以存放6个数据，刚好放下一个数据包。
    static u8 RxState = 0;              // 接收状态，判断程序应该接收第一个帧头、第二个帧头、数据或帧尾。
    // static bool okk = 0;              // 接收状态，判断程序应该接收第一个帧头、第二个帧头、数据或帧尾。
    com_data = USART_ReceiveData(USART3);
    // if(okk==1)return;
    // 当RXState处于0时，为接收帧头1模式。若接收到帧头1（0xB6），将RXState置1，切换到接收帧头2模式，并将帧头1存入RxBuffer[0]的位置，RxCounter加一。
    if (RxState == 0 && com_data == 0xB6) // 0xB6帧头
    {
        RxState = 1;
        RxCounter = 0;
        RxArrayCounter = 0;
    }
    else if (RxState == 1)
    {
        RxBuffer.Array[RxCounter++] = com_data; // 数据填入共用体数组
        RxArrayCounter++;
        if (RxArrayCounter == 4) // 收满第1个float（4字节）
        {
            RxCounter = 0;
            center_x = RxBuffer.FloatNum;
        }
        else if (RxArrayCounter == 8) // 收满第2个float（4字节）
        {
            RxCounter = 0;
            center_y = RxBuffer.FloatNum;
        }
        else if (RxArrayCounter == 9) // 收满帧尾（第10个字节）
        {
            if (com_data == 0x6B) // 校验帧尾0x6B
            {
                RxCounter = 0;
                RxArrayCounter = 0;
                RxState = 0;
                if (data_packet_count == 1) // B6包收完：执行PID控制
                {
//                     OLED_ShowFloatNum(0, 32, center_x, 3, 3, OLED_8X16);
//                     OLED_ShowFloatNum(0, 48, center_y, 3, 3, OLED_8X16);
//                     OLED_Update();
                    PID_Control((int32_t)(center_x), (int32_t)(center_y));
                    // okk=1;
                    // data_packet_count = 0;
                    return;
                }
                else
                {
                    Target_Vertical_x = 0;
                    Target_Vertical_y = 0;
                    target_x = center_x;
                    target_y = center_y; 
                }
                data_packet_count = 1; // 切换为等待B6目标激光包
                uint8_t ack_data = 1; // 单个字节数据
                Serial_SendPacket(0xA5, 0x5A, &ack_data, 1);            
                }
            else
            {
                // 帧尾不对，立即重置，不用等 RxCounter > 10
                RxState = 0;
                RxCounter = 0;
                RxArrayCounter = 0;
                data_packet_count = 0;
                center_x = 0;
                center_y = 0;
            }
        }
    }
    else // 接收异常
    {
        RxState = 0;
        RxCounter = 0;
        center_x = 0;
        center_y = 0;
        RxArrayCounter = 0;
        RxBuffer.FloatNum = 0;
    }
}

// 处理函数2：第二题（预留）
void handle_USART_BasicQuestion2(void)
{

    u8 com_data;                        // 用于读取STM32串口收到的数据，这个数据会被下一个数据掩盖，所以要将它用一个数组储存起来。
    static bool data_packet_count = 0;  // 数据包计数：0=A5包，1=B6包
    static u8 RxCounter = 0;            // 共用体数组索引计数器（0-3循环）
    static u8 RxArrayCounter = 0;       // 全局字节计数器（0-9）
    static UnionFloat_t RxBuffer = {0}; // 定义一个6个成员的数组，可以存放6个数据，刚好放下一个数据包。
    static u8 RxState = 0;              // 接收状态，判断程序应该接收第一个帧头、第二个帧头、数据或帧尾。
    static uint8_t Position = 0;
    static Point2D src[4];
    static Point2D dst[4] = {
        {0.0f, 0.0f},     // 屏幕左上
        {100.0f, 0.0f},   // 屏幕右上
        {100.0f, 100.0f}, // 屏幕右下
        {0.0f, 100.0f}    // 屏幕左下
    };

    // 矩形4个目标点（顺时针闭环）
    static const Point2D rect_points[4] = {
        {0.0f, 0.0f},
        {100.0f, 0.0f},
        {100.0f, 100.0f},
        {0.0f, 100.0f}};
    com_data = USART_ReceiveData(USART3);
    // 当RXState处于0时，为接收帧头1模式。若接收到帧头1（0xA5），将RXState置1，切换到接收帧头2模式，并将帧头1存入RxBuffer[0]的位置，RxCounter加一。
    if (RxState == 0 &&  (com_data == 0xB6 && data_packet_count == 1)) // 0xA5帧头
    {
        RxState = 1;
        RxCounter = 0;
        RxArrayCounter = 0;
        RxBuffer.FloatNum = 0.0f;
    }
    else if (RxState == 1)
    {
        RxBuffer.Array[RxCounter++] = com_data; // 数据填入共用体数组
        RxArrayCounter++;
        if (data_packet_count == 0)
        {
            if (RxArrayCounter < 33)
            {
                switch (RxArrayCounter)
                {
                case 4:
                    RxCounter = 0;
                    src[0].x = RxBuffer.FloatNum;
                    break;
                case 8:
                    RxCounter = 0;
                    src[0].y = RxBuffer.FloatNum;
                    break;
                case 12:
                    RxCounter = 0;
                    src[1].x = RxBuffer.FloatNum;
                    break;
                case 16:
                    RxCounter = 0;
                    src[1].y = RxBuffer.FloatNum;
                    break;
                case 20:
                    RxCounter = 0;
                    src[2].x = RxBuffer.FloatNum;
                    break;
                case 24:
                    RxCounter = 0;
                    src[2].y = RxBuffer.FloatNum;
                    break;
                case 28:
                    RxCounter = 0;
                    src[3].x = RxBuffer.FloatNum;
                    break;
                case 32:
                    RxCounter = 0;
                    src[3].y = RxBuffer.FloatNum;
                    break;
                default:
                    break;
                }
            }
            else if (RxArrayCounter == 33) // 收满帧尾（第10个字节）
            {
                if (com_data == 0x5A)
                {
                    RxCounter = 0;
                    RxArrayCounter = 0;
                    RxState = 0;
                    if (!(calcHomography(src, dst, H)))
                    {
                        Serial_SendByte(0x01); // 接收到A5并发1（应答视觉端）
                        data_packet_count = 1; // 切换为等待B6目标包
                    }
                }
                else
                {
                    // 帧尾不对，立即重置，不用等 RxCounter > 10
                    RxState = 0;
                    RxCounter = 0;
                    RxArrayCounter = 0;
                    data_packet_count = 0;
                    center_x = 0;
                    center_y = 0;
                }
            }
        } // -------------------- 分支2：B6目标包（目标坐标，8字节数据） --------------------
        else
        {
            // 解析X坐标（前4字节）
            if (RxArrayCounter == 4)
            {
                RxCounter = 0;
                center_x = RxBuffer.FloatNum;
            }
            // 解析Y坐标（5-8字节）
            else if (RxArrayCounter == 8)
            {
                RxCounter = 0;
                center_y = RxBuffer.FloatNum;
            }
            // 收满8字节数据，第9字节为帧尾
            else if (RxArrayCounter == 9)
            {
                // 校验B6包帧尾0x6B
                if (com_data == 0x6B)
                {
                    Point2D pixel_pt = {center_x, center_y};
                    Point2D real_pt;
                    visualToReal(H, pixel_pt, &real_pt);
                    float err_x = target_x - real_pt.x;
                    float err_y = target_y - real_pt.y;
                    float dist_sq = Get_square(err_x) + Get_square(err_y);
                    if (dist_sq < 4.0f)
                    {
                        Position = (Position + 1) % 4;
                        target_x = rect_points[Position].x;
                        target_y = rect_points[Position].y;
                    }
                    PID_Control(real_pt.x, real_pt.y);

                    // 包接收完成，重置状态，准备下一次标定
                    RxState = 0;
                    RxCounter = 0;
                    RxArrayCounter = 0;
                    data_packet_count = 0;
                }
                else
                {
                    // 帧尾错误，全状态重置
                    RxState = 0;
                    RxCounter = 0;
                    RxArrayCounter = 0;
                    data_packet_count = 0;
                }
            }
        }
    }
    else // 接收异常
    {
        RxState = 0;
        RxCounter = 0;
        center_x = 0;
        center_y = 0;
        RxArrayCounter = 0;
        RxBuffer.FloatNum = 0;
    }
}
// 处理函数3：第三题（预留）
void handle_USART_BasicQuestion3(void)
{
}
// 处理函数4：第四题（预留）
void handle_USART_BasicQuestion4(void)
{
}

// 命令编号枚举（确保从0连续递增）
enum
{
    BasicQuestion1 = 0,
    BasicQuestion2,
    BasicQuestion3,
    BasicQuestion4
};

// 命令处理函数跳转表：通过Questionx索引直接调用对应处理函数
static cmd_handler_USART_t cmd_Questionx[6] = {
    [BasicQuestion1] = handle_USART_BasicQuestion1,
    [BasicQuestion2] = handle_USART_BasicQuestion2,
    [BasicQuestion3] = handle_USART_BasicQuestion3,
    [BasicQuestion4] = handle_USART_BasicQuestion4,
};

/*

    u8 com_data; // 用于读取STM32串口收到的数据，这个数据会被下一个数据掩盖，所以要将它用一个数组储存起来。
    u8 i;
    static u8 RxCounter = 0;
    static u16 RxBuffer[6] = {0}; // 定义一个6个成员的数组，可以存放6个数据，刚好放下一个数据包。
    static u8 RxState = 0;         // 接收状态，判断程序应该接收第一个帧头、第二个帧头、数据或帧尾。
    com_data = USART_ReceiveData(USART3);
     // 当RXState处于0时，为接收帧头1模式。若接收到帧头1（0x2C），将RXState置1，切换到接收帧头2模式，并将帧头1存入RxBuffer[0]的位置，RxCounter加一。
        if (RxState == 0 && com_data == 0x2C) // 0x2c帧头
        {
            RxState = 1;
            RxBuffer[RxCounter++] = com_data;
        }

        // 当RXState处于1时，为接收帧头2模式。若接收到帧头2（0x12），将RXState置2，切换到保存数据模式，并将帧头2存入RxBuffer[1]的位置，RxCounter加一。
        else if (RxState == 1 && com_data == 0x12) // 0x12帧头
        {
            RxState = 2;
            RxBuffer[RxCounter++] = com_data;
        }
        // 当RXState处于2时，为保存数据模式。RxBuffer[]将接收到的数据依次存入RxBuffer[2]、RxBuffer[3]、RxBuffer[4]、RxBuffer[5]中。当接收到第六位数据时，进行判断是否为帧尾（0x5B），若是帧尾分别保存数据RxBuffer[2]、RxBuffer[3]、RxBuffer[4]到x、y、z中
        else if (RxState == 2)
        {
            RxBuffer[RxCounter++] = com_data;

            if (RxCounter == 6 && com_data == 0x5B) // RxBuffer接受满了,接收数据结束
            {

                center_x = RxBuffer[RxCounter - 4];
                center_y = RxBuffer[RxCounter - 3];
                z = RxBuffer[RxCounter - 2];
                RxCounter = 0;
                RxState = 0;
                // PID_Control(center_x, center_y);
            }
            // 若是不是帧尾帧尾将会把RxState、RxCounter和RxBuffer[]全部置零做接收异常处理。
            else if (RxCounter > 6) // 接收异常
            {
                RxState = 0;
                RxCounter = 0;
                for (i = 0; i < 6; i++)
                {
                    RxBuffer[i] = 0x00; // 将存放数据数组清零
                }
            }
        }
        else // 接收异常
        {
            RxState = 0;
            RxCounter = 0;
            for (i = 0; i < 6; i++)
            {
                RxBuffer[i] = 0x00; // 将存放数据数组清零
            }
        } */

// 串口初始化函数：配置USART3、GPIO和中断
void Serial_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // 使能USART3时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // 使能GPIOB时钟

    // USART3_RX	   PB11
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;    // PB11
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // USART3_TX	   PB10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;      // PB10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // USART3配置
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;                                    // 波特率115200
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                                 // 仅接收模式（如需发送可改为USART_Mode_Rx | USART_Mode_Tx）
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // 无校验
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // 1位停止位
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 8位数据位
    USART_Init(USART3, &USART_InitStructure);

    USART_ClearITPendingBit(USART3, USART_IT_RXNE); // 清除接收中断标志

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 使能接收中断

    USART_Cmd(USART3, ENABLE); // 使能USART3
}

/**
 * 函    数：串口发送一个字节
 * 参    数：Byte 要发送的一个字节
 * 返 回 值：无
 */
void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART3, Byte); // 将字节数据写入数据寄存器，写入后USART自动生成时序波形
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
        ; // 等待发送完成
    /*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
 * 函    数：串口发送一个数组
 * 参    数：Array 要发送数组的首地址
 * 参    数：Length 要发送数组的长度
 * 返 回 值：无
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++) // 遍历数组
    {
        Serial_SendByte(Array[i]); // 依次调用Serial_SendByte发送每个字节数据
    }
}

/**
 * 函    数：串口发送一个字符串
 * 参    数：String 要发送字符串的首地址
 * 返 回 值：无
 */
void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) // 遍历字符数组（字符串），遇到字符串结束标志位后停止
    {
        Serial_SendByte(String[i]); // 依次调用Serial_SendByte发送每个字节数据
    }
}

/**
 * 函    数：次方函数（内部使用）
 * 返 回 值：返回值等于X的Y次方
 */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1; // 设置结果初值为1
    while (Y--)          // 执行Y次
    {
        Result *= X; // 将X累乘到结果
    }
    return Result;
}

/**
 * 函    数：串口发送数字
 * 参    数：Number 要发送的数字，范围：0~4294967295
 * 参    数：Length 要发送数字的长度，范围：0~10
 * 返 回 值：无
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) // 根据数字长度遍历数字的每一位
    {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0'); // 依次调用Serial_SendByte发送每位数字
    }
}

/**
 * 函    数：串口发送数据包
 * 参    数：packet_header 包头
 * 参    数：packet_tail 包尾
 * 参    数：Array 数据数组首地址
 * 参    数：Length 数据长度
 * 返 回 值：无
 * 说    明：调用此函数后，Serial_TxPacket数组的内容将加上包头包尾后，作为数据包发送出去
 */
void Serial_SendPacket(uint8_t packet_header, uint8_t packet_tail, uint8_t *Array, uint16_t Length)
{
    Serial_SendByte(packet_header);  // 发送包头
    Serial_SendArray(Array, Length); // 发送数据
    Serial_SendByte(packet_tail);    // 发送包尾
}

// USART3中断服务函数：处理串口接收
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // 接收中断
    {

        USART_ClearITPendingBit(USART3, USART_IT_RXNE); // 清除中断标志
        // 开机握手：利用&&短路特性，只有Power_on_flag==0时才会读数据
        if (Power_on_flag == 0 && USART_ReceiveData(USART3) == 0x6B)
        {
            if (Stop_flag == 1)
            {
                Power_on_flag = 1;                                       // 置位开机标志
                Serial_SendPacket(0xA5, 0x5A, (uint8_t *)&Questionx, 1); // 发送题目指令包（A5+题号+5A）
            }
            return;
        }

        // 检查是否满足运行条件：题号有效、已开机、未停止
        if (((Questionx - 1) <= (-1)) || Power_on_flag == 0 || Stop_flag == 0)
        {
            return;
        }
        cmd_Questionx[Questionx - 1](); // 调用对应题目的处理函数
    }
}
