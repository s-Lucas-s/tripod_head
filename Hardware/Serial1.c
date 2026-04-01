#include "Serial1.h"

// ===================== 串口1 初始化 =====================
void Serial1_Init(void)
{
    // 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    // PA10 (RX) 上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA9 (TX) 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 串口参数配置 115200 8N1
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx; // 仅需发送，关闭接收节省资源
    USART_Init(USART1, &USART_InitStructure);

    // 使能串口1
    USART_Cmd(USART1, ENABLE);
}

// ===================== 基础发送函数 =====================
void Serial1_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial1_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for(i=0; i<Length; i++)
    {
        Serial1_SendByte(Array[i]);
    }
}

void Serial1_SendString(char *String)
{
    uint8_t i;
    for(i=0; String[i]!='\0'; i++)
    {
        Serial1_SendByte(String[i]);
    }
}

// ===================== VOFA+ 3通道波形发送函数 =====================
// 严格遵循官方协议：3路float + 帧尾 00 00 80 7F
void VOFA_Send3Ch(float ch1, float ch2, float ch3)
{
    // 定义协议帧结构
    typedef struct {
        float data[3];
        uint8_t tail[4];
    }VOFA_Frame_t;

    VOFA_Frame_t frame = {0};
    // 填充通道数据
    frame.data[0] = ch1;
    frame.data[1] = ch2;
    frame.data[2] = ch3;
    // 固定帧尾
    frame.tail[0] = 0x00;
    frame.tail[1] = 0x00;
    frame.tail[2] = 0x80;
    frame.tail[3] = 0x7F;

    // 发送完整数据包
    Serial1_SendArray((uint8_t*)&frame, sizeof(frame));
}

// ===================== 串口1中断函数 =====================
// 因仅用于发送波形，中断为空，不影响程序
void USART1_IRQHandler(void)
{
    // 清空标志位，防止中断触发报错
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART1);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}