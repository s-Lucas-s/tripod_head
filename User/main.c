#include "sys.h"
#include <math.h>  //正弦波使用

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
    SCB->VTOR = FLASH_BASE | 0x2000;
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, NVIC_VectTab_FLASH_OFFSET);
    __enable_irq();

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Enable GPIOA clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* Enable USART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* Tx */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Rx */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART resources configuration (Clock, GPIO pins and USART registers) ----*/
    /* USART configured as follow:
          - BaudRate = 115200 baud
          - Word Length = 8 Bits
          - One Stop Bit
          - No parity
          - Hardware flow control disabled (RTS and CTS signals)
          - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 使能接收中断
    USART_Cmd(USART1, ENABLE);

    nvic_init();
    PID_Init();
    board_init();
    Key_Init();
    OLED_Init();
    Serial_Init();
    Serial1_Init();
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
			// Delay_s(5);
			// while(1);
        uint8_t keyValue = 0;
        Key_LoopDetect();
        keyValue = Key_GetCode(); // 获取单击按键值

        // KEY1按下：mode递增，1-4循环
        if (keyValue == 1)
        {
            Questionx++;
            if (Questionx > 4) // 超过4则重置为1
            {
                Questionx = 1;
            }
        }
        // KEY2按下：Stop_Flag翻转(0变1，1变0)
        else if (keyValue == 2)
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
        // OLED_ShowFloatNum(0,16, center_x, 3, 3, OLED_8X16);
        // OLED_ShowFloatNum(0, 32, center_y, 3, 3, OLED_8X16);
        OLED_ShowString(0, 0, "Mode:", OLED_8X16);
        OLED_ShowNum(48, 0, Questionx, 1, OLED_8X16);
        OLED_ShowString(0, 16, "Stop:", OLED_8X16);
        OLED_ShowNum(48, 16, Stop_flag, 1, OLED_8X16);
        OLED_Update();

        static float t = 0.0f;
        float ch1, ch2, ch3;
        t += 0.05f;
        // 3路不同频率波形，方便区分
        ch1 = 500.0f * sin(t) + 500.0f;    // 通道1：0~1000，基频
        ch2 = 500.0f * sin(2 * t) + 500.0f;// 通道2：2倍频
        ch3 = 500.0f * sin(3 * t) + 500.0f;// 通道3：3倍频
        // 发送数据
        VOFA_Send3Ch(ch1, ch2, ch3);
        Delay_ms(10);
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

static uint8_t USART1_ReceiveData = 0;
static uint8_t USART1_ReceiveDataCount = 0;
void check_Bootloader(uint8_t data)
{
    if (data == 0x7E)
    {
        if (++USART1_ReceiveDataCount >= 5)
        {
            USART1_ReceiveDataCount = 0;

            // bootloader处理
            IAP_WriteFlag(IAP_UPGRADE_FLAG);
            NVIC_SystemReset();
        }
    }
    else
    {
        USART1_ReceiveDataCount = 0;
    }
}

// USART1中断服务函数：处理串口接收
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // 接收中断
    {
        USART1_ReceiveData = USART_ReceiveData(USART1); // 读取接收数据

        check_Bootloader(USART1_ReceiveData);

        USART_ClearITPendingBit(USART1, USART_IT_RXNE); // 清除中断标志
    }
}
