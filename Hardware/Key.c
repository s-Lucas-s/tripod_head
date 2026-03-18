#include "stm32f10x.h"                  // STM32F10x设备头文件
#include "Key.h"

static unsigned char Key_Code = 0;

// 按键引脚定义：可根据实际硬件修改（此处以GPIOA的4个引脚为例）
#define KEY1_PIN    GPIO_Pin_0    // 按键1引脚：GPIOB_Pin0
#define KEY2_PIN    GPIO_Pin_1    // 按键2引脚：GPIOB_Pin1
#define KEY_GPIO    GPIOB         // 按键所在GPIO端口


/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;  										// GPIO初始化结构体
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   							// 模式：上拉输入（IPU）
    GPIO_InitStructure.GPIO_Pin = KEY1_PIN | KEY2_PIN;  	// 选中4个按键引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 							
    
    GPIO_Init(KEY_GPIO, &GPIO_InitStructure);
}

/**
 * @brief 外部调用函数
 * 
 * @return unsigned char 按键非连续键值
 */
unsigned char Key_GetCode(void)
{
    unsigned char TempCode = 0;
    TempCode = Key_Code;
    Key_Code = 0;
    return TempCode;
}


/**
  * 函数名：Key_GetCurrentState
  * 功能：读取当前按键的电平状态，返回实时键码
  * 参数：无
  * 返回值：unsigned char - 实时键码（0=无按键，1=KEY1，2=KEY2）
  */
unsigned char Key_Get(void)
{
    unsigned char CurrentKey = 0;  // 默认无按键
    
    // 读取引脚电平：低电平代表按键按下（上拉输入模式）
    if (GPIO_ReadInputDataBit(KEY_GPIO, KEY1_PIN) == 0)
    {
        CurrentKey = 1;  														// KEY1按下
    }
    else if (GPIO_ReadInputDataBit(KEY_GPIO, KEY2_PIN) == 0)
    {
        CurrentKey = 2;  														// KEY2按下
    }
    
    return CurrentKey;
}

/**
  * 函数名：Key_LoopDetect
  * 功能：循环检测按键状态，实现消抖与松手检测，更新全局键码
  * 参数：无
  * 返回值：无
  * 说明：需在定时器中调用，确保实时检测
  */
void Key_LoopDetect(void)
{
    static unsigned char LastState = 0;  										// 静态变量：存储上一次按键状态（初始为无按键）
    unsigned char NowState = 0;          										// 变量：存储当前按键状态
    
    NowState = Key_Get();
    
    // 2. 状态对比：上一状态为“按键按下”，当前状态为“无按键”→ 松手瞬间
    if (LastState == 1 && NowState == 0)
    {
        Key_Code = 1;  // 更新键码为1（KEY1松手）
    }
    else if (LastState == 2 && NowState == 0)
    {
        Key_Code = 2;  // 更新键码为2（KEY2松手）
    }
    
    // 3. 更新“上一状态”：为下一次检测做准备
    LastState = NowState;
}
