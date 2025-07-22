#include "pch.h"

// 定时器中断周期（单位：毫秒）
#define TIMER_INTERRUPT_PERIOD_MS 1000

// 定时器初始化函数
void TIM6_Init(uint16_t arr, uint16_t psc) {
    // 1. 使能 TIM6 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    // 2. 配置定时器
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_TimeBaseStruct.TIM_Prescaler = psc;  // 预分频器，1MHz 计数频率
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;            // 向上计数模式
    TIM_TimeBaseStruct.TIM_Period = arr;  // 自动重装载值
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;                // 时钟分频
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;                       // 重复计数器（基本定时器无效）
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStruct);

    // 3. 使能定时器中断
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);  // 使能更新中断

    // 4. 配置 NVIC
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM6_DAC_IRQn;  // TIM6 中断通道
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 5;  // 抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;         // 子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;            // 使能中断
    NVIC_Init(&NVIC_InitStruct);

    // 5. 启动定时器
    TIM_Cmd(TIM6, ENABLE);
}

// TIM6 中断服务函数
void TIM6_DAC_IRQHandler(void) {
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {
        // 清除中断标志
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

        // 定时器中断处理逻辑
        // 例如：翻转 LED 状态
         LED1_TOGGLE();                                  /* LED1反转 */

    }
}
