/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-26 15:38:11
 * @LastEditors: Elapse userszy@163.com
 * @LastEditTime: 2024-12-21 17:19:09
 * @FilePath: \Demo\User\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pch.h"


/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
    NVIC_SetPriorityGrouping(2);     
    SystemClock_Config(360, 25, 2, 8);    /* 设置时钟,180Mhz */  
    delay_init(180);                         /* 延时初始化 */
    led_init();                              /* 初始化LED */
    key_init();                              /* 初始化按键 */
    delay_ms(100);                           /* 延时100ms再初始化看门狗,LED0的变化"可见" */
    // IWDG_Init(IWDG_Prescaler_64, 500);       /* 预分频数为64,重载值为500,溢出时间约为1s */
    TIM6_Init(5000 - 1, 9000 - 1);              /* 90 000 000 / 9000 = 10KHz 10KHz的计数频率，计数5K次为500ms */
    LED0(0);                                    /* 点亮LED0(红灯) */

    while (1)
    {
        LED0_TOGGLE();                       /* LED0(红灯) 翻转 */
        delay_ms(2000);
    }
}


