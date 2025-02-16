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
    USART1_Init();                      /* 初始化USART */
    led_init();                              /* 初始化LED */
    key_init();                              /* 初始化按键 */
    extix_init();                            /* 初始化外部中断输入 */
    LED0(0);                                 /* 先点亮红灯 */

    while (1)
    {
        delay_ms(1000);
    }
}


