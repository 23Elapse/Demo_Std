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
    delay_init(180);		  //初始化延时函数
    led_init();                                 /* 初始化LED */
    while(1)
    {
        LED0(0);                                /* LED0 亮 */
        LED1(1);                                /* LED1 灭 */
        delay_ms(500);
        LED0(1);                                /* LED0 灭 */
        LED1(0);                                /* LED1 亮 */
        delay_ms(500);
    }
}


