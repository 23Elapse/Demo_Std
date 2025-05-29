/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-27 21:33:38
 * @LastEditors: Elapse userszy@163.com
 * @LastEditTime: 2024-10-27 21:33:44
 * @FilePath: \Demo\Drivers\BSP\Inc\wdg.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __WDG_H
#define __WDG_H


void IWDG_Init(uint8_t prescaler, uint16_t reload);        /* 初始化IWDG，并使能IWDG */
void IWDG_Feed(void);                               /* 喂狗 */

#endif
