#ifndef __DELAY_H
#define __DELAY_H

#include "sys.h"


void delay_init(uint32_t SYSCLK);           /* 初始化延迟函数 */
void delay_ms(uint32_t nms);                /* 延时nms */
void delay_us(uint32_t nus);                /* 延时nus */

void HAL_Delay(uint32_t Delay);             /* HAL库的延时函数，SDIO等需要用到 */

#endif

