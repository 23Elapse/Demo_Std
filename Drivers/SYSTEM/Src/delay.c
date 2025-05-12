/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-05 21:24:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-12 21:20:40
 * @FilePath: \Demo\Drivers\SYSTEM\Src\delay.c
 * @Description: 
 * 
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved. 
 */
#include "pch.h"
#include "delay.h"
static uint32_t fac_us;  // 每微秒的SysTick周期数

void delay_init(uint32_t SYSCLK) {
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);  // SysTick时钟源 = HCLK
    fac_us = SYSCLK / 1000000;  // 计算每微秒的周期数
}
void delay_us(uint32_t nus) {
    if (nus == 0) return;

    uint32_t ticks = nus * fac_us;
    if (ticks == 0) return;

    const uint32_t max_ticks = 0xFFFFFF;

    SysTick->CTRL = 0;  // 关闭 SysTick，避免乱跳

    while (ticks > max_ticks) {
        SysTick->LOAD = max_ticks - 1;  // SysTick 是从 LOAD 开始递减到 0
        SysTick->VAL = 0;
        SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;

        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

        SysTick->CTRL = 0;
        ticks -= max_ticks;
    }

    SysTick->LOAD = ticks - 1;  // 注意这里也是 -1
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;

    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

    SysTick->CTRL = 0;
}

// void delay_us(uint32_t nus) {
//     uint32_t ticks, start, end;
//     uint32_t max_ticks = 0xFFFFFF;  // SysTick LOAD最大值

//     if (nus == 0) return;

//     ticks = nus * fac_us;  // 总需要的周期数

//     while (ticks > max_ticks) {
//         // 分次延时，每次最多max_ticks周期
//         SysTick->LOAD = max_ticks;
//         SysTick->VAL = 0;
//         SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
//         do {
//             start = SysTick->CTRL;
//         } while ((start & SysTick_CTRL_COUNTFLAG_Msk) == 0);
//         SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
//         ticks -= max_ticks;
//     }

//     // 剩余周期数
//     SysTick->LOAD = ticks;
//     SysTick->VAL = 0;
//     SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
//     do {
//         end = SysTick->CTRL;
//     } while ((end & SysTick_CTRL_COUNTFLAG_Msk) == 0);
//     SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
// }

void delay_ms(uint32_t nms) {
    while (nms--) {
        delay_us(1000);  // 分解为1ms延时
    }
}
