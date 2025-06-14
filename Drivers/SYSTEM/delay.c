/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-06-08 16:13:58
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-08 20:34:40
 * @FilePath: \Demo_backup\Drivers\SYSTEM\delay.c
 * @Description: 
 * 
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved. 
 */
/**
 * =====================================================================================
 * @file        delay.c
 * @brief       延时函数模块实现 (已优化，支持RTOS)
 * @author      23Elapse & Gemini
 * @version     2.0 (RTOS-Aware)
 * @date        2025-06-08
 * =====================================================================================
 */
#include "delay.h"
#include "pch.h" // 包含rtos_abstraction.h和FreeRTOS相关头文件

// fac_us: 每微秒对应的SysTick计数值
static uint32_t fac_us = 0;

/**
 * @brief 初始化延时模块
 */
void delay_init(uint32_t SYSCLK) {
#if SYS_SUPPORT_OS  // 如果使用RTOS
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);  // SysTick时钟源 = HCLK
    fac_us = SYSCLK / 1000000;  // 计算每微秒的周期数
    // 在RTOS中，SysTick已被RTOS用于生成系统节拍，
    // FreeRTOS的vTaskDelay等函数依赖于此。
    // 通常不需要再次手动配置SysTick。
    // FreeRTOSConfig.h中的configTICK_RATE_HZ决定了节拍频率。
    // 如果需要独立的硬件定时器做delay_us，则需另外实现。
    // 此处我们仍然基于SysTick，但需注意其已被RTOS接管。
#else
    // 裸机环境下的SysTick配置
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    fac_us = SYSCLK / 1000000;
#endif
}
void delay_us(uint32_t nus);
/**
 * @brief 微秒级延时 (忙等待)
 */
// void delay_us(uint32_t nus) {
//     // 这部分实现与您原版相同，是精确但阻塞的。
//     // 在RTOS启动前或中断服务程序中，或需要高精度短延时(几十us)时使用。
//     uint32_t ticks;
//     uint32_t told, tnow, tcnt = 0;
//     uint32_t reload = SysTick->LOAD;      // 获取LOAD的值
//     fac_us = SystemCoreClock / 1000000;   // 更新us因子
//     ticks = nus * fac_us;                 // 需要的节拍数
//     told = SysTick->VAL;                  // 刚进入时的计数器值
//     while (1) {
//         tnow = SysTick->VAL;
//         if (tnow != told) {
//             if (tnow < told) {
//                 tcnt += told - tnow;
//             } else {
//                 tcnt += reload - tnow + told;
//             }
//             told = tnow;
//             if (tcnt >= ticks) break; // 时间超过/等于要延迟的时间,则退出
//         }
//     }
// }

/**
 * @brief 毫秒级延时 (RTOS适配)
 */
void delay_ms(uint32_t nms) {
#if SYS_SUPPORT_OS // 如果定义了SYS_SUPPORT_OS,说明使用RTOS
    if (g_rtos_ops && g_rtos_ops->Delay) {
        // 使用RTOS提供的延时函数，这将使当前任务进入阻塞态，让出CPU
        g_rtos_ops->Delay(nms);
    } else {
        // RTOS环境下但抽象层未就绪，使用旧方法
        while(nms--) {
            delay_us(1000);
        }
    }
#else // 非RTOS环境
    while(nms--) {
        delay_us(1000);
    }
#endif
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


