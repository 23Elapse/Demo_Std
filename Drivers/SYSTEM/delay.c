/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-06-08 16:13:58
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-07-26 18:00:43
 * @FilePath: \Demo_Std_F407\Drivers\SYSTEM\delay.c
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
#include "rtos_abstraction.h" // RTOS抽象层接口
void delay_init(void)
{
    // 启用 DWT 寄存器
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t cycles = us * (SystemCoreClock / 1000000U);
    uint32_t start = DWT->CYCCNT;

    while ((DWT->CYCCNT - start) < cycles);
}

// void delay_ms(uint32_t ms)
// {
//     while (ms--)
//         delay_us(1000);

// }
void delay_ms(uint32_t ms)
{
    if (g_rtos_ops && g_rtos_ops->Delay 
        && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        && (ms >= 1)) {

            g_rtos_ops->Delay(pdMS_TO_TICKS(ms));
    }   
    else
    {
        // 若未启动调度器或非RTOS环境，使用us延时实现ms延时
        while (ms--)
            delay_us(1000);
    }
}

