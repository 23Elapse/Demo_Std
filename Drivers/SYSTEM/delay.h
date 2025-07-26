/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-06-08 16:13:58
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-08 18:41:58
 * @FilePath: \Demo_backup\Drivers\SYSTEM\Inc\delay.h
 * @Description: 
 * 
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved. 
 */
/**
 * =====================================================================================
 * @file        delay.h
 * @brief       延时函数模块头文件 (已优化，支持RTOS)
 * @author      23Elapse & Gemini
 * @version     2.0 (RTOS-Aware)
 * @date        2025-06-08
 * =====================================================================================
 */
#ifndef __DELAY_H
#define __DELAY_H

#include "sys.h" // 包含 SYS_SUPPORT_OS 宏定义

/**
 * @brief 初始化延时函数
 * @note  此函数会配置SysTick作为时钟源。在RTOS环境下，SysTick通常被RTOS本身用于心跳节拍。
 * FreeRTOS会自动处理SysTick的配置，因此在RTOS环境下此函数可以为空或不被调用。
 * @param SYSCLK 系统主时钟频率 (MHz)
 */
void delay_init(void);

/**
 * @brief 微秒级延时
 * @note  这是一个阻塞式的忙等待延时，会占用CPU。
 * 请仅用于短时间延时（建议 < 1000us），尤其是在RTOS任务中。
 * @param nus 要延时的微秒数
 */
void delay_us(uint32_t nus);

/**
 * @brief 毫秒级延时
 * @note  此函数已对RTOS环境进行适配。
 * - 在RTOS环境下，它会调用 vTaskDelay()，实现非阻塞的任务调度延时。
 * - 在非RTOS环境下，它会循环调用 delay_us() 实现阻塞延时。
 * @param nms 要延时的毫秒数
 */
void delay_ms(uint32_t nms);

#endif
