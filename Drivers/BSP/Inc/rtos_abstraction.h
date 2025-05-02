/*
 * @File: rtos_abstraction.h
 * @Description: RTOS 抽象层头文件
 * @Date: 2025-04-30
 */

#ifndef __RTOS_ABSTRACTION_H
#define __RTOS_ABSTRACTION_H

#include <stdint.h>
#include <stddef.h>
/**
 * @brief RTOS 操作接口结构体
 */
typedef struct
{
    void *(*SemaphoreCreate)(void);                                                                                        // 创建信号量
    void (*SemaphoreDelete)(void *sem);                                                                                    // 删除信号量
    int (*SemaphoreTake)(void *sem, uint32_t timeout);                                                                     // 获取信号量
    void (*SemaphoreGive)(void *sem);                                                                                      // 释放信号量
    int (*SemaphoreGiveFromISR)(void *sem, void *xHigherPriorityTaskWoken);                                                // 中断中释放信号量
    void (*YieldFromISR)(void *xHigherPriorityTaskWoken);                                                                  // 中断中任务切换
    uint32_t (*GetTickCount)(void);                                                                                        // 获取当前 tick 计数
    void (*Delay)(uint32_t ticks);                                                                                         // 任务延时
    void *(*TaskCreate)(void (*task_func)(void *), const char *name, uint32_t stack_size, void *param, uint32_t priority); // 创建任务
    void (*TaskDelete)(void *task);                                                                                        // 删除任务
    void *(*Malloc)(size_t size);                                                                                          // 动态内存分配
    void (*Free)(void *ptr);                                                                                               // 释放动态内存
} RTOS_Ops_t;

/**
 * @brief 设置 RTOS 操作接口
 * @param ops RTOS 操作接口指针
 */
int RTOS_SetOps(const RTOS_Ops_t *ops);

/**
 * @brief 获取 RTOS 操作接口
 * @return RTOS 操作接口指针
 */
const RTOS_Ops_t *RTOS_GetOps(void);
extern const RTOS_Ops_t FreeRTOS_Ops;
#endif // RTOS_ABSTRACTION_H
