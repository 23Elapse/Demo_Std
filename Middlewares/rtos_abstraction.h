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
    void (*TaskStartScheduler)(void);                                                                                      // 开始任务调度
    void *(*CreateQueueMutex)(void);                                                                                       // 创建互斥信号量
    void (*DeleteQueueMutex)(void *mutex);                                                                                 // 删除互斥信号量
    void *(*CreateQueue)(size_t item_size, size_t queue_length);                                                          // 创建队列
    void (*DeleteQueue)(void *queue);                                                                                     // 删除队列
    uint8_t (*ReceiveQueue)(void *queue, void *item, uint32_t timeout);                                                      // 接收队列
    uint8_t (*SendQueue)(void *queue, const void *item, uint32_t timeout);                                                         // 发送队列
    uint8_t (*SendQueueFromISR)(void *queue, const void *item, void *xHigherPriorityTaskWoken);                                     // 中断中发送队列
    uint8_t (*ReceiveQueueFromISR)(void *queue, void *item, void *xHigherPriorityTaskWoken);                                   // 中断中接收队列
    void (*QueueGive)(void *queue);                                                                                        // 释放队列
    uint8_t (*QueueTake)(void *queue, uint32_t timeout);                                                                     // 获取队列
    void (*Task_Suspend)(void *task);                                                                                       // 挂起任务
    void (*Task_Resume)(void *task);                                                                                        // 恢复任务
} RTOS_Ops_t;

// 全局 RTOS 操作接口
extern RTOS_Ops_t *g_rtos_ops;

// FreeRTOS 操作接口实例
extern RTOS_Ops_t FreeRTOS_Ops;

#endif // RTOS_ABSTRACTION_H
