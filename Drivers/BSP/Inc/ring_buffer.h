/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-17 07:00:00
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-29 10:01:44
 * @FilePath: \Demo\Drivers\BSP\Inc\ring_buffer.h
 * @Description: 通用环形缓冲区接口，适配FreeRTOS，支持多设备（如CAN、SPI Flash）
 *
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved.
 */

#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
/**
 * @brief  环形缓冲区状态枚举
 */
typedef enum {
    RB_OK = 0,              // 操作成功
    RB_ERROR_NULL_PTR,      // 空指针错误
    RB_ERROR_BUFFER_FULL,   // 缓冲区满
    RB_ERROR_BUFFER_EMPTY,  // 缓冲区空
    RB_ERROR_LOCK_FAILED,   // 互斥锁获取失败
    RB_ERROR_MEMORY         // 内存分配失败
} RB_Status;

/**
 * @brief  环形缓冲区结构体
 */
typedef struct {
    void *buffer;           // 数据缓冲区
    uint32_t item_size;     // 每个元素的大小（字节）
    uint32_t capacity;      // 缓冲区总容量（元素数）
    uint32_t head;          // 写指针
    uint32_t tail;          // 读指针
    uint32_t count;         // 当前元素数量
    SemaphoreHandle_t mutex;// FreeRTOS互斥锁
    SemaphoreHandle_t sem;  // FreeRTOS二值信号量，用于ISR通知
} RingBuffer_t;

/* 函数声明 */
RB_Status RingBuffer_Init(RingBuffer_t *rb, uint32_t capacity, uint32_t item_size);
RB_Status RingBuffer_Deinit(RingBuffer_t *rb);
RB_Status RingBuffer_Write(RingBuffer_t *rb, const void *data);
RB_Status RingBuffer_WriteFromISR(RingBuffer_t *rb, const void *data, BaseType_t *pxHigherPriorityTaskWoken);
RB_Status RingBuffer_Read(RingBuffer_t *rb, void *data);
bool RingBuffer_IsFull(const RingBuffer_t *rb);
bool RingBuffer_IsEmpty(const RingBuffer_t *rb);
bool RingBuffer_IsAvailable(const RingBuffer_t *rb);

#endif /* RING_BUFFER_H */
