/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-17 07:00:00
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-02 21:23:27
 * @FilePath: \Demo\Drivers\BSP\Inc\ring_buffer.h
 * @Description: 通用环形缓冲区接口，适配 RTOS 抽象，支持多设备
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include "rtos_abstraction.h"

typedef enum
{
    RB_OK = 0,
    RB_ERROR_INIT,
    RB_ERROR_BUFFER_FULL,
    RB_ERROR_BUFFER_EMPTY
} RB_Status;

typedef struct
{
    uint8_t *buffer;
    uint32_t element_size;
    uint32_t capacity;
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    void *sem;
} RingBuffer_t;

RB_Status RingBuffer_Init(RingBuffer_t *rb, uint32_t capacity, uint32_t element_size);
RB_Status RingBuffer_Deinit(RingBuffer_t *rb);
RB_Status RingBuffer_Write(RingBuffer_t *rb, const void *data);
RB_Status RingBuffer_Read(RingBuffer_t *rb, void *data);
RB_Status RingBuffer_WriteFromISR(RingBuffer_t *rb, const void *data, void *xHigherPriorityTaskWoken);

bool RingBuffer_IsFull(const RingBuffer_t *rb);
bool RingBuffer_IsEmpty(const RingBuffer_t *rb);
bool RingBuffer_IsAvailable(const RingBuffer_t *rb);

#endif /* RING_BUFFER_H */