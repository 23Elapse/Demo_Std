#include "ring_buffer.h"
#include "pch.h"

/**
 * @brief 初始化环形缓冲区
 * @param rb           缓冲区指针
 * @param capacity     缓冲区容量（元素数）
 * @param element_size    每个元素的大小（字节）
 * @return RB_Status   状态码
 */
RB_Status RingBuffer_Init(RingBuffer_t *rb, uint32_t capacity, uint32_t element_size)
{
    if (!rb || capacity == 0 || element_size == 0)
        return RB_ERROR_INIT;

    rb->buffer = (uint8_t *)g_rtos_ops->Malloc(capacity * element_size);
    if (!rb->buffer)
        return RB_ERROR_INIT;

    rb->element_size = element_size;
    rb->capacity = capacity;
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;

    rb->sem = g_rtos_ops->SemaphoreCreate();
    if (!rb->sem)
    {
        g_rtos_ops->Free(rb->buffer);
        return RB_ERROR_INIT;
    }

    return RB_OK;
}

/**
 * @brief 释放缓冲区资源
 * @param rb 缓冲区指针
 * @return RB_Status
 */
RB_Status RingBuffer_Deinit(RingBuffer_t *rb)
{
    if (!rb || !rb->buffer)
        return RB_ERROR_INIT;

    g_rtos_ops->Free(rb->buffer);
    rb->buffer = NULL;
    rb->element_size = 0;
    rb->capacity = 0;
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;

    if (rb->sem)
    {
        g_rtos_ops->SemaphoreDelete(rb->sem);
        rb->sem = NULL;
    }

    return RB_OK;
}

/**
 * @brief 写入数据到缓冲区
 * @param rb   缓冲区指针
 * @param data 待写入数据
 * @return RB_Status
 */
RB_Status RingBuffer_Write(RingBuffer_t *rb, const void *data)
{
    if (!rb || !rb->buffer || !data)
        return RB_ERROR_INIT;

    if (rb->count >= rb->capacity)
        return RB_ERROR_BUFFER_FULL;

    memcpy(rb->buffer + rb->tail * rb->element_size, data, rb->element_size);
    rb->tail = (rb->tail + 1) % rb->capacity;
    rb->count++;

    return RB_OK;
}

/**
 * @brief 从中断写入数据到缓冲区
 * @param rb   缓冲区指针
 * @param data 待写入数据
 * @param xHigherPriorityTaskWoken FreeRTOS任务切换标志
 * @return RB_Status
 */
RB_Status RingBuffer_WriteFromISR(RingBuffer_t *rb, const void *data, void *xHigherPriorityTaskWoken)
{
    if (!rb || !rb->buffer || !data)
        return RB_ERROR_INIT;

    if (rb->count >= rb->capacity)
        return RB_ERROR_BUFFER_FULL;

    memcpy(rb->buffer + rb->tail * rb->element_size, data, rb->element_size);
    rb->tail = (rb->tail + 1) % rb->capacity;
    rb->count++;

    if (rb->sem)
    {
        g_rtos_ops->SemaphoreGiveFromISR(rb->sem, xHigherPriorityTaskWoken);
    }

    return RB_OK;
}

/**
 * @brief 从缓冲区读取数据
 * @param rb   缓冲区指针
 * @param data 读取数据存储地址
 * @return RB_Status
 */
RB_Status RingBuffer_Read(RingBuffer_t *rb, void *data)
{
    if (!rb || !rb->buffer || !data)
        return RB_ERROR_INIT;

    if (rb->count == 0)
        return RB_ERROR_BUFFER_EMPTY;

    memcpy(data, rb->buffer + rb->head * rb->element_size, rb->element_size);
    rb->head = (rb->head + 1) % rb->capacity;
    rb->count--;

    return RB_OK;
}

/**
 * @brief 判断缓冲区是否已满
 * @param rb 缓冲区指针
 * @return true/false
 */
bool RingBuffer_IsFull(const RingBuffer_t *rb)
{
    if (!rb)
        return false;
    return (rb->count == rb->capacity);
}

/**
 * @brief 判断缓冲区是否为空
 * @param rb 缓冲区指针
 * @return true/false
 */
bool RingBuffer_IsEmpty(const RingBuffer_t *rb)
{
    if (!rb)
        return true;
    return (rb->count == 0);
}

/**
 * @brief 判断缓冲区是否有数据可读
 * @param rb 缓冲区指针
 * @return true/false
 */
bool RingBuffer_IsAvailable(const RingBuffer_t *rb)
{
    if (!rb)
        return false;
    return !RingBuffer_IsEmpty(rb);
}
