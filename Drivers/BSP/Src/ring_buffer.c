#include "ring_buffer.h"
#include "pch.h"

/**
 * @brief 初始化环形缓冲区
 * @param rb           缓冲区指针
 * @param capacity     缓冲区容量（元素数）
 * @param item_size    每个元素的大小（字节）
 * @return RB_Status   状态码
 */
RB_Status RingBuffer_Init(RingBuffer_t *rb, uint32_t capacity, uint32_t item_size)
{
    if (!rb) return RB_ERROR_NULL_PTR;

    rb->buffer = pvPortMalloc(capacity * item_size);
    if (!rb->buffer) return RB_ERROR_MEMORY;

    rb->capacity = capacity;
    rb->item_size = item_size;
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;

    rb->mutex = xSemaphoreCreateMutex();
    if (!rb->mutex) {
        vPortFree(rb->buffer);
        return RB_ERROR_MEMORY;
    }

    rb->sem = xSemaphoreCreateBinary();
    if (!rb->sem) {
        vSemaphoreDelete(rb->mutex);
        vPortFree(rb->buffer);
        return RB_ERROR_MEMORY;
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
    if (!rb) return RB_ERROR_NULL_PTR;

    if (rb->mutex) vSemaphoreDelete(rb->mutex);
    if (rb->sem) vSemaphoreDelete(rb->sem);
    if (rb->buffer) vPortFree(rb->buffer);

    memset(rb, 0, sizeof(RingBuffer_t));
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
    if (!rb || !data) return RB_ERROR_NULL_PTR;

    if (xSemaphoreTake(rb->mutex, portMAX_DELAY) != pdTRUE) {
        return RB_ERROR_LOCK_FAILED;
    }

    if (RingBuffer_IsFull(rb)) {
        xSemaphoreGive(rb->mutex);
        return RB_ERROR_BUFFER_FULL;
    }

    uint8_t *dest = (uint8_t *)rb->buffer + (rb->head * rb->item_size);
    memcpy(dest, data, rb->item_size);

    rb->head = (rb->head + 1) % rb->capacity;
    rb->count++;
    xSemaphoreGive(rb->mutex);

    return RB_OK;
}

/**
 * @brief 从中断写入数据到缓冲区
 * @param rb   缓冲区指针
 * @param data 待写入数据
 * @param pxHigherPriorityTaskWoken FreeRTOS任务切换标志
 * @return RB_Status
 */
RB_Status RingBuffer_WriteFromISR(RingBuffer_t *rb, const void *data, BaseType_t *pxHigherPriorityTaskWoken)
{
    if (!rb || !data) return RB_ERROR_NULL_PTR;

    if (rb->count == rb->capacity) {
        return RB_ERROR_BUFFER_FULL;
    }

    uint8_t *dest = (uint8_t *)rb->buffer + (rb->head * rb->item_size);
    memcpy(dest, data, rb->item_size);

    rb->head = (rb->head + 1) % rb->capacity;
    rb->count++;
    xSemaphoreGiveFromISR(rb->sem, pxHigherPriorityTaskWoken);

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
    if (!rb || !data) return RB_ERROR_NULL_PTR;

    if (xSemaphoreTake(rb->mutex, portMAX_DELAY) != pdTRUE) {
        return RB_ERROR_LOCK_FAILED;
    }

    if (RingBuffer_IsEmpty(rb)) {
        xSemaphoreGive(rb->mutex);
        return RB_ERROR_BUFFER_EMPTY;
    }

    uint8_t *src = (uint8_t *)rb->buffer + (rb->tail * rb->item_size);
    memcpy(data, src, rb->item_size);

    rb->tail = (rb->tail + 1) % rb->capacity;
    rb->count--;
    xSemaphoreGive(rb->mutex);

    return RB_OK;
}

/**
 * @brief 判断缓冲区是否已满
 * @param rb 缓冲区指针
 * @return true/false
 */
bool RingBuffer_IsFull(const RingBuffer_t *rb)
{
    if (!rb) return false;
    return (rb->count == rb->capacity);
}

/**
 * @brief 判断缓冲区是否为空
 * @param rb 缓冲区指针
 * @return true/false
 */
bool RingBuffer_IsEmpty(const RingBuffer_t *rb)
{
    if (!rb) return true;
    return (rb->count == 0);
}

/**
 * @brief 判断缓冲区是否有数据可读
 * @param rb 缓冲区指针
 * @return true/false
 */
bool RingBuffer_IsAvailable(const RingBuffer_t *rb)
{
    if (!rb) return false;
    return !RingBuffer_IsEmpty(rb);
}

