/*
 * @File: rtos_freertos_impl.c
 * @Description: FreeRTOS 环境下对 RTOS_Ops_t 的实现
 * @Date: 2025-04-29
 */

#include "rtos_abstraction.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/**
 * @brief  创建信号量
 * @retval 信号量句柄，失败返回 NULL
 */
static void *FreeRTOS_SemaphoreCreate(void)
{
    return xSemaphoreCreateMutex();
}

/**
 * @brief  删除信号量
 * @param  sem: 信号量句柄
 */
static void FreeRTOS_SemaphoreDelete(void *sem)
{
    if (sem)
    {
        vSemaphoreDelete(sem);
    }
}

/**
 * @brief  获取信号量
 * @param  sem: 信号量句柄
 * @param  timeout: 超时时间（ticks）
 * @retval 1: 成功, 0: 失败
 */
static int FreeRTOS_SemaphoreTake(void *sem, uint32_t timeout)
{
    if (!sem)
        return 0;
    return xSemaphoreTake(sem, timeout) == pdTRUE ? 1 : 0;
}

/**
 * @brief  释放信号量
 * @param  sem: 信号量句柄
 */
static void FreeRTOS_SemaphoreGive(void *sem)
{
    if (sem)
    {
        xSemaphoreGive(sem);
    }
}

/**
 * @brief  中断中释放信号量
 * @param  sem: 信号量句柄
 * @param  xHigherPriorityTaskWoken: 任务切换标志（FreeRTOS 中使用 BaseType_t）
 * @retval 1: 成功, 0: 失败
 */
static int FreeRTOS_SemaphoreGiveFromISR(void *sem, void *xHigherPriorityTaskWoken)
{
    if (!sem)
        return 0;
    BaseType_t woken = pdFALSE;
    int ret = xSemaphoreGiveFromISR(sem, &woken);
    if (xHigherPriorityTaskWoken)
    {
        *(BaseType_t *)xHigherPriorityTaskWoken = woken;
    }
    return ret == pdTRUE ? 1 : 0;
}

/**
 * @brief  中断中任务切换
 * @param  xHigherPriorityTaskWoken: 任务切换标志
 */
static void FreeRTOS_YieldFromISR(void *xHigherPriorityTaskWoken)
{
    if (xHigherPriorityTaskWoken && *(BaseType_t *)xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR(*(BaseType_t *)xHigherPriorityTaskWoken);
    }
}

/**
 * @brief  获取当前 tick 计数
 * @retval 当前 tick 值
 */
static uint32_t FreeRTOS_GetTickCount(void)
{
    return xTaskGetTickCount();
}

/**
 * @brief  任务延时
 * @param  ticks: 延时时间（ticks）
 */
static void FreeRTOS_Delay(uint32_t ticks)
{
    vTaskDelay(ticks);
}

/**
 * @brief  创建任务
 * @param  task_func: 任务函数
 * @param  name: 任务名称
 * @param  stack_size: 堆栈大小
 * @param  param: 任务参数
 * @param  priority: 任务优先级
 * @retval 任务句柄，失败返回 NULL
 */
static void *FreeRTOS_TaskCreate(void (*task_func)(void *), const char *name, uint32_t stack_size, void *param, uint32_t priority)
{
    TaskHandle_t task;
    BaseType_t ret = xTaskCreate(task_func, name, stack_size, param, priority, &task);
    return ret == pdPASS ? task : NULL;
}

/**
 * @brief  删除任务
 * @param  task: 任务句柄
 */
static void FreeRTOS_TaskDelete(void *task)
{
    if (task)
    {
        vTaskDelete((TaskHandle_t)task);
    }
}

/**
 * @brief  动态内存分配
 * @param  size: 分配的内存大小（字节）
 * @retval 分配的内存指针，失败返回 NULL
 */
static void *FreeRTOS_Malloc(size_t size)
{
    return pvPortMalloc(size);
}

/** 
 * @brief  释放动态内存
 * @param  ptr: 要释放的内存指针
 */
static void FreeRTOS_Free(void *ptr)
{
   if (ptr)
   {
       vPortFree(ptr);
   }
}

/**
 * @brief  开始任务调度
 * @note   该函数会启动调度器，开始任务调度
 * @note   该函数会调用 vTaskStartScheduler() 函数
 * @param  无参数
 * @retval 无返回值 
 */
static void FreeRTOS_TaskStartScheduler(void)
{
    vTaskStartScheduler();
}

/**
 * @brief  创建互斥信号量
 * @retval 互斥信号量句柄，失败返回 NULL
 */
static void *FreeRTOS_QueueCreateMutex(void)
{
    return xSemaphoreCreateMutex();
}
/**
 * @brief  接收队列
 * @param  queue: 队列句柄
 * @param  item: 接收的项
 * @param  timeout: 超时时间（ticks）
 * @retval 1: 成功, 0: 失败
 */
static uint8_t FreeRTOS_ReceiveQueue(void *queue, void *item, uint32_t timeout)
{
    if (!queue)
        return 0;
    return xQueueReceive(queue, item, timeout) == pdTRUE ? 1 : 0;
}

/**
 * @brief  发送队列
 * @param  queue: 队列句柄
 * @param  item: 要发送的项
 * @param  timeout: 超时时间（ticks）
 * @retval 1: 成功, 0: 失败
 */
static uint8_t FreeRTOS_SendQueue(void *queue, const void *item, uint32_t timeout)
{
    if (!queue)
        return 0;
    return xQueueSend(queue, item, timeout) == pdTRUE ? 1 : 0;
}

/**
 * @brief  中断中发送队列
 * @param  queue: 队列句柄
 * @param  item: 要发送的项
 * @param  xHigherPriorityTaskWoken: 任务切换标志
 * @retval 1: 成功, 0: 失败
 */
static uint8_t FreeRTOS_SendQueueFromISR(void *queue, const void *item, void *xHigherPriorityTaskWoken)
{
    if (!queue)
        return 0;
    BaseType_t woken = pdFALSE;
    int ret = xQueueSendFromISR(queue, item, &woken);
    if (xHigherPriorityTaskWoken)
    {
        *(BaseType_t *)xHigherPriorityTaskWoken = woken;
    }
    return ret == pdTRUE ? 1 : 0;
}
/**
 * @brief  中断中接收队列
 * @param  queue: 队列句柄
 * @param  item: 接收的项
 * @param  xHigherPriorityTaskWoken: 任务切换标志
 * @retval 1: 成功, 0: 失败
 */
static uint8_t FreeRTOS_ReceiveQueueFromISR(void *queue, void *item, void *xHigherPriorityTaskWoken)
{
    if (!queue)
        return 0;
    BaseType_t woken = pdFALSE;
    int ret = xQueueReceiveFromISR(queue, item, &woken);
    if (xHigherPriorityTaskWoken)
    {
        *(BaseType_t *)xHigherPriorityTaskWoken = woken;
    }
    return ret == pdTRUE ? 1 : 0;
}
/**
 * @brief  释放队列
 * @param  queue: 队列句柄
 */
static void FreeRTOS_QueueGive(void *queue)
{
    if (queue)
    {
        // xQueueGive(queue);
    }
}
/**
 * @brief  获取队列
 * @param  queue: 队列句柄
 * @param  timeout: 超时时间（ticks）
 * @retval 1: 成功, 0: 失败
 */
static uint8_t FreeRTOS_QueueTake(void *queue, uint32_t timeout)
{
    if (!queue)
        return 0;
    return xQueueSemaphoreTake(queue, timeout) == pdTRUE ? 1 : 0;
}
/**
 * @brief  创建队列
 * @param  queue_size: 队列大小
 * @param  item_size: 每个队列项的大小
 * @retval 队列句柄，失败返回 NULL
 */
static void *FreeRTOS_CreateQueue(size_t item_size, size_t queue_length)
{
    return xQueueCreate(queue_length, item_size);
}
/**
 * @brief  删除队列
 * @param  queue: 队列句柄
 */
static void FreeRTOS_DeleteQueue(void *queue)
{
    if (queue)
    {
        vQueueDelete(queue);
    }
}

/**
 * @brief  FreeRTOS 的 RTOS_Ops_t 实现
 */
const RTOS_Ops_t FreeRTOS_Ops = {
    .SemaphoreCreate = FreeRTOS_SemaphoreCreate,
    .SemaphoreDelete = FreeRTOS_SemaphoreDelete,
    .SemaphoreTake = FreeRTOS_SemaphoreTake,
    .SemaphoreGive = FreeRTOS_SemaphoreGive,
    .SemaphoreGiveFromISR = FreeRTOS_SemaphoreGiveFromISR,
    .YieldFromISR = FreeRTOS_YieldFromISR,
    .GetTickCount = FreeRTOS_GetTickCount,
    .Delay = FreeRTOS_Delay,
    .TaskCreate = FreeRTOS_TaskCreate,
    .TaskDelete = FreeRTOS_TaskDelete,
    .Malloc = FreeRTOS_Malloc,
    .Free = FreeRTOS_Free,
    .TaskStartScheduler = FreeRTOS_TaskStartScheduler,
    .CreateQueue = FreeRTOS_CreateQueue,
    .DeleteQueue = FreeRTOS_DeleteQueue, 
    .ReceiveQueue = FreeRTOS_ReceiveQueue, 
    .SendQueue = FreeRTOS_SendQueue,
    .SendQueueFromISR = FreeRTOS_SendQueueFromISR,
    .ReceiveQueueFromISR = FreeRTOS_ReceiveQueueFromISR,
    .QueueGive = FreeRTOS_QueueGive,
    .QueueTake = FreeRTOS_QueueTake,
    .CreateQueueMutex = FreeRTOS_QueueCreateMutex,
};
