/**
 * =====================================================================================
 * @file        rtos_abstraction.c
 * @brief       RTOS 抽象层针对 FreeRTOS 的实现 (已优化并补全)
 * @author      23Elapse & Gemini
 * @version     2.2 (Completed & Corrected)
 * @date        2025-06-08
 * =====================================================================================
 */
#include "rtos_abstraction.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <stdlib.h>

// 全局指针定义
RTOS_Ops_t *g_rtos_ops = NULL;

// --- 静态包装函数 (Static Wrapper Functions for FreeRTOS) ---

static void* FreeRTOS_SemaphoreCreate(void) { return xSemaphoreCreateMutex(); }
static void FreeRTOS_SemaphoreDelete(void* sem) { if(sem) vSemaphoreDelete((SemaphoreHandle_t)sem); }
static int FreeRTOS_SemaphoreTake(void* sem, uint32_t timeout) { if (!sem) return 0; return xSemaphoreTake((SemaphoreHandle_t)sem, (TickType_t)timeout) == pdTRUE ? 1 : 0; }
static void FreeRTOS_SemaphoreGive(void* sem) { if(sem) xSemaphoreGive((SemaphoreHandle_t)sem); }
static int FreeRTOS_SemaphoreGiveFromISR(void* sem, void* xHigherPriorityTaskWoken) {
    BaseType_t woken = pdFALSE;
    if (!sem) return 0;
    int ret = xSemaphoreGiveFromISR((SemaphoreHandle_t)sem, &woken);
    if (xHigherPriorityTaskWoken) {
        *(BaseType_t *)xHigherPriorityTaskWoken = woken;
    }
    return ret == pdTRUE ? 1 : 0;
}
static void FreeRTOS_YieldFromISR(void *xHigherPriorityTaskWoken) { if (xHigherPriorityTaskWoken && *(BaseType_t *)xHigherPriorityTaskWoken) portYIELD_FROM_ISR(*(BaseType_t *)xHigherPriorityTaskWoken); }
static uint32_t FreeRTOS_GetTickCount(void) { return xTaskGetTickCount(); }
static void FreeRTOS_Delay(uint32_t ticks) { vTaskDelay((TickType_t)ticks); }
static void* FreeRTOS_TaskCreate(void (*task_func)(void *), const char *name, uint32_t stack_size, void *param, uint32_t priority) {
    TaskHandle_t task;
    BaseType_t ret = xTaskCreate(task_func, name, (configSTACK_DEPTH_TYPE)stack_size, param, (UBaseType_t)priority, &task);
    return ret == pdPASS ? task : NULL;
}
static void FreeRTOS_TaskDelete(void *task) { if(task) vTaskDelete((TaskHandle_t)task); }
static void* FreeRTOS_Malloc(size_t size) { return pvPortMalloc(size); }
static void FreeRTOS_Free(void *ptr) { if(ptr) vPortFree(ptr); }
static void FreeRTOS_TaskStartScheduler(void) { vTaskStartScheduler(); }
static void* FreeRTOS_CreateQueueMutex(void) { return xSemaphoreCreateMutex(); }
static void FreeRTOS_DeleteQueueMutex(void* mutex) { if (mutex) vSemaphoreDelete((SemaphoreHandle_t)mutex); } // 补全缺失的实现
static void* FreeRTOS_CreateQueue(size_t item_size, size_t queue_length) { return xQueueCreate((UBaseType_t)queue_length, (UBaseType_t)item_size); }
static void FreeRTOS_DeleteQueue(void* queue) { if(queue) vQueueDelete((QueueHandle_t)queue); }
static uint8_t FreeRTOS_ReceiveQueue(void* queue, void* item, uint32_t timeout) { if (!queue) return 0; return xQueueReceive((QueueHandle_t)queue, item, (TickType_t)timeout) == pdTRUE ? 1 : 0; }
static uint8_t FreeRTOS_SendQueue(void* queue, const void* item, uint32_t timeout) { if (!queue) return 0; return xQueueSend((QueueHandle_t)queue, item, (TickType_t)timeout) == pdTRUE ? 1 : 0; }
static uint8_t FreeRTOS_SendQueueFromISR(void *queue, const void *item, void *xHigherPriorityTaskWoken) {
    BaseType_t woken = pdFALSE;
    if (!queue) return 0;
    int ret = xQueueSendFromISR((QueueHandle_t)queue, item, &woken);
    if (xHigherPriorityTaskWoken) {
        *(BaseType_t *)xHigherPriorityTaskWoken = woken;
    }
    return ret == pdTRUE ? 1 : 0;
}
static uint8_t FreeRTOS_ReceiveQueueFromISR(void *queue, void *item, void *xHigherPriorityTaskWoken) {
    BaseType_t woken = pdFALSE;
    if (!queue) return 0;
    int ret = xQueueReceiveFromISR((QueueHandle_t)queue, item, &woken);
    if (xHigherPriorityTaskWoken) {
        *(BaseType_t *)xHigherPriorityTaskWoken = woken;
    }
    return ret == pdTRUE ? 1 : 0;
}
static void FreeRTOS_QueueGive(void* queue) { /* 此函数在FreeRTOS中没有直接对应，保留为空以兼容旧接口 */ }
static uint8_t FreeRTOS_QueueTake(void *queue, uint32_t timeout) { if (!queue) return 0; return xQueueSemaphoreTake((QueueHandle_t)queue, (TickType_t)timeout) == pdTRUE ? 1 : 0; }
static void FreeRTOS_SuspendTask(void *task) { if(task) vTaskSuspend((TaskHandle_t)task); }
static void FreeRTOS_ResumeTask(void *task) { if(task) vTaskResume((TaskHandle_t)task); }

// --- 新增的函数实现 ---
static void* FreeRTOS_SemaphoreCreateCounting(uint32_t max_count, uint32_t initial_count) {
    return xSemaphoreCreateCounting((UBaseType_t)max_count, (UBaseType_t)initial_count);
}


// --- 定义全局唯一的 RTOS 操作接口实例 ---
RTOS_Ops_t FreeRTOS_Ops = {
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
    .CreateQueueMutex = FreeRTOS_CreateQueueMutex,
    .DeleteQueueMutex = FreeRTOS_DeleteQueueMutex, // 补全缺失的初始化
    .CreateQueue = FreeRTOS_CreateQueue,
    .DeleteQueue = FreeRTOS_DeleteQueue,
    .ReceiveQueue = FreeRTOS_ReceiveQueue,
    .SendQueue = FreeRTOS_SendQueue,
    .SendQueueFromISR = FreeRTOS_SendQueueFromISR,
    .ReceiveQueueFromISR = FreeRTOS_ReceiveQueueFromISR,
    .QueueGive = FreeRTOS_QueueGive,
    .QueueTake = FreeRTOS_QueueTake,
    .Task_Suspend = FreeRTOS_SuspendTask,
    .Task_Resume = FreeRTOS_ResumeTask,
    // --- 新增的功能 ---
    .SemaphoreCreateCounting = FreeRTOS_SemaphoreCreateCounting,
};
