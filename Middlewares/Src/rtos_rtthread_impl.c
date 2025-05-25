/*
 * @File: rtos_rtthread_impl.c
 * @Description: RT-Thread 环境下对 RTOS_Ops_t 的实现
 * @Date: 2025-04-29
 */

#include "rtos_abstraction.h"
#ifdef RT_THREAD
#include "rtthread.h"

/**
 * @brief  创建信号量
 * @retval 信号量句柄，失败返回 NULL
 */
static void *RTThread_SemaphoreCreate(void)
{
    return rt_mutex_create("serial", RT_IPC_FLAG_FIFO);
}

/**
 * @brief  删除信号量
 * @param  sem: 信号量句柄
 */
static void RTThread_SemaphoreDelete(void *sem)
{
    if (sem)
    {
        rt_mutex_delete((rt_mutex_t)sem);
    }
}

/**
 * @brief  获取信号量
 * @param  sem: 信号量句柄
 * @param  timeout: 超时时间（ticks）
 * @retval 1: 成功, 0: 失败
 */
static int RTThread_SemaphoreTake(void *sem, uint32_t timeout)
{
    if (!sem)
        return 0;
    return rt_mutex_take((rt_mutex_t)sem, timeout) == RT_EOK ? 1 : 0;
}

/**
 * @brief  释放信号量
 * @param  sem: 信号量句柄
 */
static void RTThread_SemaphoreGive(void *sem)
{
    if (sem)
    {
        rt_mutex_release((rt_mutex_t)sem);
    }
}

/**
 * @brief  中断中释放信号量
 * @param  sem: 信号量句柄
 * @param  xHigherPriorityTaskWoken: 任务切换标志（RT-Thread 中使用 rt_bool_t）
 * @retval 1: 成功, 0: 失败
 */
static int RTThread_SemaphoreGiveFromISR(void *sem, void *xHigherPriorityTaskWoken)
{
    if (!sem)
        return 0;
    rt_bool_t need_schedule = RT_FALSE;
    rt_err_t ret = rt_mutex_release((rt_mutex_t)sem); // RT-Thread 中断中释放信号量
    if (ret == RT_EOK)
    {
        need_schedule = RT_TRUE;
    }
    if (xHigherPriorityTaskWoken)
    {
        *(rt_bool_t *)xHigherPriorityTaskWoken = need_schedule;
    }
    return ret == RT_EOK ? 1 : 0;
}

/**
 * @brief  中断中任务切换
 * @param  xHigherPriorityTaskWoken: 任务切换标志
 */
static void RTThread_YieldFromISR(void *xHigherPriorityTaskWoken)
{
    if (xHigherPriorityTaskWoken && *(rt_bool_t *)xHigherPriorityTaskWoken)
    {
        rt_schedule(); // RT-Thread 中断调度
    }
}

/**
 * @brief  获取当前 tick 计数
 * @retval 当前 tick 值
 */
static uint32_t RTThread_GetTickCount(void)
{
    return rt_tick_get();
}

/**
 * @brief  任务延时
 * @param  ticks: 延时时间（ticks）
 */
static void RTThread_Delay(uint32_t ticks)
{
    rt_thread_mdelay(ticks); // RT-Thread 延时（毫秒）
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
static void *RTThread_TaskCreate(void (*task_func)(void *), const char *name, uint32_t stack_size, void *param, uint32_t priority)
{
    rt_thread_t task = rt_thread_create(name, task_func, param, stack_size, priority, 10);
    if (task)
    {
        rt_thread_startup(task);
    }
    return task;
}

/**
 * @brief  删除任务
 * @param  task: 任务句柄
 */
static void RTThread_TaskDelete(void *task)
{
    if (task)
    {
        rt_thread_delete((rt_thread_t)task);
    }
}

/**
 * @brief  RT-Thread 的 RTOS_Ops_t 实现
 */
const RTOS_Ops_t RTThread_Ops = {
    .SemaphoreCreate = RTThread_SemaphoreCreate,
    .SemaphoreDelete = RTThread_SemaphoreDelete,
    .SemaphoreTake = RTThread_SemaphoreTake,
    .SemaphoreGive = RTThread_SemaphoreGive,
    .SemaphoreGiveFromISR = RTThread_SemaphoreGiveFromISR,
    .YieldFromISR = RTThread_YieldFromISR,
    .GetTickCount = RTThread_GetTickCount,
    .Delay = RTThread_Delay,
    .TaskCreate = RTThread_TaskCreate,
    .TaskDelete = RTThread_TaskDelete};

#endif // RT_THREAD
/*
 * 示例用法：
 * 1. 初始化 RTOS 抽象层
 * RTOS_SetOps(&RTThread_Ops);
 *
 * 2. 使用 RTOS_Ops_t 接口
 * const RTOS_Ops_t *g_rtos_ops = RTOS_GetOps();
 * g_rtos_ops->Delay(100); // 延时 100ms
 *
 * 3. 创建任务
 * void MyTask(void *param) {
 *     while (1) {
 *         // 任务逻辑
 *         g_rtos_ops->Delay(500);
 *     }
 * }
 * g_rtos_ops->TaskCreate(MyTask, "MyTask", 256, NULL, 3);
 */
