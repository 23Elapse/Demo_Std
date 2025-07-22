/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 20:01:43
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-14 18:00:00
 * @FilePath: \Demo\Drivers\BSP\Src\can_driver.c
 * @Description: CAN 驱动实现 (Refactored)
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "can_driver.h"
#include "rtos_abstraction.h"
#include "log_system.h"
#include "common_driver.h" // For Common_GPIO_Init
#include "pch.h"
#include <string.h>

// 存储已注册的CAN设备实例，用于中断处理
// 通常只有CAN1和CAN2两个实例
static CAN_Device_t *s_can_devices[2] = {NULL, NULL};
static uint8_t s_can_device_count = 0;

/**
 * @brief 根据波特率设置 CAN_Prescaler 和时间段
 * @param baudrate 期望的波特率
 * @param p_prescaler 指向预分频器的指针
 * @param p_ts1 指向时间段1的指针
 * @param p_ts2 指向时间段2的指针
 * @return CAN_Status_t 配置是否成功
 */
static CAN_Status_t CAN_SetBaudrateParams(uint32_t baudrate, uint16_t *p_prescaler, uint8_t *p_ts1, uint8_t *p_ts2)
{
    // 假设APB1时钟为42MHz (STM32F4系列通常是这个频率)
    // CAN_CLK = APB1_CLK / Prescaler
    // NominalBitTime = (1 + TS1 + TS2) * Tq
    // Tq = 1 / CAN_CLK
    // BitRate = 1 / NominalBitTime
    // 简化计算，直接查找表或根据常用配置。这里提供一个示例配置。
    // For 42MHz APB1 clock:
    // 500kbps: Prescaler=6, TS1=10, TS2=3  -> Tq = 1/7MHz, BitTime = 14 Tq = 2us
    // 250kbps: Prescaler=12, TS1=10, TS2=3 -> Tq = 1/3.5MHz, BitTime = 14 Tq = 4us
    // 125kbps: Prescaler=24, TS1=10, TS2=3 -> Tq = 1/1.75MHz, BitTime = 14 Tq = 8us

    switch (baudrate) {
        case 1000000: *p_prescaler = 3; *p_ts1 = CAN_BS1_10tq; *p_ts2 = CAN_BS2_3tq; break; // 1Mbps
        case 500000:  *p_prescaler = 6; *p_ts1 = CAN_BS1_10tq; *p_ts2 = CAN_BS2_3tq; break; // 500Kbps
        case 250000:  *p_prescaler = 12; *p_ts1 = CAN_BS1_10tq; *p_ts2 = CAN_BS2_3tq; break; // 250Kbps
        case 125000:  *p_prescaler = 24; *p_ts1 = CAN_BS1_10tq; *p_ts2 = CAN_BS2_3tq; break; // 125Kbps
        case 100000:  *p_prescaler = 30; *p_ts1 = CAN_BS1_10tq; *p_ts2 = CAN_BS2_3tq; break; // 100Kbps
        case 50000:   *p_prescaler = 60; *p_ts1 = CAN_BS1_10tq; *p_ts2 = CAN_BS2_3tq; break; // 50Kbps
        default:
            Log_Message(LOG_LEVEL_ERROR, "[CAN] Unsupported baudrate: %lu", baudrate);
            return CAN_ERROR_PARAM;
    }
    return CAN_OK;
}

/**
 * @brief 初始化 CAN 设备
 * @param dev CAN 设备实例
 * @return CAN_Status_t 操作状态
 */
static CAN_Status_t CAN_Driver_Init(CAN_Device_t *dev)
{
    if (!dev || !dev->instance || !dev->tx_port || !dev->rx_port) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] CAN_Driver_Init: Invalid device parameters.");
        return CAN_ERROR_PARAM;
    }

    if (s_can_device_count >= 2) { // 限制最多两个CAN设备
        Log_Message(LOG_LEVEL_ERROR, "[CAN] CAN_Driver_Init: Max CAN devices reached.");
        return CAN_ERROR_INIT;
    }

    // 1. 初始化 GPIO
    if (Common_GPIO_Init(dev->tx_port, dev->tx_pin, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Failed to init TX GPIO.");
        return CAN_ERROR_INIT;
    }
    if (Common_GPIO_Init(dev->rx_port, dev->rx_pin, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Failed to init RX GPIO.");
        return CAN_ERROR_INIT;
    }

    // 2. 使能 CAN 时钟
    if (dev->instance == CAN1) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    } else if (dev->instance == CAN2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
    } else {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Invalid CAN instance.");
        return CAN_ERROR_PARAM;
    }

    // 3. 配置 CAN 波特率参数
    uint16_t prescaler;
    uint8_t ts1, ts2;
    if (CAN_SetBaudrateParams(dev->baudrate, &prescaler, &ts1, &ts2) != CAN_OK) {
        return CAN_ERROR_INIT;
    }

    // 4. 初始化 CAN 核心配置
    CAN_InitTypeDef can_init = {0};
    can_init.CAN_Prescaler = prescaler;
    can_init.CAN_Mode = CAN_Mode_LoopBack; // 正常工作模式
    can_init.CAN_SJW = CAN_SJW_1tq;
    can_init.CAN_BS1 = ts1;
    can_init.CAN_BS2 = ts2;
    can_init.CAN_TTCM = DISABLE; // 时间触发通信模式禁用
    can_init.CAN_ABOM = DISABLE; // 自动总线关闭管理禁用
    can_init.CAN_AWUM = DISABLE; // 自动唤醒模式禁用
    can_init.CAN_NART = DISABLE; // 自动重传禁用 (通常发送失败会重试，这里可以根据需求决定)
    can_init.CAN_RFLM = DISABLE; // FIFO 锁定模式禁用
    can_init.CAN_TXFP = DISABLE; // 发送 FIFO 优先级禁用
    CAN_Init(dev->instance, &can_init);

    // 5. 配置 CAN 过滤器 (接收所有消息的简单配置)
    CAN_FilterInitTypeDef filter_init = {0};
    filter_init.CAN_FilterNumber = 0; // 使用过滤器0
    filter_init.CAN_FilterMode = CAN_FilterMode_IdMask; // 标识符屏蔽位模式
    filter_init.CAN_FilterScale = CAN_FilterScale_32bit; // 32位宽度
    filter_init.CAN_FilterIdHigh = 0x0000;              // 接收所有ID (或设置特定ID)
    filter_init.CAN_FilterIdLow = 0x0000;
    filter_init.CAN_FilterMaskIdHigh = 0x0000;          // 屏蔽所有位，即不筛选
    filter_init.CAN_FilterMaskIdLow = 0x0000;
    filter_init.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; // 消息存储到 FIFO0
    filter_init.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter_init);

    // 6. 初始化接收环形缓冲区
    if (RingBuffer_Init(&dev->rx_buffer, CAN_RX_BUFFER_CAPACITY, sizeof(CAN_Message_t)) != RB_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Failed to init RX buffer.");
        return CAN_ERROR_INIT;
    }

    // 7. 配置并使能中断
    CAN_ITConfig(dev->instance, CAN_IT_FMP0, ENABLE); // 使能 FIFO0 消息挂起中断
    NVIC_InitTypeDef nvic_init = {
        .NVIC_IRQChannel = dev->irqn,
        .NVIC_IRQChannelPreemptionPriority = 5, // 优先级可根据系统调整
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE
    };
    NVIC_Init(&nvic_init);

    // 8. 创建互斥锁 (如果需要)
    if (dev->mutex == NULL) {
        dev->mutex = g_rtos_ops->SemaphoreCreate();
        if (dev->mutex == NULL) {
            Log_Message(LOG_LEVEL_ERROR, "[CAN] Failed to create mutex for CAN.");
            return CAN_ERROR_INIT;
        }
    }

    // 9. 注册设备实例到静态数组，以便中断处理函数查找
    s_can_devices[s_can_device_count++] = dev;

    Log_Message(LOG_LEVEL_INFO, "[CAN] Initialized successfully. Instance: %p", dev->instance);
    return CAN_OK;
}

/**
 * @brief 反初始化 CAN 设备
 * @param dev CAN 设备实例
 * @return CAN_Status_t 操作状态
 */
static CAN_Status_t CAN_Driver_Deinit(CAN_Device_t *dev)
{
    if (!dev || !dev->instance) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] CAN_Driver_Deinit: Invalid device.");
        return CAN_ERROR_PARAM;
    }

    CAN_ITConfig(dev->instance, CAN_IT_FMP0, DISABLE); // 禁用中断
    CAN_DeInit(dev->instance); // 反初始化 CAN 外设

    if (RingBuffer_Deinit(&dev->rx_buffer) != RB_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Failed to deinit RX buffer.");
        return CAN_ERROR_INIT; // 应该不会失败，除非缓冲区未初始化
    }

    // 释放互斥锁
    if (dev->mutex != NULL) {
        g_rtos_ops->SemaphoreDelete(dev->mutex);
        dev->mutex = NULL;
    }

    // 从已注册设备列表中移除
    for (uint8_t i = 0; i < s_can_device_count; i++) {
        if (s_can_devices[i] == dev) {
            s_can_devices[i] = s_can_devices[--s_can_device_count]; // 移动最后一个到当前位置
            s_can_devices[s_can_device_count] = NULL; // 清空最后一个
            break;
        }
    }

    Log_Message(LOG_LEVEL_INFO, "[CAN] Deinitialized successfully. Instance: %p", dev->instance);
    return CAN_OK;
}

/**
 * @brief 发送 CAN 消息
 * @param dev CAN 设备实例
 * @param msg CAN 消息
 * @return CAN_Status_t 操作状态
 */
static CAN_Status_t CAN_Driver_SendMessage(CAN_Device_t *dev, const CAN_Message_t *msg)
{
    if (!dev || !msg || msg->length > 8) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] CAN_Driver_SendMessage: Invalid message or length.");
        return CAN_ERROR_PARAM;
    }

    // 尝试获取互斥锁以保护发送过程 (如果支持多任务发送)
    if (dev->mutex && !g_rtos_ops->SemaphoreTake(dev->mutex, 100)) { // 100ms 超时
        Log_Message(LOG_LEVEL_ERROR, "[CAN] SendMessage: Failed to take mutex.");
        return CAN_ERROR_MUTEX_TIMEOUT;
    }

    CanTxMsg tx_msg = {0};
    tx_msg.StdId = msg->id;
    tx_msg.IDE = msg->ide;
    tx_msg.RTR = msg->rtr;
    tx_msg.DLC = msg->length;
    memcpy(tx_msg.Data, msg->data, msg->length);

    uint8_t mailbox = CAN_Transmit(dev->instance, &tx_msg);
    if (mailbox == CAN_TxStatus_NoMailBox) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] SendMessage: No mailbox available.");
        if (dev->mutex) g_rtos_ops->SemaphoreGive(dev->mutex);
        return CAN_ERROR_TRANSMIT;
    }

    // 等待发送完成
    uint32_t timeout = 1000000; // 假设一个超时计数
    while (CAN_TransmitStatus(dev->instance, mailbox) != CAN_TxStatus_Ok && timeout--) {
        // 可以添加延时或让出CPU
        // g_rtos_ops->Delay(1); // 避免忙等
    }

    if (timeout == 0) {
        CAN_CancelTransmit(dev->instance, mailbox);
        Log_Message(LOG_LEVEL_ERROR, "[CAN] SendMessage: Transmit timeout.");
        if (dev->mutex) g_rtos_ops->SemaphoreGive(dev->mutex);
        return CAN_ERROR_TRANSMIT;
    }

    if (dev->mutex) g_rtos_ops->SemaphoreGive(dev->mutex);
    Log_Message(LOG_LEVEL_INFO, "[CAN] Message sent successfully. ID: 0x%lX, Len: %u", msg->id, msg->length);
    return CAN_OK;
}

/**
 * @brief 接收 CAN 消息
 * @param dev CAN 设备实例
 * @param msg CAN 消息存储指针
 * @param timeout_ms 超时时间（毫秒）
 * @return CAN_Status_t 操作状态
 */
static CAN_Status_t CAN_Driver_ReceiveMessage(CAN_Device_t *dev, CAN_Message_t *msg, uint32_t timeout_ms)
{
    if (!dev || !msg) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] CAN_Driver_ReceiveMessage: Invalid device or message buffer.");
        return CAN_ERROR_PARAM;
    }

    RB_Status rb_status;
    if (g_rtos_ops->SemaphoreTake(dev->rx_buffer.sem, timeout_ms)) { // 使用环形缓冲区的信号量等待数据
        rb_status = RingBuffer_Read(&dev->rx_buffer, msg);
        if (rb_status == RB_OK) {
            // Log_Message(LOG_LEVEL_INFO, "[CAN] Message received. ID: 0x%lX", msg->id); // 太频繁会影响性能
            return CAN_OK;
        } else {
            // 理论上如果信号量成功，这里不会是 RB_ERROR_BUFFER_EMPTY
            Log_Message(LOG_LEVEL_WARNING, "[CAN] ReceiveMessage: Ring buffer read error: %d", rb_status);
            return (rb_status == RB_ERROR_BUFFER_EMPTY) ? CAN_ERROR_NO_DATA : CAN_ERROR_BUFFER_FULL;
        }
    } else {
        Log_Message(LOG_LEVEL_DEBUG, "[CAN] ReceiveMessage: No message received within timeout (%lu ms).", timeout_ms);
        return CAN_ERROR_NO_DATA;
    }
}

/**
 * @brief CAN 中断服务总处理函数
 * @param dev 发生中断的CAN设备实例
 */
void CAN_IRQHandler(CAN_Device_t *dev)
{
    if (dev == NULL) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] CAN_IRQHandler: NULL device pointer.");
        return;
    }

    // 检查是否有消息挂起在 FIFO0
    if (CAN_GetITStatus(dev->instance, CAN_IT_FMP0) != RESET) {
        CanRxMsg rx_msg = {0};
        CAN_Receive(dev->instance, CAN_FIFO0, &rx_msg); // 从 FIFO0 读取消息

        CAN_Message_t msg = {0};
        msg.id = (rx_msg.IDE == CAN_Id_Standard) ? rx_msg.StdId : rx_msg.ExtId;
        msg.length = rx_msg.DLC;
        msg.ide = rx_msg.IDE;
        msg.rtr = rx_msg.RTR;
        memcpy(msg.data, rx_msg.Data, rx_msg.DLC);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE; // 用于FreeRTOS任务切换

        // 将接收到的消息写入环形缓冲区
        RB_Status rb_status = RingBuffer_WriteFromISR(&dev->rx_buffer, &msg, &xHigherPriorityTaskWoken);

        if (rb_status != RB_OK) {
            Log_Message(LOG_LEVEL_WARNING, "[CAN] IRQ: RX buffer full or error: %d", rb_status);
        }

        // 清除中断挂起位
        CAN_ClearITPendingBit(dev->instance, CAN_IT_FMP0);

        // 如果有更高优先级的任务被唤醒，请求上下文切换
        if (g_rtos_ops && xHigherPriorityTaskWoken == pdTRUE) {
            g_rtos_ops->YieldFromISR(xHigherPriorityTaskWoken);
        }
    }
    // 可以在这里处理其他CAN中断，如FIFO1、错误中断等
}

// 定义全局唯一的CAN操作接口实例
const CAN_Ops_t g_can_ops = {
    .Init = CAN_Driver_Init,
    .Deinit = CAN_Driver_Deinit,
    .SendMessage = CAN_Driver_SendMessage,
    .ReceiveMessage = CAN_Driver_ReceiveMessage
};
