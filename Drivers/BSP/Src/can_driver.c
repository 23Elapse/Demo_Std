/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 20:01:43
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 00:54:08
 * @FilePath: \Demo\Drivers\BSP\Src\can_driver.c
 * @Description: CAN 驱动实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "can_driver.h"
#include "rtos_abstraction.h"
#include "log_system.h"
#include <string.h>
#include "pch.h"
static CAN_Device_t *can_devices[2] = {0};
static uint8_t can_device_count = 0;

/**
 * @brief 初始化 CAN 设备
 * @param dev CAN 设备实例
 * @return CAN_Status 操作状态
 */
CAN_Status CANx_Init(CAN_Device_t *dev)
{
    if (!dev || !dev->instance || can_device_count >= 2) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Invalid device or instance");
        return CAN_ERROR_INIT;
    }

    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] RTOS ops not initialized");
        return CAN_ERROR_INIT;
    }

    if (Common_GPIO_Init(dev->tx_port, dev->tx_pin, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Failed to init TX GPIO");
        return CAN_ERROR_INIT;
    }

    if (Common_GPIO_Init(dev->rx_port, dev->rx_pin, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Failed to init RX GPIO");
        return CAN_ERROR_INIT;
    }

    if (dev->instance == CAN1) RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    else if (dev->instance == CAN2) RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    CAN_InitTypeDef can_init = {0};
    can_init.CAN_Prescaler = 4; // 需根据实际时钟和波特率计算
    can_init.CAN_Mode = CAN_Mode_Normal;
    can_init.CAN_SJW = CAN_SJW_1tq;
    can_init.CAN_BS1 = CAN_BS1_6tq;
    can_init.CAN_BS2 = CAN_BS2_5tq;
    can_init.CAN_TTCM = DISABLE;
    can_init.CAN_ABOM = ENABLE;
    can_init.CAN_AWUM = DISABLE;
    can_init.CAN_NART = DISABLE;
    can_init.CAN_RFLM = DISABLE;
    can_init.CAN_TXFP = DISABLE;
    CAN_Init(dev->instance, &can_init);

    CAN_FilterInitTypeDef filter_init = {0};
    filter_init.CAN_FilterNumber = 0;
    filter_init.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter_init.CAN_FilterScale = CAN_FilterScale_32bit;
    filter_init.CAN_FilterIdHigh = 0x0000;
    filter_init.CAN_FilterIdLow = 0x0000;
    filter_init.CAN_FilterMaskIdHigh = 0x0000;
    filter_init.CAN_FilterMaskIdLow = 0x0000;
    filter_init.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    filter_init.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter_init);

    CAN_ITConfig(dev->instance, CAN_IT_FMP0, ENABLE);
    NVIC_InitTypeDef nvic_init = {
        .NVIC_IRQChannel = dev->irqn,
        .NVIC_IRQChannelPreemptionPriority = 0,
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE
    };
    NVIC_Init(&nvic_init);

    if (RingBuffer_Init(&dev->rx_buffer, 16, sizeof(CAN_Message_t)) != RB_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Failed to init RX buffer");
        return CAN_ERROR_INIT;
    }

    can_devices[can_device_count++] = dev;
    Log_Message(LOG_LEVEL_INFO, "[CAN] Initialized successfully");
    return CAN_OK;
}

/**
 * @brief 反初始化 CAN 设备
 * @param dev CAN 设备实例
 * @return CAN_Status 操作状态
 */
CAN_Status CAN_Deinit(CAN_Device_t *dev)
{
    if (!dev) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Invalid device");
        return CAN_ERROR_INIT;
    }

    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] RTOS ops not initialized");
        return CAN_ERROR_INIT;
    }

    CAN_ITConfig(dev->instance, CAN_IT_FMP0, DISABLE);
    CAN_DeInit(dev->instance);

    if (RingBuffer_Deinit(&dev->rx_buffer) != RB_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Failed to deinit RX buffer");
        return CAN_ERROR_INIT;
    }

    for (uint8_t i = 0; i < can_device_count; i++) {
        if (can_devices[i] == dev) {
            can_devices[i] = can_devices[--can_device_count];
            can_devices[can_device_count] = NULL;
            break;
        }
    }

    Log_Message(LOG_LEVEL_INFO, "[CAN] Deinitialized successfully");
    return CAN_OK;
}

/**
 * @brief 发送 CAN 消息
 * @param dev CAN 设备实例
 * @param msg CAN 消息
 * @return CAN_Status 操作状态
 */
CAN_Status CAN_SendMessage(CAN_Device_t *dev, CAN_Message_t *msg)
{
    if (!dev || !msg || msg->length > 8) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Invalid message or length");
        return CAN_ERROR_INIT;
    }

    CanTxMsg tx_msg = {0};
    tx_msg.StdId = msg->id;
    tx_msg.IDE = CAN_Id_Standard;
    tx_msg.RTR = CAN_RTR_Data;
    tx_msg.DLC = msg->length;
    memcpy(tx_msg.Data, msg->data, msg->length);

    uint8_t mailbox = CAN_Transmit(dev->instance, &tx_msg);
    if (mailbox == CAN_TxStatus_NoMailBox) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] No mailbox available");
        return CAN_ERROR_TRANSMIT;
    }

    uint32_t timeout = 1000000;
    while (CAN_TransmitStatus(dev->instance, mailbox) != CAN_TxStatus_Ok && timeout--) {}
    if (timeout == 0) {
        CAN_CancelTransmit(dev->instance, mailbox);
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Transmit timeout");
        return CAN_ERROR_TRANSMIT;
    }

    Log_Message(LOG_LEVEL_INFO, "[CAN] Message sent successfully");
    return CAN_OK;
}

/**
 * @brief 接收 CAN 消息
 * @param dev CAN 设备实例
 * @param msg CAN 消息存储指针
 * @param timeout 超时时间（毫秒）
 * @return CAN_Status 操作状态
 */
CAN_Status CAN_ReceiveMessage(CAN_Device_t *dev, CAN_Message_t *msg, uint32_t timeout)
{
    if (!dev || !msg) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Invalid device or message");
        return CAN_ERROR_INIT;
    }

    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] RTOS ops not initialized");
        return CAN_ERROR_INIT;
    }

    if (rtos_ops->SemaphoreTake(dev->rx_buffer.sem, timeout)) {
        RB_Status rb_status = RingBuffer_Read(&dev->rx_buffer, msg);
        if (rb_status != RB_OK) {
            rtos_ops->SemaphoreGive(dev->rx_buffer.sem);
            Log_Message(LOG_LEVEL_WARNING, "[CAN] RX buffer error: %d", rb_status);
            return (rb_status == RB_ERROR_BUFFER_EMPTY) ? CAN_ERROR_NO_DATA : CAN_ERROR_BUFFER_FULL;
        }
        rtos_ops->SemaphoreGive(dev->rx_buffer.sem);
        Log_Message(LOG_LEVEL_INFO, "[CAN] Message received");
        return CAN_OK;
    }
    Log_Message(LOG_LEVEL_WARNING, "[CAN] No message received within timeout");
    return CAN_ERROR_NO_DATA;
}

/**
 * @brief CAN 中断处理函数
 * @param dev CAN 设备实例
 */
void CAN_IRQHandler(CAN_Device_t *dev)
{
    if (CAN_GetITStatus(dev->instance, CAN_IT_FMP0) != RESET) {
        CanRxMsg rx_msg = {0};
        CAN_Receive(dev->instance, CAN_FIFO0, &rx_msg);

        CAN_Message_t msg = {0};
        msg.id = rx_msg.StdId;
        msg.length = rx_msg.DLC;
        memcpy(msg.data, rx_msg.Data, rx_msg.DLC);

        void *xHigherPriorityTaskWoken = NULL;
        RingBuffer_WriteFromISR(&dev->rx_buffer, &msg, &xHigherPriorityTaskWoken);

        const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
        if (rtos_ops) {
            rtos_ops->YieldFromISR(xHigherPriorityTaskWoken);
        }

        CAN_ClearITPendingBit(dev->instance, CAN_IT_FMP0);
    }
}

const CAN_Ops_t CAN_Operations = {
    .Init = CANx_Init,
    .Deinit = CAN_Deinit,
    .SendMessage = CAN_SendMessage,
    .ReceiveMessage = CAN_ReceiveMessage
};

/*
 * 示例用法：
 * 1. 设置 RTOS 抽象层
 * RTOS_SetOps(&FreeRTOS_Ops);
 *
 * 2. 初始化 CAN 设备
 * CAN_Device_t can_dev = {
 *     .instance = CAN1,
 *     .tx_port = GPIOA, .tx_pin = GPIO_Pin_12,
 *     .rx_port = GPIOA, .rx_pin = GPIO_Pin_11,
 *     .baudrate = 500000,
 *     .af = GPIO_AF_CAN1,
 *     .irqn = CAN1_RX0_IRQn
 * };
 * CAN_Operations.Init(&can_dev);
 *
 * 3. 发送消息
 * CAN_Message_t msg = { .id = 0x123, .length = 8, .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08} };
 * CAN_Operations.SendMessage(&can_dev, &msg);
 *
 * 4. 接收消息
 * CAN_Message_t rx_msg;
 * CAN_Operations.ReceiveMessage(&can_dev, &rx_msg, 1000);
 */