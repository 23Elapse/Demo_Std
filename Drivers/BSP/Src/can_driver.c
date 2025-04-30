#include "can_driver.h"
#include "rtos_abstraction.h"
#include "pch.h"

static CAN_Device_t *can_devices[2] = {0};
static uint8_t can_device_count = 0;

CAN_Status CANx_Init(CAN_Device_t *dev)
{
    if (!dev || !dev->instance || can_device_count >= 2) return CAN_ERROR_INIT;

    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops) return CAN_ERROR_INIT;

    if (Common_GPIO_Init(dev->tx_port, dev->tx_pin, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK) {
        return CAN_ERROR_INIT;
    }

    if (Common_GPIO_Init(dev->rx_port, dev->rx_pin, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK) {
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
        return CAN_ERROR_INIT;
    }

    can_devices[can_device_count++] = dev;
    return CAN_OK;
}

CAN_Status CAN_Deinit(CAN_Device_t *dev)
{
    if (!dev) return CAN_ERROR_INIT;

    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops) return CAN_ERROR_INIT;

    CAN_ITConfig(dev->instance, CAN_IT_FMP0, DISABLE);
    CAN_DeInit(dev->instance);

    if (RingBuffer_Deinit(&dev->rx_buffer) != RB_OK) {
        return CAN_ERROR_INIT;
    }

    for (uint8_t i = 0; i < can_device_count; i++) {
        if (can_devices[i] == dev) {
            can_devices[i] = can_devices[--can_device_count];
            can_devices[can_device_count] = NULL;
            break;
        }
    }

    return CAN_OK;
}

CAN_Status CAN_SendMessage(CAN_Device_t *dev, CAN_Message_t *msg)
{
    if (!dev || !msg || msg->length > 8) return CAN_ERROR_INIT;

    CanTxMsg tx_msg = {0};
    tx_msg.StdId = msg->id;
    tx_msg.IDE = CAN_Id_Standard;
    tx_msg.RTR = CAN_RTR_Data;
    tx_msg.DLC = msg->length;
    memcpy(tx_msg.Data, msg->data, msg->length);

    uint8_t mailbox = CAN_Transmit(dev->instance, &tx_msg);
    if (mailbox == CAN_TxStatus_NoMailBox) {
        return CAN_ERROR_TRANSMIT;
    }

    uint32_t timeout = 1000000;
    while (CAN_TransmitStatus(dev->instance, mailbox) != CAN_TxStatus_Ok && timeout--) {}
    if (timeout == 0) {
        CAN_CancelTransmit(dev->instance, mailbox);
        return CAN_ERROR_TRANSMIT;
    }

    return CAN_OK;
}

CAN_Status CAN_ReceiveMessage(CAN_Device_t *dev, CAN_Message_t *msg, uint32_t timeout)
{
    if (!dev || !msg) return CAN_ERROR_INIT;

    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops) return CAN_ERROR_INIT;

    if (rtos_ops->SemaphoreTake(dev->rx_buffer.sem, timeout)) {
        RB_Status rb_status = RingBuffer_Read(&dev->rx_buffer, msg);
        if (rb_status != RB_OK) {
            rtos_ops->SemaphoreGive(dev->rx_buffer.sem);
            return (rb_status == RB_ERROR_BUFFER_EMPTY) ? CAN_ERROR_NO_DATA : CAN_ERROR_BUFFER_FULL;
        }
        return CAN_OK;
    }
    return CAN_ERROR_NO_DATA;
}

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
        RingBuffer_WriteFromISR(&dev->rx_buffer, &msg, xHigherPriorityTaskWoken);

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

/**
 * @brief  从环形缓冲区接收CAN消息（任务模式）
 * @param  dev: CAN设备实例指针
 * @param  msg: 接收到的消息
 * @param  timeout: 等待时间 (ticks)
 * @retval CAN_Status 状态码
 */
CAN_Status CAN_ReceiveMessageFromBuffer(CAN_Device_t *dev, CanRxMsg *msg, TickType_t timeout)
{
    if (!dev || !msg) return CAN_ERROR_INIT;

    if (xSemaphoreTake(dev->rx_buffer.sem, timeout) == pdTRUE) {
        RB_Status rb_status = RingBuffer_Read(&dev->rx_buffer, msg);
        if (rb_status != RB_OK) {
            xSemaphoreGive(dev->rx_buffer.sem);
            return (rb_status == RB_ERROR_BUFFER_EMPTY) ? CAN_ERROR_NO_DATA : CAN_ERROR_BUFFER_FULL;
        }
        xSemaphoreGive(dev->rx_buffer.sem);
        return CAN_OK;
    }
    xSemaphoreGive(dev->rx_buffer.sem);
    return CAN_ERROR_NO_DATA;
}

CAN_Device_t CAN1_Device = {
    .instance = CAN1,
    .tx_port = GPIOB,
    .tx_pin = GPIO_Pin_9,
    .rx_port = GPIOB,
    .rx_pin = GPIO_Pin_8,
    .baudrate = 1000000, // 1Mbps
    .rx_buffer = {0}, // 在 CAN_Init 中初始化
    // .overflow_callback = Example_OverflowCallback
};
CAN_Device_t CAN2_Device = {        //TODO can2 需重新配置
    .instance = CAN2,
    .tx_port = GPIOB,
    .tx_pin = GPIO_Pin_9,
    .rx_port = GPIOB,
    .rx_pin = GPIO_Pin_8,
    .baudrate = 1000000, // 1Mbps
    .rx_buffer = {0}, // 在 CAN_Init 中初始化
    // .overflow_callback = Example_OverflowCallback
};
/**
 * @brief  初始化CAN设备
 * @param  无
 * @retval 无
 * @note   示例设备需用户定义
 */
void CAN_INIT(void)
{
    // // 示例溢出回调函数
    // static void Example_OverflowCallback(void)
    // {
    //     // 用户实现：处理FIFO溢出，例如记录日志或重置
    // }

    // 示例设备定义

    CAN_Operations.Init(&CAN1_Device);
}

/**
 * @brief  CAN1 FIFO0中断服务函数
 */
void CAN1_RX0_IRQHandler(void)
{
    extern CAN_Device_t CAN1_Device;
    CAN_IRQHandler(&CAN1_Device);
}

/**
 * @brief  CAN2 FIFO0中断服务函数
 */
void CAN2_RX0_IRQHandler(void)
{
    extern CAN_Device_t CAN2_Device;
    CAN_IRQHandler(&CAN2_Device);
}
/*
 * 示例用法：
 * 1. 定义设备
 * void MyOverflowCallback(void) {
 *     // 处理FIFO溢出
 * }
 * CAN_Device_t CAN1_Device = {
 *     .instance = CAN1,
 *     .tx_port = GPIOB,
 *     .tx_pin = GPIO_Pin_9,
 *     .rx_port = GPIOB,
 *     .rx_pin = GPIO_Pin_8,
 *     .baudrate = 1000000,
 *     .rx_buffer = {0},
 *     .overflow_callback = MyOverflowCallback
 * };
 *
 * 2. 初始化
 * CAN_Operations.Init(&CAN1_Device);
 *
 * 3. 发送消息
 * CanTxMsg tx_msg = {
 *     .StdId = 0x123,
 *     .IDE = CAN_Id_Standard,
 *     .RTR = CAN_RTR_Data,
 *     .DLC = 8,
 *     .Data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
 * };
 * CAN_Operations.SendMessage(&CAN1_Device, &tx_msg);
 *
 * 4. 接收消息（任务中）
 * void CAN_RxTask(void *pvParameters) {
 *     CAN_Device_t *dev = (CAN_Device_t *)pvParameters;
 *     CanRxMsg rx_msg;
 *     while (1) {
 *         if (CAN_Operations.ReceiveMessageFromBuffer(dev, &rx_msg, portMAX_DELAY) == CAN_OK) {
 *             // 处理消息
 *         }
 *     }
 * }
 * xTaskCreate(CAN_RxTask, "CAN_Rx", 256, &CAN1_Device, 1, NULL);
 *
 * 5. 释放资源
 * CAN_Operations.Deinit(&CAN1_Device);
 */

