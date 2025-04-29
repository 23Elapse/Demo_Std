//#include "stm32f4xx.h"
//#include "stm32f4xx_rcc.h"
//#include "stm32f4xx_gpio.h"
//#include "stm32f4xx_can.h"
//#include "FreeRTOS.h"
//#include "semphr.h"
//#include "ring_buffer.h"
#include "can_driver.h"

#include "pch.h"
/**
 * @brief  获取GPIO引脚源编号
 * @param  pin: GPIO引脚掩码 (如 GPIO_Pin_9)
 * @retval 引脚源编号 (0-15)
 */
static uint8_t GetPinSource(uint16_t pin)
{
    uint8_t source = 0;
    while (pin != 0) {
        pin >>= 1;
        source++;
    }
    return source - 1;
}

/**
 * @brief  初始化CAN设备
 * @param  dev: CAN设备实例指针
 * @retval CAN_Status 状态码
 * @note   需在系统时钟和FreeRTOS初始化后调用，配置GPIO为复用模式，启用FIFO0和溢出中断，配置NVIC
 */
CAN_Status CANx_Init(CAN_Device_t *dev)
{
    if (!dev->instance || !dev->tx_port || !dev->rx_port) {
        return CAN_ERR_INIT;
    }

    // 初始化环形缓冲区（16条CanRxMsg消息）
    if (RingBuffer_Init(&dev->rx_buffer, 16, sizeof(CanRxMsg)) != RB_OK) {
        return CAN_ERR_INIT;
    }

    // 启用GPIO时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);

    // 配置TX引脚
    GPIO_InitTypeDef gpio_init = {
        .GPIO_Pin = dev->tx_pin,
        .GPIO_Mode = GPIO_Mode_AF,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd = GPIO_PuPd_UP,
        .GPIO_Speed = GPIO_Speed_50MHz
    };
    GPIO_Init(dev->tx_port, &gpio_init);

    // 配置RX引脚
    gpio_init.GPIO_Pin = dev->rx_pin;
    GPIO_Init(dev->rx_port, &gpio_init);

    // 配置复用功能
    uint8_t af = (dev->instance == CAN1) ? GPIO_AF_CAN1 : GPIO_AF_CAN2;
    GPIO_PinAFConfig(dev->tx_port, GetPinSource(dev->tx_pin), af);
    GPIO_PinAFConfig(dev->rx_port, GetPinSource(dev->rx_pin), af);

    // 启用CAN时钟
    if (dev->instance == CAN1) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    } else if (dev->instance == CAN2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
    } else {
        return CAN_ERR_INIT;
    }

    // 复位CAN
    CAN_DeInit(dev->instance);

    // 配置CAN参数
    CAN_InitTypeDef can_init = {
        .CAN_TTCM = DISABLE,
        .CAN_ABOM = DISABLE,
        .CAN_AWUM = DISABLE,
        .CAN_NART = DISABLE,
        .CAN_RFLM = DISABLE,
        .CAN_TXFP = DISABLE,
        .CAN_Mode = CAN_Mode_Normal
    };

    // 计算时序参数
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    uint32_t apb1_clk = clocks.PCLK1_Frequency;
    uint32_t prescaler = apb1_clk / (dev->baudrate * 16); // 16个时间量子
    if (prescaler < 1 || prescaler > 1024) {
        return CAN_ERR_INIT;
    }
    can_init.CAN_Prescaler = prescaler;
    can_init.CAN_SJW = CAN_SJW_1tq;
    can_init.CAN_BS1 = CAN_BS1_13tq;
    can_init.CAN_BS2 = CAN_BS2_2tq;

    if (CAN_Init(dev->instance, &can_init) != CAN_InitStatus_Success) {
        return CAN_ERR_INIT;
    }

    // 配置过滤器（接受所有消息）
    uint8_t filter_number = (dev->instance == CAN1) ? 0 : 14;
    CAN_FilterInitTypeDef filter = {
        .CAN_FilterNumber = filter_number,
        .CAN_FilterMode = CAN_FilterMode_IdMask,
        .CAN_FilterScale = CAN_FilterScale_32bit,
        .CAN_FilterIdHigh = 0x0000,
        .CAN_FilterIdLow = 0x0000,
        .CAN_FilterMaskIdHigh = 0x0000,
        .CAN_FilterMaskIdLow = 0x0000,
        .CAN_FilterFIFOAssignment = CAN_FIFO0,
        .CAN_FilterActivation = ENABLE
    };
    CAN_FilterInit(&filter);

    // 启用FIFO0消息挂起中断和FIFO溢出中断
    CAN_ITConfig(dev->instance, CAN_IT_FMP0 | CAN_IT_FF0, ENABLE);

    // 配置NVIC
    NVIC_InitTypeDef nvic_init = {
        .NVIC_IRQChannelPreemptionPriority = 0,
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE
    };
    if (dev->instance == CAN1) {
        nvic_init.NVIC_IRQChannel = CAN1_RX0_IRQn;
    } else {
        nvic_init.NVIC_IRQChannel = CAN2_RX0_IRQn;
    }
    NVIC_Init(&nvic_init);

    return CAN_OK;
}

/**
 * @brief  释放CAN设备资源
 * @param  dev: CAN设备实例指针
 * @retval CAN_Status 状态码
 */
CAN_Status CAN_Deinit(CAN_Device_t *dev)
{
    if (!dev) return CAN_ERR_INIT;

    // 释放环形缓冲区
    if (RingBuffer_Deinit(&dev->rx_buffer) != RB_OK) {
        return CAN_ERR_INIT;
    }

    // 禁用CAN中断
    CAN_ITConfig(dev->instance, CAN_IT_FMP0 | CAN_IT_FF0, DISABLE);

    // 复位CAN
    CAN_DeInit(dev->instance);

    return CAN_OK;
}

/**
 * @brief  发送CAN消息
 * @param  dev: CAN设备实例指针
 * @param  msg: 要发送的消息
 * @retval CAN_Status 状态码
 */
CAN_Status CAN_SendMessage(CAN_Device_t *dev, CanTxMsg *msg)
{
    uint8_t mailbox = CAN_Transmit(dev->instance, msg);
    if (mailbox == CAN_TxStatus_NoMailBox) {
        return CAN_ERR_NO_MAILBOX;
    }

    uint32_t timeout = 1000000;
    while (CAN_TransmitStatus(dev->instance, mailbox) == CAN_TxStatus_Pending && timeout--) {
        // 等待传输完成
    }
    if (timeout == 0) {
        return CAN_ERR_TIMEOUT;
    }
    if (CAN_TransmitStatus(dev->instance, mailbox) != CAN_TxStatus_Ok) {
        return CAN_ERR_TRANSMIT;
    }
    return CAN_OK;
}

/**
 * @brief  接收CAN消息（轮询模式）
 * @param  dev: CAN设备实例指针
 * @param  msg: 接收到的消息
 * @retval CAN_Status 状态码
 * @note   可用于非中断场景，建议优先使用环形缓冲区
 */
CAN_Status CAN_ReceiveMessage(CAN_Device_t *dev, CanRxMsg *msg)
{
    if (CAN_MessagePending(dev->instance, CAN_FIFO0) > 0) {
        CAN_Receive(dev->instance, CAN_FIFO0, msg);
        return CAN_OK;
    }
    return CAN_ERR_NO_MESSAGE;
}

/**
 * @brief  从环形缓冲区接收CAN消息（任务模式）
 * @param  dev: CAN设备实例指针
 * @param  msg: 接收到的消息
 * @param  timeout: 等待时间 (ticks)
 * @retval CAN_Status 状态码
 */
CAN_Status CAN_ReceiveMessageFromBuffer(CAN_Device_t *dev, CanRxMsg *msg, TickType_t timeout)
{
    if (!dev || !msg) return CAN_ERR_INIT;

    if (xSemaphoreTake(dev->rx_buffer.sem, timeout) == pdTRUE) {
        RB_Status rb_status = RingBuffer_Read(&dev->rx_buffer, msg);
        if (rb_status != RB_OK) {
            xSemaphoreGive(dev->rx_buffer.sem);
            return (rb_status == RB_ERROR_BUFFER_EMPTY) ? CAN_ERR_NO_MESSAGE : CAN_ERR_BUFFER_FULL;
        }
        return CAN_OK;
    }
    return CAN_ERR_NO_MESSAGE;
}

/**
 * @brief  CAN操作接口
 */
typedef struct {
    CAN_Status (*Init)(CAN_Device_t*);
    CAN_Status (*Deinit)(CAN_Device_t*);
    CAN_Status (*SendMessage)(CAN_Device_t*, CanTxMsg*);
    CAN_Status (*ReceiveMessage)(CAN_Device_t*, CanRxMsg*);
    CAN_Status (*ReceiveMessageFromBuffer)(CAN_Device_t*, CanRxMsg*, TickType_t);
} CAN_Ops_t;

/**
 * @brief  CAN操作接口实例
 */
const CAN_Ops_t CAN_Operations = {
    .Init = CANx_Init,
    .Deinit = CAN_Deinit,
    .SendMessage = CAN_SendMessage,
    .ReceiveMessage = CAN_ReceiveMessage,
    .ReceiveMessageFromBuffer = CAN_ReceiveMessageFromBuffer
};
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
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    CanRxMsg rx_msg;

    // 处理FIFO0消息挂起中断
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        CAN_Receive(CAN1, CAN_FIFO0, &rx_msg);
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        extern CAN_Device_t CAN1_Device;
        RingBuffer_WriteFromISR(&CAN1_Device.rx_buffer, &rx_msg, &xHigherPriorityTaskWoken);
    }

    // 处理FIFO0溢出中断
    if (CAN_GetITStatus(CAN1, CAN_IT_FF0) != RESET) {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
        // 清空FIFO以恢复接收
        while (CAN_MessagePending(CAN1, CAN_FIFO0) > 0) {
            CAN_Receive(CAN1, CAN_FIFO0, &rx_msg);
        }
        extern CAN_Device_t CAN1_Device;
        if (CAN1_Device.overflow_callback) {
            CAN1_Device.overflow_callback();
        }
    }

    // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

/**
 * @brief  CAN2 FIFO0中断服务函数
 */
void CAN2_RX0_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    CanRxMsg rx_msg;

    // 处理FIFO0消息挂起中断
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET) {
        CAN_Receive(CAN2, CAN_FIFO0, &rx_msg);
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        extern CAN_Device_t CAN2_Device;
        RingBuffer_WriteFromISR(&CAN2_Device.rx_buffer, &rx_msg, &xHigherPriorityTaskWoken);
    }

    // 处理FIFO0溢出中断
    if (CAN_GetITStatus(CAN2, CAN_IT_FF0) != RESET) {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FF0);
        // 清空FIFO以恢复接收
        while (CAN_MessagePending(CAN2, CAN_FIFO0) > 0) {
            CAN_Receive(CAN2, CAN_FIFO0, &rx_msg);
        }
        extern CAN_Device_t CAN2_Device;
        if (CAN2_Device.overflow_callback) {
            CAN2_Device.overflow_callback();
        }
    }

    // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

