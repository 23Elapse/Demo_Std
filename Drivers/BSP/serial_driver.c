/**
 * =====================================================================================
 * @file        serial_driver.c
 * @brief       通用串口驱动实现 (已优化)
 * @author      23Elapse & Gemini
 * @version     2.0 (Refactored)
 * @date        2025-06-08
 * =====================================================================================
 */
#include "serial_driver.h"
#include "common_driver.h"
#include "log_system.h"
#include "pch.h"

#define SERIAL_RX_BUFFER_SIZE 256 // 定义默认的接收缓冲区大小

// 内部函数，根据引脚号获取引脚源
static uint8_t GetPinSource(uint16_t pin) {
    uint8_t source = 0;
    while (pin > 1) {
        pin >>= 1;
        source++;
    }
    return source - 1;
}

Serial_Status Serial_Driver_Init(Serial_Device_t* dev) {
    if (!dev || !dev->instance || !dev->tx_port || !dev->rx_port) {
        return SERIAL_ERR_PARAM;
    }

    // 1. 初始化GPIO
    Common_GPIO_Init(dev->tx_port, dev->tx_pin, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af);
    Common_GPIO_Init(dev->rx_port, dev->rx_pin, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af);
    
    // 如果是RS485模式，初始化DE引脚
    if (dev->mode == RS485_MODE && dev->de_port) {
        Common_GPIO_Init(dev->de_port, dev->de_pin, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0);
        GPIO_ResetBits(dev->de_port, dev->de_pin); // 默认设置为接收模式
    }

    // 2. 初始化USART
    Common_USART_Init(dev->instance, dev->baudrate, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);

    // 3. 初始化接收环形缓冲区
    if (RingBuffer_Init(&dev->rx_buffer, SERIAL_RX_BUFFER_SIZE, sizeof(uint8_t)) != RB_OK) {
        Log_Message(LOG_LEVEL_ERROR, "Error: Failed to init RX ring buffer for USART.");
        return SERIAL_ERR_INIT;
    }

    // 4. 配置并使能中断
    USART_ITConfig(dev->instance, USART_IT_RXNE, ENABLE); // 使能接收中断
    NVIC_InitTypeDef nvic_init = {
        .NVIC_IRQChannel = dev->irqn,
        .NVIC_IRQChannelPreemptionPriority = 5,
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE
    };
    NVIC_Init(&nvic_init);

    USART_Cmd(dev->instance, ENABLE);
    return SERIAL_OK;
}

Serial_Status Serial_Driver_Deinit(Serial_Device_t* dev) {
    if (!dev || !dev->instance) return SERIAL_ERR_PARAM;
    USART_Cmd(dev->instance, DISABLE);
    RingBuffer_Deinit(&dev->rx_buffer);
    // 可选：反初始化GPIO和时钟
    return SERIAL_OK;
}

Serial_Status Serial_Driver_SendData(Serial_Device_t* dev, const uint8_t* data, uint32_t length) {
    if (!dev || !data || length == 0) return SERIAL_ERR_PARAM;

    // 如果是RS485模式，先切换到发送模式
    if (dev->mode == RS485_MODE && dev->de_port) {
        GPIO_SetBits(dev->de_port, dev->de_pin);
    }
    
    for (uint32_t i = 0; i < length; i++) {
        while (USART_GetFlagStatus(dev->instance, USART_FLAG_TXE) == RESET);
        USART_SendData(dev->instance, data[i]);
    }
    // 等待最后一个字节发送完成
    while (USART_GetFlagStatus(dev->instance, USART_FLAG_TC) == RESET);

    // 如果是RS485模式，切换回接收模式
    if (dev->mode == RS485_MODE && dev->de_port) {
        // 此处可以加一个极短的延时确保总线状态稳定，但通常TC标志已足够
        GPIO_ResetBits(dev->de_port, dev->de_pin);
    }
    return SERIAL_OK;
}

void Serial_Driver_IRQHandler(Serial_Device_t* dev) {
    if (dev && USART_GetITStatus(dev->instance, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(dev->instance);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // 将接收到的数据写入环形缓冲区，并唤醒等待的消费者任务
        RingBuffer_WriteFromISR(&dev->rx_buffer, &data, &xHigherPriorityTaskWoken);

        // 如果有更高优先级的任务被唤醒，执行任务切换
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    // 可在此处添加其他中断处理，如发送完成、错误等
}
