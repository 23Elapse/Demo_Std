/**
 * =====================================================================================
 * @file        serial_driver.c
 * @brief       通用串口驱动实现 (Refactored)
 * @author      23Elapse & Gemini
 * @version     2.1 (Refactored)
 * @date        2025-06-14
 * =====================================================================================
 */
#include "serial_driver.h"
#include "common_driver.h" // For Common_GPIO_Init, Common_USART_Init
#include "log_system.h"
#include "pch.h" // For pdFALSE, portYIELD_FROM_ISR (FreeRTOS related)

// 注意：GetPinSource 函数不再需要，因为 Common_GPIO_Init 已经处理了 GPIO 复用功能。

/**
 * @brief 初始化一个串口设备
 * @param dev 指向要初始化的设备结构体
 * @return Serial_Status_t 操作状态
 */
Serial_Status_t Serial_Driver_Init(Serial_Device_t* dev) {
    if (!dev || !dev->instance || !dev->tx_port || !dev->rx_port) {
        Log_Message(LOG_LEVEL_ERROR, "[SERIAL] Init: Invalid device parameters.");
        return SERIAL_ERR_PARAM;
    }

    // 1. 初始化 GPIO
    // TX GPIO 配置为复用推挽输出
    if (Common_GPIO_Init(dev->tx_port, dev->tx_pin, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[SERIAL] Init: Failed to init TX GPIO.");
        return SERIAL_ERR_INIT;
    }
    // RX GPIO 配置为复用输入
    if (Common_GPIO_Init(dev->rx_port, dev->rx_pin, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[SERIAL] Init: Failed to init RX GPIO.");
        return SERIAL_ERR_INIT;
    }

    // 如果是RS485模式，初始化DE引脚
    if (dev->mode == RS485_MODE && dev->de_port) {
        if (Common_GPIO_Init(dev->de_port, dev->de_pin, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0) != COMMON_OK) {
            Log_Message(LOG_LEVEL_ERROR, "[SERIAL] Init: Failed to init RS485 DE GPIO.");
            return SERIAL_ERR_INIT;
        }
        GPIO_ResetBits(dev->de_port, dev->de_pin); // 默认设置为接收模式 (DE低电平)
    }

    // 2. 初始化 USART 外设
    if (Common_USART_Init(dev->instance, dev->baudrate, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No) != COMMON_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[SERIAL] Init: Failed to init USART peripheral.");
        return SERIAL_ERR_INIT;
    }

    // 3. 初始化接收环形缓冲区
    if (RingBuffer_Init(&dev->rx_buffer, SERIAL_RX_BUFFER_DEFAULT_SIZE, sizeof(uint8_t)) != RB_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[SERIAL] Init: Failed to init RX ring buffer.");
        return SERIAL_ERR_INIT;
    }

    // 4. 创建发送互斥锁 (保护发送数据)
    if (dev->tx_mutex == NULL) {
        dev->tx_mutex = g_rtos_ops->SemaphoreCreate();
        if (dev->tx_mutex == NULL) {
            Log_Message(LOG_LEVEL_ERROR, "[SERIAL] Init: Failed to create TX mutex.");
            return SERIAL_ERR_INIT;
        }
    }

    // 5. 配置并使能中断
    USART_ITConfig(dev->instance, USART_IT_RXNE, ENABLE); // 使能接收非空中断
    // 可以根据需要使能其他中断，例如空闲帧中断 (IDLE)
    // USART_ITConfig(dev->instance, USART_IT_IDLE, ENABLE);

    NVIC_InitTypeDef nvic_init = {
        .NVIC_IRQChannel = dev->irqn,
        .NVIC_IRQChannelPreemptionPriority = 5, // 优先级可根据系统调整
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE
    };
    NVIC_Init(&nvic_init);

    USART_Cmd(dev->instance, ENABLE); // 使能 USART 外设

    Log_Message(LOG_LEVEL_INFO, "[SERIAL] Device %p initialized successfully with baudrate %lu.", dev->instance, dev->baudrate);
    return SERIAL_OK;
}

/**
 * @brief 反初始化一个串口设备
 * @param dev 指向要反初始化的设备结构体
 * @return Serial_Status_t 操作状态
 */
Serial_Status_t Serial_Driver_Deinit(Serial_Device_t* dev) {
    if (!dev || !dev->instance) {
        Log_Message(LOG_LEVEL_ERROR, "[SERIAL] Deinit: Invalid device parameter.");
        return SERIAL_ERR_PARAM;
    }

    USART_Cmd(dev->instance, DISABLE); // 禁用 USART 外设
    USART_ITConfig(dev->instance, USART_IT_RXNE, DISABLE); // 禁用中断
    // 禁用其他可能的中断

    if (RingBuffer_Deinit(&dev->rx_buffer) != RB_OK) {
        Log_Message(LOG_LEVEL_WARNING, "[SERIAL] Deinit: Failed to deinit RX ring buffer.");
    }

    if (dev->tx_mutex != NULL) {
        g_rtos_ops->SemaphoreDelete(dev->tx_mutex);
        dev->tx_mutex = NULL;
    }

    // TODO: 可选地，反初始化GPIO和时钟 (取决于 Common_GPIO_Deinit 的实现)

    Log_Message(LOG_LEVEL_INFO, "[SERIAL] Device %p deinitialized successfully.", dev->instance);
    return SERIAL_OK;
}

/**
 * @brief 发送数据 (阻塞方式)
 * @param dev 指向设备结构体
 * @param data 要发送的数据缓冲区
 * @param length 数据长度
 * @return Serial_Status_t 操作状态
 */
Serial_Status_t Serial_Driver_SendData(Serial_Device_t* dev, const uint8_t* data, uint32_t length) {
    if (!dev || !data || length == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[SERIAL] SendData: Invalid parameters.");
        return SERIAL_ERR_PARAM;
    }

    // 获取互斥锁以保护发送过程
    if (!g_rtos_ops->SemaphoreTake(dev->tx_mutex, 100)) { // 100ms 超时
        Log_Message(LOG_LEVEL_WARNING, "[SERIAL] SendData: Failed to get TX mutex (timeout).");
        return SERIAL_ERR_MUTEX_TIMEOUT;
    }

    // 如果是RS485模式，先切换到发送模式 (DE高电平)
    if (dev->mode == RS485_MODE && dev->de_port) {
        GPIO_SetBits(dev->de_port, dev->de_pin);
        // 通常需要一个短延时让DE引脚稳定，或等待一些位时间
        // Common_Delay_us(5); // 示例，具体值需根据硬件和波特率调整
    }
    
    for (uint32_t i = 0; i < length; i++) {
        // 等待发送数据寄存器空中断标志位
        while (USART_GetFlagStatus(dev->instance, USART_FLAG_TXE) == RESET);
        USART_SendData(dev->instance, data[i]);
    }
    // 等待传输完成标志 (确保最后一个字节完全发送出去)
    while (USART_GetFlagStatus(dev->instance, USART_FLAG_TC) == RESET);

    // 如果是RS485模式，切换回接收模式 (DE低电平)
    if (dev->mode == RS485_MODE && dev->de_port) {
        // 在切换回接收前，可能需要一个总线静默时间
        // dev->silent_ticks 可以在这里使用，例如：
        // g_rtos_ops->Delay(dev->silent_ticks);
        GPIO_ResetBits(dev->de_port, dev->de_pin);
    }

    g_rtos_ops->SemaphoreGive(dev->tx_mutex); // 释放互斥锁
    // Log_Message(LOG_LEVEL_DEBUG, "[SERIAL] Sent %lu bytes from %p.", length, dev->instance); // 过于频繁
    return SERIAL_OK;
}

/**
 * @brief 从串口接收数据 (阻塞，带超时)
 * @param dev 指向设备结构体
 * @param data 接收数据缓冲区
 * @param length 期望接收的字节数
 * @param timeout_ms 超时时间 (毫秒)
 * @return Serial_Status_t 操作状态
 */
Serial_Status_t Serial_Driver_ReceiveData(Serial_Device_t* dev, uint8_t* data, uint32_t length, uint32_t timeout_ms) {
    if (!dev || !data || length == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[SERIAL] ReceiveData: Invalid parameters.");
        return SERIAL_ERR_PARAM;
    }

    RB_Status rb_status;
    for (uint32_t i = 0; i < length; i++) {
        // 尝试从环形缓冲区读取一个字节，并等待信号量
        // RingBuffer_Read 内部会尝试获取信号量，所以这里直接调用即可
        rb_status = RingBuffer_Read(&dev->rx_buffer, &data[i]);
        if (rb_status != RB_OK) {
            Log_Message(LOG_LEVEL_DEBUG, "[SERIAL] ReceiveData: Read error (byte %lu/%lu): %d.", i + 1, length, rb_status);
            return (rb_status == RB_ERROR_BUFFER_EMPTY) ? SERIAL_ERR_NO_DATA : SERIAL_ERR_BUFFER_FULL;
        }
    }
    // Log_Message(LOG_LEVEL_DEBUG, "[SERIAL] Received %lu bytes to %p.", length, dev->instance); // 过于频繁
    return SERIAL_OK;
}

/**
 * @brief 从串口环形缓冲区读取一个字节 (非阻塞)
 * @param dev 指向设备结构体
 * @param data 接收字节存储指针
 * @return Serial_Status_t 操作状态 (SERIAL_OK 或 SERIAL_ERR_NO_DATA)
 */
Serial_Status_t Serial_Driver_ReadByte(Serial_Device_t* dev, uint8_t* data) {
    if (!dev || !data) {
        Log_Message(LOG_LEVEL_ERROR, "[SERIAL] ReadByte: Invalid parameters.");
        return SERIAL_ERR_PARAM;
    }

    // 非阻塞读取，不等待信号量
    RB_Status rb_status = RingBuffer_Read(&dev->rx_buffer, data); // timeout_ms = 0 表示非阻塞
    if (rb_status == RB_OK) {
        return SERIAL_OK;
    } else if (rb_status == RB_ERROR_BUFFER_EMPTY) {
        return SERIAL_ERR_NO_DATA;
    } else {
        Log_Message(LOG_LEVEL_WARNING, "[SERIAL] ReadByte: Ring buffer error: %d.", rb_status);
        return SERIAL_ERR_BUFFER_FULL; // 或其他更合适的错误码
    }
}

/**
 * @brief 串口中断服务总处理函数
 * @note  在具体的stm32f4xx_it.c中的USARTx_IRQHandler内调用此函数
 * @param dev 发生中断的设备实例
 */
void Serial_Driver_IRQHandler(Serial_Device_t* dev) {
    if (dev == NULL) {
        // 如果传入NULL，表示中断处理函数没有正确获取到设备实例，
        // 这通常是配置错误，应记录到日志系统。
        Log_Message(LOG_LEVEL_ERROR, "[SERIAL] IRQHandler: NULL device pointer received.");
        return;
    }

    // 接收中断处理
    if (USART_GetITStatus(dev->instance, USART_IT_RXNE) != RESET) {
        uint8_t rx_data = USART_ReceiveData(dev->instance); // 读取接收到的数据
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; // 用于FreeRTOS任务切换

        // 将接收到的数据写入环形缓冲区。
        // RingBuffer_WriteFromISR 内部会处理信号量的给予，并更新 xHigherPriorityTaskWoken
        RB_Status rb_status = RingBuffer_WriteFromISR(&dev->rx_buffer, &rx_data, &xHigherPriorityTaskWoken);

        if (rb_status != RB_OK) {
            // 如果缓冲区满，记录警告。不应影响正常中断流程。
            Log_Message(LOG_LEVEL_INFO, "[SERIAL] IRQHandler: RX buffer full for %p.", dev->instance);
        }

        // 清除接收中断标志位 (部分STM32器件通过读取DR寄存器自动清除，但显式清除更安全)
        // 在STM32F4系列中，读取DR通常会清除RXNE标志。如果还有其他标志，需手动清除。
        // USART_ClearITPendingBit(dev->instance, USART_IT_RXNE); // 根据HAL库，可能不需要手动清除

        // 如果有更高优先级的任务被唤醒，请求上下文切换
        if (g_rtos_ops && xHigherPriorityTaskWoken == pdTRUE) {
            g_rtos_ops->YieldFromISR(xHigherPriorityTaskWoken);
        }
    }

    // 处理其他可能的串口中断，例如：
    // 空闲帧中断 (IDLE): 当总线空闲一段时间后触发，可用于检测一帧数据的结束
    // if (USART_GetITStatus(dev->instance, USART_IT_IDLE) != RESET) {
    //     // 处理空闲帧中断
    //     // 清除IDLE标志位 (读SR，然后读DR)
    //     USART_GetFlagStatus(dev->instance, USART_FLAG_IDLE);
    //     USART_ReceiveData(dev->instance);
    //     // 此时可以判断一帧数据接收完毕，通知上层处理
    // }

    // 错误中断 (ORE, NE, FE, PE):
    // if (USART_GetITStatus(dev->instance, USART_IT_ORE_RX) != RESET) {
    //     // 溢出错误，读取DR清除
    //     USART_ReceiveData(dev->instance);
    //     Log_Message(LOG_LEVEL_ERROR, "[SERIAL] IRQHandler: Overrun error on %p.", dev->instance);
    // }
    // if (USART_GetITStatus(dev->instance, USART_IT_FE) != RESET) { /* 帧错误 */ }
    // if (USART_GetITStatus(dev->instance, USART_IT_NE) != RESET) { /* 噪声错误 */ }
    // if (USART_GetITStatus(dev->instance, USART_IT_PE) != RESET) { /* 奇偶校验错误 */ }
}