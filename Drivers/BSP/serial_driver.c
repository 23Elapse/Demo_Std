#include "serial_driver.h"
#include "serial_interface.h"
#include "pch.h" // 假设包含 Common_GPIO_Init, Common_USART_Init
#include "rtos_abstraction.h" // For g_rtos_ops

// 定义ESP32串口（USART6）的特定缓冲区大小
#define ESP32_SERIAL_RX_BUFFER_SIZE 512
#define DEFAULT_SERIAL_RX_BUFFER_SIZE 64 // 其他串口的默认大小

static Serial_Device_t *serial_devices[4] = {0}; // 假设最多管理4个串口设备
static uint8_t serial_device_count = 0;

static Serial_Status Serial_GPIO_Init(Serial_Device_t *dev)
{
    if (!dev || !dev->tx_port || !dev->rx_port || (dev->mode == RS485_MODE && !dev->de_port))
    {
        return SERIAL_ERR_INIT;
    }

    if (Common_GPIO_Init(dev->tx_port, dev->tx_pin, GPIO_Mode_AF, GPIO_OType_PP,
                         GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK)
    {
        return SERIAL_ERR_INIT;
    }

    if (Common_GPIO_Init(dev->rx_port, dev->rx_pin, GPIO_Mode_AF, GPIO_OType_PP,
                         GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK)
    {
        return SERIAL_ERR_INIT;
    }

    if (dev->mode == RS485_MODE)
    {
        if (Common_GPIO_Init(dev->de_port, dev->de_pin, GPIO_Mode_OUT, GPIO_OType_PP,
                             GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0) != COMMON_OK)
        {
            return SERIAL_ERR_INIT;
        }
        GPIO_ResetBits(dev->de_port, dev->de_pin); // 默认DE为低电平（接收模式）
    }
    return SERIAL_OK;
}

static Serial_Status Serial_USART_Init(Serial_Device_t *dev)
{
    if (!dev || !dev->instance)
    {
        return SERIAL_ERR_INIT;
    }
    // Common_USART_Init 应该会使能 USART 时钟和 USART 本身
    if (Common_USART_Init(dev->instance, dev->baudrate, USART_WordLength_8b,
                          USART_StopBits_1, USART_Parity_No) != COMMON_OK)
    {
        return SERIAL_ERR_INIT;
    }
    USART_Cmd(dev->instance, ENABLE); // 确保串口已使能
    return SERIAL_OK;
}

static Serial_Status Serial_NVIC_Init(Serial_Device_t *dev)
{
    if (!dev || !dev->instance)
    {
        return SERIAL_ERR_INIT;
    }

    USART_ITConfig(dev->instance, USART_IT_RXNE, ENABLE); // 使能接收非空中断
    NVIC_InitTypeDef nvic_init = {
        .NVIC_IRQChannel = dev->irqn,
        .NVIC_IRQChannelPreemptionPriority = 5, // 优先级根据系统设计调整
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE};
    NVIC_Init(&nvic_init);
    return SERIAL_OK;
}

static Serial_Status Serial_RingBuffer_Init(Serial_Device_t *dev)
{
    if (!dev) return SERIAL_ERR_INIT;

    uint32_t buffer_capacity;
    // 为ESP32的串口(USART6)使用更大的缓冲区
    if (dev->instance == USART6) { // 假设ESP32_Serial使用USART6
        buffer_capacity = ESP32_SERIAL_RX_BUFFER_SIZE;
        Log_Message(LOG_LEVEL_DEBUG, "[SerialDriver] USART6 (ESP32) Rx buffer size: %u", buffer_capacity);
    } else {
        buffer_capacity = DEFAULT_SERIAL_RX_BUFFER_SIZE;
        Log_Message(LOG_LEVEL_DEBUG, "[SerialDriver] Other serial Rx buffer size: %u", buffer_capacity);
    }

    if (RingBuffer_Init(&dev->rx_buffer, buffer_capacity, sizeof(uint8_t)) != RB_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[SerialDriver] Failed to init RingBuffer for USART %p", dev->instance);
        return SERIAL_ERR_INIT;
    }
    return SERIAL_OK;
}

Serial_Status Serial_Driver_Init(Serial_Device_t *dev)
{
    if (!dev || !dev->instance) // 简化检查，其他在子函数中检查
    {
        Log_Message(LOG_LEVEL_ERROR, "[SerialDriver] Init: Invalid device or USART instance");
        return SERIAL_ERR_INIT;
    }
    // 确保 g_rtos_ops 已初始化
    if (!g_rtos_ops || !g_rtos_ops->Malloc || !g_rtos_ops->Free || !g_rtos_ops->SemaphoreCreate || !g_rtos_ops->SemaphoreDelete) {
        Log_Message(LOG_LEVEL_ERROR, "[SerialDriver] Init: RTOS ops not fully available for RingBuffer_Init.");
        return SERIAL_ERR_INIT;
    }


    Serial_Status status;

    status = Serial_GPIO_Init(dev);
    if (status != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[SerialDriver] GPIO Init failed for USART %p", dev->instance);
        // Serial_Driver_Deinit(dev); // Deinit 可能会因部分未初始化而出错，谨慎调用
        return status;
    }

    status = Serial_USART_Init(dev);
    if (status != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[SerialDriver] USART Init failed for USART %p", dev->instance);
        return status;
    }

    status = Serial_NVIC_Init(dev);
    if (status != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[SerialDriver] NVIC Init failed for USART %p", dev->instance);
        return status;
    }

    status = Serial_RingBuffer_Init(dev); // 此函数已修改以支持不同大小
    if (status != SERIAL_OK)
    {
        // Log_Message 已在 Serial_RingBuffer_Init 内部打印
        return status;
    }

    // 计算RS485模式下的静默时间 (如果适用)
    if (dev->mode == RS485_MODE) {
        uint32_t silent_us = (35 * 1000000) / dev->baudrate; // 3.5字符时间的微秒数
        dev->silent_ticks = pdMS_TO_TICKS(silent_us / 1000); // 转换为RTOS ticks
        if (dev->silent_ticks == 0) dev->silent_ticks = 1; // 至少1 tick
    }


    // 将设备添加到内部数组以便中断处理函数查找 (如果需要)
    // 此部分逻辑取决于您的中断处理方式，如果中断函数直接由USARTx_IRQHandler调用并传递dev，则不需要
    // if (serial_device_count < sizeof(serial_devices)/sizeof(serial_devices[0])) {
    //     serial_devices[serial_device_count++] = dev;
    // } else {
    //     Log_Message(LOG_LEVEL_ERROR, "[SerialDriver] Max serial devices reached, cannot track USART %p", dev->instance);
    // }

    Log_Message(LOG_LEVEL_INFO, "[SerialDriver] USART %p initialized successfully.", dev->instance);
    return SERIAL_OK;
}

Serial_Status Serial_Driver_Deinit(Serial_Device_t *dev)
{
    if (!dev) return SERIAL_ERR_INIT;

    if (dev->instance)
    {
        USART_ITConfig(dev->instance, USART_IT_RXNE, DISABLE);
        USART_Cmd(dev->instance, DISABLE);
        // 可能还需要反初始化RCC时钟等，取决于Common_USART_Init的对应操作
    }

    // 从 serial_devices 数组中移除 (如果使用了该数组)
    // for (uint8_t i = 0; i < serial_device_count; i++) {
    //     if (serial_devices[i] == dev) {
    //         for (uint8_t j = i; j < serial_device_count - 1; j++) {
    //             serial_devices[j] = serial_devices[j+1];
    //         }
    //         serial_devices[serial_device_count -1] = NULL;
    //         serial_device_count--;
    //         break;
    //     }
    // }


    if (RingBuffer_Deinit(&dev->rx_buffer) != RB_OK)
    {
        // Log_Message(LOG_LEVEL_ERROR, "[SerialDriver] Failed to deinit RingBuffer for USART %p", dev->instance);
        // 即使环形缓冲区反初始化失败，也应继续尝试其他清理
    }


    Log_Message(LOG_LEVEL_INFO, "[SerialDriver] USART %p de-initialized.", dev->instance);
    return SERIAL_OK;
}

Serial_Status Serial_Driver_SendData(Serial_Device_t *dev, const uint8_t *data, uint32_t length)
{
    if (!dev || !dev->instance || !data || length == 0)
        return SERIAL_ERR_INIT;

    if (dev->mode == RS485_MODE && dev->de_port)
    {
        GPIO_SetBits(dev->de_port, dev->de_pin); // DE高电平，发送模式
        // 可能需要微小延时确保DE稳定，但通常STM32足够快
    }

    uint32_t timeout_cnt; // 用于超时计数
    for (uint32_t i = 0; i < length; i++)
    {
        timeout_cnt = 0xFFFF; // 设置超时计数值 (根据系统时钟和波特率调整)
        while (USART_GetFlagStatus(dev->instance, USART_FLAG_TXE) == RESET)
        {
            if (--timeout_cnt == 0) {
                if (dev->mode == RS485_MODE && dev->de_port) GPIO_ResetBits(dev->de_port, dev->de_pin);
                Log_Message(LOG_LEVEL_INFO, "[SerialDriver] USART %p de-initialized.", dev->instance);
                return SERIAL_ERR_TIMEOUT;
            }
        }
        USART_SendData(dev->instance, data[i]);
    }

    timeout_cnt = 0xFFFFFF; // 等待发送完成的超时应更长
    while (USART_GetFlagStatus(dev->instance, USART_FLAG_TC) == RESET)
    {
         if (--timeout_cnt == 0) {
            if (dev->mode == RS485_MODE && dev->de_port) GPIO_ResetBits(dev->de_port, dev->de_pin);
            Log_Message(LOG_LEVEL_INFO, "[SerialDriver] USART %p de-initialized.", dev->instance);
//            Serial_LogError(ERROR_TIMEOUT, dev->instance);
            return SERIAL_ERR_TIMEOUT;
        }
    }

    if (dev->mode == RS485_MODE && dev->de_port)
    {
        // 在切换到接收模式前，可能需要确保最后一个字节完全移出移位寄存器
        // 对于RS485，通常在TC标志后立即切换DE即可，或根据具体收发器特性稍作延时
        // delay_us(10); // 例如，非常短的延时
        GPIO_ResetBits(dev->de_port, dev->de_pin); // DE低电平，接收模式
    }

    return SERIAL_OK;
}

/**
 * @brief 串口驱动中断处理函数 - 由具体的 USARTx_IRQHandler 调用
 * @param dev 触发中断的串口设备实例
 */
void Serial_Driver_IRQHandler(Serial_Device_t *dev)
{
    if (!dev || !dev->instance || !g_rtos_ops) return;

    void *pxHigherPriorityTaskWoken = NULL; // FreeRTOS specific

    if (USART_GetITStatus(dev->instance, USART_IT_RXNE) != RESET)
    {
        uint8_t byte = USART_ReceiveData(dev->instance); // 读取数据以清除RXNE标志
        // USART_ClearITPendingBit(dev->instance, USART_IT_RXNE); // ReadData应已清除

        if (RingBuffer_WriteFromISR(&dev->rx_buffer, &byte, &pxHigherPriorityTaskWoken) != RB_OK)
        {
            // 缓冲区满，数据丢失。可以考虑设置一个错误标志或计数器。
            // Serial_LogError(ERROR_BUFFER_FULL, dev->instance); // 从ISR记录错误可能复杂
        }
    }
    // 处理其他中断标志，如ORE (Overrun error)
    if (USART_GetFlagStatus(dev->instance, USART_FLAG_ORE) == SET) {
        (void)USART_ReceiveData(dev->instance); // 清除ORE标志需要先读SR再读DR
        USART_ClearFlag(dev->instance, USART_FLAG_ORE); // 有些STM32型号可能需要显式清除
        // Serial_LogError(ERROR_OVERRUN, dev->instance);
    }


    if (g_rtos_ops->YieldFromISR) {
         g_rtos_ops->YieldFromISR(pxHigherPriorityTaskWoken);
    }
}
