//#include "stm32f4xx.h"
//#include "stm32f4xx_rcc.h"
//#include "stm32f4xx_gpio.h"
//#include "stm32f4xx_usart.h"
//#include "stm32f4xx_tim.h"
//#include "FreeRTOS.h"
//#include "semphr.h"
//#include "task.h"
//#include "common_driver.h"
#include "serial_driver.h"
#include "pch.h"
/**
 * @brief  初始化错误日志缓冲区
 * @retval Serial_Status 状态码
 */
static Serial_Status Serial_ErrorLog_Init(void)
{
    if (RingBuffer_Init(&error_log_buffer, 16, sizeof(Serial_ErrorLog_t)) != RB_OK) {
        return SERIAL_ERR_INIT;
    }
    return SERIAL_OK;
}

/**
 * @brief  记录错误日志
 * @param  type: 错误类型
 * @param  instance: 设备实例
 */
static void Serial_LogError(Serial_ErrorType_t type, USART_TypeDef* instance)
{
    Serial_ErrorLog_t log = {
        .type = type,
        .timestamp = xTaskGetTickCount(),
        .instance = instance
    };
    RingBuffer_Write(&error_log_buffer, &log);
}

/**
 * @brief  获取错误日志
 * @param  log: 日志存储地址
 * @param  timeout: 等待时间 (ticks)
 * @retval Serial_Status 状态码
 */
Serial_Status Serial_GetErrorLog(Serial_ErrorLog_t *log, TickType_t timeout)
{
    if (!log) return SERIAL_ERR_INIT;

    if (xSemaphoreTake(error_log_buffer.sem, timeout) == pdTRUE) {
        RB_Status rb_status = RingBuffer_Read(&error_log_buffer, log);
        if (rb_status != RB_OK) {
            xSemaphoreGive(error_log_buffer.sem);
            return (rb_status == RB_ERROR_BUFFER_EMPTY) ? SERIAL_ERR_NO_DATA : SERIAL_ERR_BUFFER_FULL;
        }
        return SERIAL_OK;
    }
    return SERIAL_ERR_NO_DATA;
}

/**
 * @brief  计算Modbus RTU的CRC16
 * @param  data: 数据缓冲区
 * @param  length: 数据长度
 * @retval CRC16值
 */
static uint16_t Modbus_CRC16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief  初始化DE引脚定时器（RS485模式）
 * @param  dev: 串口设备实例指针
 * @retval Serial_Status 状态码
 */
static Serial_Status Serial_TIM_Init(Serial_Device_t *dev)
{
    if (dev->mode != RS485_MODE) {
        return SERIAL_OK; // UART模式无需定时器
    }

    if (!dev->timer || !dev->timer_irqn) {
        return SERIAL_ERR_INIT;
    }

    // 初始化定时器（100us）
    if (Common_TIM_Init(dev->timer, 100, dev->timer_irqn) != COMMON_OK) {
        return SERIAL_ERR_INIT;
    }

    return SERIAL_OK;
}

/**
 * @brief  初始化串口的GPIO
 * @param  dev: 串口设备实例指针
 * @retval Serial_Status 状态码
 */
static Serial_Status Serial_GPIO_Init(Serial_Device_t *dev)
{
    if (!dev->tx_port || !dev->rx_port || (dev->mode == RS485_MODE && !dev->de_port)) {
        return SERIAL_ERR_INIT;
    }

    // 配置TX引脚（复用推挽）
    if (Common_GPIO_Init(dev->tx_port, dev->tx_pin, GPIO_Mode_AF, GPIO_OType_PP,
                         GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK) {
        return SERIAL_ERR_INIT;
    }

    // 配置RX引脚（复用上拉）
    if (Common_GPIO_Init(dev->rx_port, dev->rx_pin, GPIO_Mode_AF, GPIO_OType_PP,
                         GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK) {
        return SERIAL_ERR_INIT;
    }

    // 配置DE引脚（仅RS485模式）
    if (dev->mode == RS485_MODE) {
        if (Common_GPIO_Init(dev->de_port, dev->de_pin, GPIO_Mode_OUT, GPIO_OType_PP,
                             GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0) != COMMON_OK) {
            return SERIAL_ERR_INIT;
        }
        // 默认接收模式（DE引脚低电平）
        GPIO_ResetBits(dev->de_port, dev->de_pin);
    }

    return SERIAL_OK;
}

/**
 * @brief  初始化串口的USART
 * @param  dev: 串口设备实例指针
 * @retval Serial_Status 状态码
 */
static Serial_Status Serial_USART_Init(Serial_Device_t *dev)
{
    if (!dev->instance) {
        return SERIAL_ERR_INIT;
    }

    // 初始化USART（8-N-1）
    if (Common_USART_Init(dev->instance, dev->baudrate, USART_WordLength_8b,
                         USART_StopBits_1, USART_Parity_No) != COMMON_OK) {
        return SERIAL_ERR_INIT;
    }

    return SERIAL_OK;
}

/**
 * @brief  初始化串口的NVIC
 * @param  dev: 串口设备实例指针
 * @retval Serial_Status 状态码
 */
static Serial_Status Serial_NVIC_Init(Serial_Device_t *dev)
{
    if (!dev->instance) {
        return SERIAL_ERR_INIT;
    }

    // 启用接收中断
    USART_ITConfig(dev->instance, USART_IT_RXNE, ENABLE);

    // 配置NVIC
    NVIC_InitTypeDef nvic_init = {
        .NVIC_IRQChannel = dev->irqn,
        .NVIC_IRQChannelPreemptionPriority = 0,
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE
    };
    NVIC_Init(&nvic_init);

    return SERIAL_OK;
}

/**
 * @brief  初始化串口的环形缓冲区
 * @param  dev: 串口设备实例指针
 * @retval Serial_Status 状态码
 */
static Serial_Status Serial_RingBuffer_Init(Serial_Device_t *dev)
{
    // 初始化环形缓冲区（16条Serial_RxData_t消息）
    if (RingBuffer_Init(&dev->rx_buffer, 16, sizeof(Serial_RxData_t)) != RB_OK) {
        return SERIAL_ERR_INIT;
    }

    return SERIAL_OK;
}

/**
 * @brief  初始化串口设备
 * @param  dev: 串口设备实例指针
 * @retval Serial_Status 状态码
 * @note   模块化调用，失败时回滚
 */
Serial_Status Serial_Init(Serial_Device_t *dev)
{
    if (!dev || !dev->instance || serial_device_count >= 4) {
        return SERIAL_ERR_INIT;
    }

    // 初始化错误日志（仅首次调用）
    static uint8_t log_initialized = 0;
    if (!log_initialized) {
        if (Serial_ErrorLog_Init() != SERIAL_OK) {
            return SERIAL_ERR_INIT;
        }
        log_initialized = 1;
    }

    Serial_Status status;

    // 初始化GPIO
    status = Serial_GPIO_Init(dev);
    if (status != SERIAL_OK) {
        Serial_Deinit(dev);
        return status;
    }

    // 初始化USART
    status = Serial_USART_Init(dev);
    if (status != SERIAL_OK) {
        Serial_Deinit(dev);
        return status;
    }

    // 初始化NVIC
    status = Serial_NVIC_Init(dev);
    if (status != SERIAL_OK) {
        Serial_Deinit(dev);
        return status;
    }

    // 初始化定时器（仅RS485）
    status = Serial_TIM_Init(dev);
    if (status != SERIAL_OK) {
        Serial_Deinit(dev);
        return status;
    }

    // 初始化环形缓冲区
    status = Serial_RingBuffer_Init(dev);
    if (status != SERIAL_OK) {
        Serial_Deinit(dev);
        return status;
    }

    // 注册设备
    serial_devices[serial_device_count++] = dev;

    return SERIAL_OK;
}

/**
 * @brief  释放串口设备资源
 * @param  dev: 串口设备实例指针
 * @retval Serial_Status 状态码
 */
Serial_Status Serial_Deinit(Serial_Device_t *dev)
{
    if (!dev) return SERIAL_ERR_INIT;

    // 禁用USART和中断
    if (dev->instance) {
        USART_ITConfig(dev->instance, USART_IT_RXNE, DISABLE);
        USART_Cmd(dev->instance, DISABLE);
    }

    // 禁用定时器（仅RS485）
    if (dev->mode == RS485_MODE && dev->timer) {
        TIM_Cmd(dev->timer, DISABLE);
        TIM_ITConfig(dev->timer, TIM_IT_Update, DISABLE);
    }

    // 释放环形缓冲区
    if (RingBuffer_Deinit(&dev->rx_buffer) != RB_OK) {
        return SERIAL_ERR_INIT;
    }

    // 移除设备
    for (uint8_t i = 0; i < serial_device_count; i++) {
        if (serial_devices[i] == dev) {
            serial_devices[i] = serial_devices[--serial_device_count];
            serial_devices[serial_device_count] = NULL;
            break;
        }
    }

    return SERIAL_OK;
}

/**
 * @brief  发送串口数据
 * @param  dev: 串口设备实例指针
 * @param  data: 要发送的数据
 * @param  length: 数据长度
 * @retval Serial_Status 状态码
 * @note   RS485使用定时器控制DE，UART直接发送
 */
Serial_Status Serial_SendData(Serial_Device_t *dev, const uint8_t *data, uint32_t length)
{
    if (!dev || !data) return SERIAL_ERR_INIT;

    // RS485模式：启动定时器，触发DE高电平
    if (dev->mode == RS485_MODE) {
        TIM_SetCounter(dev->timer, 0);
        TIM_Cmd(dev->timer, ENABLE);
        while (!GPIO_ReadOutputDataBit(dev->de_port, dev->de_pin)) {
            // 等待DE高电平（100us）
        }
    }

    // 发送数据
    uint32_t timeout = 1000000;
    for (uint32_t i = 0; i < length; i++) {
        while (USART_GetFlagStatus(dev->instance, USART_FLAG_TXE) == RESET && timeout--) {
            // 等待发送寄存器空
        }
        if (timeout == 0) {
            if (dev->mode == RS485_MODE) {
                TIM_SetCounter(dev->timer, 0);
                TIM_Cmd(dev->timer, ENABLE); // 触发DE低电平
            }
            Serial_LogError(ERROR_TIMEOUT, dev->instance);
            return SERIAL_ERR_TIMEOUT;
        }
        USART_SendData(dev->instance, data[i]);
    }

    // 等待发送完成
    timeout = 1000000;
    while (USART_GetFlagStatus(dev->instance, USART_FLAG_TC) == RESET && timeout--) {
        // 等待传输完成
    }
    if (timeout == 0) {
        if (dev->mode == RS485_MODE) {
            TIM_SetCounter(dev->timer, 0);
            TIM_Cmd(dev->timer, ENABLE); // 触发DE低电平
        }
        Serial_LogError(ERROR_TIMEOUT, dev->instance);
        return SERIAL_ERR_TIMEOUT;
    }

    // RS485模式：触发DE低电平
    if (dev->mode == RS485_MODE) {
        TIM_SetCounter(dev->timer, 0);
        TIM_Cmd(dev->timer, ENABLE);
    }

    return SERIAL_OK;
}

/**
 * @brief  从环形缓冲区接收串口数据
 * @param  dev: 串口设备实例指针
 * @param  data: 接收到的数据
 * @param  timeout: 等待时间 (ticks)
 * @retval Serial_Status 状态码
 */
Serial_Status Serial_ReceiveFromBuffer(Serial_Device_t *dev, Serial_RxData_t *data, TickType_t timeout)
{
    if (!dev || !data) return SERIAL_ERR_INIT;

    if (xSemaphoreTake(dev->rx_buffer.sem, timeout) == pdTRUE) {
        RB_Status rb_status = RingBuffer_Read(&dev->rx_buffer, data);
        if (rb_status != RB_OK) {
            xSemaphoreGive(dev->rx_buffer.sem);
            return (rb_status == RB_ERROR_BUFFER_EMPTY) ? SERIAL_ERR_NO_DATA : SERIAL_ERR_BUFFER_FULL;
        }
        return SERIAL_OK;
    }
    return SERIAL_ERR_NO_DATA;
}



/**
 * @brief  串口操作接口实例
 */
const Serial_Ops_t Serial_Operations = {
    .Init = Serial_Init,
    .Deinit = Serial_Deinit,
    .SendData = Serial_SendData,
    .ReceiveFromBuffer = Serial_ReceiveFromBuffer,
    .GetErrorLog = Serial_GetErrorLog
};
// 示例RS485设备（USART1，TIM2）
Serial_Device_t RS485_Device = {
    .instance = USART1,
    .tx_port = GPIOA,
    .tx_pin = GPIO_Pin_9,
    .rx_port = GPIOA,
    .rx_pin = GPIO_Pin_10,
    .de_port = GPIOA,
    .de_pin = GPIO_Pin_8,
    .baudrate = 115200,
    .af = GPIO_AF_USART1,
    .irqn = USART1_IRQn,
    .slave_addr = 0x01,
    .mode = RS485_MODE,
    .timer = TIM2,
    .timer_irqn = TIM2_IRQn,
    .rx_buffer = {0}
};


// 示例UART设备（USART2）
Serial_Device_t UART_Device = {
    .instance = USART2,
    .tx_port = GPIOA,
    .tx_pin = GPIO_Pin_2,
    .rx_port = GPIOA,
    .rx_pin = GPIO_Pin_3,
    .de_port = NULL,
    .de_pin = 0,
    .baudrate = 9600,
    .af = GPIO_AF_USART2,
    .irqn = USART2_IRQn,
    .slave_addr = 0,
    .mode = UART_MODE,
    .timer = NULL,
    .timer_irqn = 0,
    .rx_buffer = {0}
};
/**
 * @brief  初始化串口设备
 * @param  无
 * @retval 无
 * @note   示例设备需用户定义
 */
void SERIAL_INIT(void)
{
    Serial_Operations.Init(&RS485_Device);
    Serial_Operations.Init(&UART_Device);
}

/**
 * @brief  USART中断服务函数
 */
void USART1_IRQHandler(void)
{
    extern Serial_Device_t RS485_Device;
    Serial_IRQHandler(&RS485_Device);
}

void USART2_IRQHandler(void)
{
    extern Serial_Device_t UART_Device;
    Serial_IRQHandler(&UART_Device);
}

/**
 * @brief  通用串口中断处理
 * @param  dev: 串口设备实例指针
 */
void Serial_IRQHandler(Serial_Device_t *dev)
{
    static Serial_RxData_t rx_data = {0};
    static uint32_t index = 0;
    static uint8_t state = 0; // RS485: 0=等待地址，1=接收数据，2=接收CRC；UART: 0=等待帧头，1=接收长度，2=接收数据，3=接收帧尾
    static uint8_t expected_length = 0; // UART预期数据长度
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (USART_GetITStatus(dev->instance, USART_IT_RXNE) != RESET) {
        uint8_t byte = USART_ReceiveData(dev->instance);
        USART_ClearITPendingBit(dev->instance, USART_IT_RXNE);

        if (dev->mode == RS485_MODE) {
            // RS485模式：Modbus RTU解析
            switch (state) {
                case 0: // 等待从站地址
                    if (byte == dev->slave_addr) {
                        rx_data.data[0] = byte;
                        rx_data.length = 1;
                        index = 1;
                        state = 1;
                    }
                    break;
                case 1: // 接收数据
                    if (index < 30) { // 预留2字节CRC
                        rx_data.data[index++] = byte;
                        rx_data.length = index;
                    } else {
                        rx_data.data[index++] = byte;
                        rx_data.length = index;
                        state = 2;
                    }
                    break;
                case 2: // 接收CRC
                    rx_data.data[index++] = byte;
                    rx_data.length = index;
                    if (index >= 4) { // 最小帧：地址+功能码+CRC
                        // 验证CRC
                        uint16_t crc = Modbus_CRC16(rx_data.data, rx_data.length - 2);
                        uint16_t rx_crc = (rx_data.data[rx_data.length - 2] | (rx_data.data[rx_data.length - 1] << 8));
                        if (crc == rx_crc) {
                            RingBuffer_WriteFromISR(&dev->rx_buffer, &rx_data, &xHigherPriorityTaskWoken);
                        } else {
                            Serial_LogError(ERROR_CRC, dev->instance);
                        }
                        index = 0;
                        state = 0;
                        memset(&rx_data, 0, sizeof(Serial_RxData_t));
                    }
                    break;
            }
        } else {
            // UART模式：[0xAA][长度][数据][0x55]
            switch (state) {
                case 0: // 等待帧头
                    if (byte == 0xAA) {
                        rx_data.data[0] = byte;
                        rx_data.length = 1;
                        index = 1;
                        state = 1;
                    } else {
                        Serial_LogError(ERROR_FRAME, dev->instance);
                    }
                    break;
                case 1: // 接收长度
                    if (byte <= 30) { // 数据部分最大30字节
                        rx_data.data[index++] = byte;
                        rx_data.length = index;
                        expected_length = byte;
                        state = 2;
                    } else {
                        Serial_LogError(ERROR_FRAME, dev->instance);
                        index = 0;
                        state = 0;
                        memset(&rx_data, 0, sizeof(Serial_RxData_t));
                    }
                    break;
                case 2: // 接收数据
                    if (index < expected_length + 2) { // 帧头+长度+数据
                        rx_data.data[index++] = byte;
                        rx_data.length = index;
                    }
                    if (index >= expected_length + 2) {
                        state = 3;
                    }
                    break;
                case 3: // 接收帧尾
                    rx_data.data[index++] = byte;
                    rx_data.length = index;
                    if (byte == 0x55 && index == expected_length + 3) {
                        RingBuffer_WriteFromISR(&dev->rx_buffer, &rx_data, &xHigherPriorityTaskWoken);
                    } else {
                        Serial_LogError(ERROR_FRAME, dev->instance);
                    }
                    index = 0;
                    state = 0;
                    expected_length = 0;
                    memset(&rx_data, 0, sizeof(Serial_RxData_t));
                    break;
            }
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief  定时器中断服务函数（DE引脚控制，仅RS485）
 */
void TIM2_IRQHandler(void)
{
    Serial_TIM_IRQHandler(TIM2);
}

void TIM3_IRQHandler(void)
{
    Serial_TIM_IRQHandler(TIM3);
}

void TIM4_IRQHandler(void)
{
    Serial_TIM_IRQHandler(TIM4);
}

/**
 * @brief  通用定时器中断处理
 * @param  timer: 定时器实例
 */
void Serial_TIM_IRQHandler(TIM_TypeDef *timer)
{
    static uint8_t state[4] = {0}; // 每个定时器的DE状态（0:置高，1:置低）
    uint8_t timer_idx = (timer == TIM2) ? 0 : (timer == TIM3) ? 1 : (timer == TIM4) ? 2 : 3;

    if (TIM_GetITStatus(timer, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(timer, TIM_IT_Update);
        TIM_Cmd(timer, DISABLE);

        // 查找使用该定时器的设备
        for (uint8_t i = 0; i < serial_device_count; i++) {
            Serial_Device_t *dev = serial_devices[i];
            if (dev && dev->mode == RS485_MODE && dev->timer == timer) {
                if (state[timer_idx] == 0) {
                    GPIO_SetBits(dev->de_port, dev->de_pin);
                    state[timer_idx] = 1;
                } else {
                    GPIO_ResetBits(dev->de_port, dev->de_pin);
                    state[timer_idx] = 0;
                }
                break;
            }
        }
    }
}



/*
 * 示例用法：
 * 1. 定义RS485设备（TIM2）
 * Serial_Device_t RS485_Device = {
 *     .instance = USART1,
 *     .tx_port = GPIOA,
 *     .tx_pin = GPIO_Pin_9,
 *     .rx_port = GPIOA,
 *     .rx_pin = GPIO_Pin_10,
 *     .de_port = GPIOA,
 *     .de_pin = GPIO_Pin_8,
 *     .baudrate = 115200,
 *     .af = GPIO_AF_USART1,
 *     .irqn = USART1_IRQn,
 *     .slave_addr = 0x01,
 *     .mode = RS485_MODE,
 *     .timer = TIM2,
 *     .timer_irqn = TIM2_IRQn,
 *     .rx_buffer = {0}
 * };
 *
 * 2. 定义UART设备
 * Serial_Device_t UART_Device = {
 *     .instance = USART2,
 *     .tx_port = GPIOA,
 *     .tx_pin = GPIO_Pin_2,
 *     .rx_port = GPIOA,
 *     .rx_pin = GPIO_Pin_3,
 *     .de_port = NULL,
 *     .de_pin = 0,
 *     .baudrate = 9600,
 *     .af = GPIO_AF_USART2,
 *     .irqn = USART2_IRQn,
 *     .slave_addr = 0,
 *     .mode = UART_MODE,
 *     .timer = NULL,
 *     .timer_irqn = 0,
 *     .rx_buffer = {0}
 * };
 *
 * 3. 初始化
 * Serial_Operations.Init(&RS485_Device);
 * Serial_Operations.Init(&UART_Device);
 *
 * 4. 发送数据
 * uint8_t rs485_data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B}; // Modbus
 * Serial_Operations.SendData(&RS485_Device, rs485_data, 8);
 * uint8_t uart_data[] = {0xAA, 0x05, 0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x55}; // [0xAA][5]["Hello"][0x55]
 * Serial_Operations.SendData(&UART_Device, uart_data, 8);
 *
 * 5. 接收数据（任务中）
 * void Serial_RxTask(void *pvParameters) {
 *     Serial_Device_t *dev = (Serial_Device_t *)pvParameters;
 *     Serial_RxData_t rx_data;
 *     while (1) {
 *         if (Serial_Operations.ReceiveFromBuffer(dev, &rx_data, portMAX_DELAY) == SERIAL_OK) {
 *             // 处理 rx_data.data
 *         }
 *     }
 * }
 * xTaskCreate(Serial_RxTask, "RS485_Rx", 256, &RS485_Device, 1, NULL);
 * xTaskCreate(Serial_RxTask, "UART_Rx", 256, &UART_Device, 1, NULL);
 *
 * 6. 读取错误日志（任务中）
 * void Error_Log_Task(void *pvParameters) {
 *     Serial_ErrorLog_t log;
 *     while (1) {
 *         if (Serial_Operations.GetErrorLog(&log, portMAX_DELAY) == SERIAL_OK) {
 *             // 处理 log（type, timestamp, instance）
 *         }
 *     }
 * }
 * xTaskCreate(Error_Log_Task, "Error_Log", 256, NULL, 1, NULL);
 *
 * 7. 释放资源
 * Serial_Operations.Deinit(&RS485_Device);
 * Serial_Operations.Deinit(&UART_Device);
 */

