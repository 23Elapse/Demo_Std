#include "my_rs485.h"
// RS485方向控制引脚初始化
void RS485_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOG, &GPIO_InitStruct);
    RS485_RX_MODE(); // 初始化为接收模式
}
void rs485_init(void) {
    // 初始化发送和接收缓冲区
    tx_buffer.head = 0;
    tx_buffer.tail = 0;
    rx_buffer.head = 0;
    rx_buffer.tail = 0;
    pcf8574_init(); // 初始化PCF8574，用于控制RE脚
    // 初始化RS485 GPIO
    RS485_GPIO_Init();
    USARTx_Init(&USART2_Config, &GPIO2_Config); // 初始化串口
    
    
}
// 中断服务函数
void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART2);
        // 将数据写入接收缓冲区
        if ((rx_buffer.head + 1) % BUFFER_SIZE != rx_buffer.tail) {
            rx_buffer.buffer[rx_buffer.head] = data;
            rx_buffer.head = (rx_buffer.head + 1) % BUFFER_SIZE;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

// 写入发送缓冲区（主循环调用）
void Buffer_Write_Tx(uint8_t data) {
    if ((tx_buffer.head + 1) % BUFFER_SIZE != tx_buffer.tail) {
        tx_buffer.buffer[tx_buffer.head] = data;
        tx_buffer.head = (tx_buffer.head + 1) % BUFFER_SIZE;
    }
}

// 写入接收缓冲区（主循环调用）
void Buffer_Write_Rx(uint8_t data) {
    if ((rx_buffer.head + 1) % BUFFER_SIZE != rx_buffer.tail) {
        rx_buffer.buffer[rx_buffer.head] = data;
        rx_buffer.head = (rx_buffer.head + 1) % BUFFER_SIZE;
    }
}
// 从接收缓冲区读取（主循环调用）
uint8_t Buffer_Read_Rx(void) {
    uint8_t data = 0;
    if (rx_buffer.head != rx_buffer.tail) {
        data = rx_buffer.buffer[rx_buffer.tail];
        rx_buffer.tail = (rx_buffer.tail + 1) % BUFFER_SIZE;
    }
    return data;
}

// 发送数据（主循环调用）
void RS485_Send(void) {
    if (tx_buffer.tail != tx_buffer.head) {
        RS485_TX_MODE(); // 切换为发送模式
        USART_SendData(USART2, tx_buffer.buffer[tx_buffer.tail]);
        tx_buffer.tail = (tx_buffer.tail + 1) % BUFFER_SIZE;
        // 等待发送完成（轮询）
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
        RS485_RX_MODE(); // 恢复接收模式
    }
}

// 写入发送缓冲区
void Buffer_Write(RingBuffer *buf, uint8_t data) {
    buf->buffer[buf->head] = data;
    buf->head = (buf->head + 1) % BUFFER_SIZE;
}

// 从接收缓冲区读取
uint8_t Buffer_Read(RingBuffer *buf) {
    uint8_t data = buf->buffer[buf->tail];
    buf->tail = (buf->tail + 1) % BUFFER_SIZE;
    return data;
}
// 清空发送缓冲区
void Buffer_Clear_Tx(void) {
    tx_buffer.head = 0;
    tx_buffer.tail = 0;
}
// 清空接收缓冲区
void Buffer_Clear_Rx(void) {
    rx_buffer.head = 0;
    rx_buffer.tail = 0;
}


USART_ConfigTypeDef USART2_Config = {
    .USARTx = USART2,
    .BaudRate = 115200,
    .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
    .USART_IRQChannel = USART2_IRQn,
    .USART_CLK = RCC_APB2Periph_USART1,
    .NVIC_EnableIRQ = 1,
    .NVIC_IRQChannelPreemptionPriority = 1,
    .NVIC_IRQChannelSubPriority = 1,
    .USART_IRQChannelCmd = ENABLE
};


GPIO_ConfigTypeDef GPIO2_Config = {
    .GPIOx_TX = GPIOA,
    .GPIOx_RX = GPIOA,
    .GPIO_Pin_TX = GPIO_Pin_2,
    .GPIO_Pin_RX = GPIO_Pin_3,
    .GPIO_CLK = RCC_AHB1Periph_GPIOA,
    .GPIO_AF = GPIO_AF_USART2,
    .GPIO_PinSource_RX = GPIO_PinSource3,
    .GPIO_PinSource_TX = GPIO_PinSource2,
    .GPIO_Mode = GPIO_Mode_AF,
    .GPIO_Speed = GPIO_Speed_50MHz,
    .GPIO_OType = GPIO_OType_PP,
    .GPIO_PuPd = GPIO_PuPd_UP
};

