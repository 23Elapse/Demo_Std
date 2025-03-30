#include "pch.h"
/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART_UX->SR & 0X40) == 0);               /* 等待上一个字符发送完成 */

    USART_UX->DR = (uint8_t)ch;                       /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}

RingBuffer rx_buffer = {0};  // 接收缓冲区
RingBuffer tx_buffer = {0};  // 发送缓冲区

uint8_t Get_GPIO_PinSource(uint16_t GPIO_Pin) {
    uint8_t PinSource = 0;
    while (GPIO_Pin > 1) {
        GPIO_Pin >>= 1;
        PinSource++;
    }
    return PinSource;
}

void USART_InitWithInterrupt(USART_ConfigTypeDef* USART_Config, GPIO_ConfigTypeDef* GPIO_Config) {
    // 1. 使能 GPIO 时钟
    RCC_AHB1PeriphClockCmd(GPIO_Config->GPIO_CLK, ENABLE);

    // 2. 使能 USART 时钟
    if (USART_Config->USARTx == USART1 || USART_Config->USARTx == USART6)
        RCC_APB2PeriphClockCmd(USART_Config->USART_CLK, ENABLE);
    else
        RCC_APB1PeriphClockCmd(USART_Config->USART_CLK, ENABLE);

    // 3. 配置 GPIO 复用功能
    if(GPIO_Config->GPIO_Mode == GPIO_Mode_AF)
    {
        GPIO_PinAFConfig(GPIO_Config->GPIOx_TX, GPIO_Config->GPIO_PinSource_TX, GPIO_Config->GPIO_AF);
        GPIO_PinAFConfig(GPIO_Config->GPIOx_RX, GPIO_Config->GPIO_PinSource_RX, GPIO_Config->GPIO_AF);
    }

    // 4. 配置 GPIO 模式
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Config->GPIO_Pin_TX | GPIO_Config->GPIO_Pin_RX;
    GPIO_InitStruct.GPIO_Mode = GPIO_Config->GPIO_Mode;          // 复用模式
    GPIO_InitStruct.GPIO_Speed = GPIO_Config->GPIO_Speed;        // 50MHz
    GPIO_InitStruct.GPIO_OType = GPIO_Config->GPIO_OType;        // 推挽输出
    GPIO_InitStruct.GPIO_PuPd = GPIO_Config->GPIO_PuPd;          // 上拉
    GPIO_Init(GPIO_Config->GPIOx_RX, &GPIO_InitStruct);
    GPIO_Init(GPIO_Config->GPIOx_TX, &GPIO_InitStruct);

    // 5. 配置 USART 参数
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = USART_Config->BaudRate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Config->USART_Mode;
    USART_Init(USART_Config->USARTx, &USART_InitStruct);

    // 7. 配置 NVIC
    if(1 == USART_Config->NVIC_EnableIRQ) 
    {
        NVIC_InitTypeDef NVIC_InitStruct;
        NVIC_InitStruct.NVIC_IRQChannel = USART_Config->USART_IRQChannel;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = USART_Config->NVIC_IRQChannelPreemptionPriority;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = USART_Config->NVIC_IRQChannelSubPriority;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStruct);
        USART_ITConfig(USART_Config->USARTx, USART_IT_RXNE, ENABLE);  // 使能接收中断
        USART_ITConfig(USART_Config->USARTx, USART_IT_TXE, DISABLE);  // 默认关闭发送中断
    }

    // 8. 使能 USART
    USART_Cmd(USART_Config->USARTx, ENABLE);
}
// 中断服务函数
void USART1_IRQHandler(void) {
    // 接收中断处理
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART1);
        // 将数据存入接收缓冲区
        uint16_t next = (rx_buffer.head + 1) % RX_BUFFER_SIZE;
        if (next != rx_buffer.tail) {
            rx_buffer.buffer[rx_buffer.head] = data;
            rx_buffer.head = next;
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }

    // 发送中断处理
    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
        if (tx_buffer.head != tx_buffer.tail) {
            USART_SendData(USART1, tx_buffer.buffer[tx_buffer.tail]);
            tx_buffer.tail = (tx_buffer.tail + 1) % TX_BUFFER_SIZE;
        } else {
            // 缓冲区空，关闭发送中断
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
        USART_ClearITPendingBit(USART1, USART_IT_TXE);
    }
}

// 发送字符串（中断方式）
void USART1_SendString(const char *str) {
    // 禁用中断防止数据竞争
    __disable_irq();
    uint16_t len = strlen(str);
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (tx_buffer.head + 1) % TX_BUFFER_SIZE;
        while (next == tx_buffer.tail); // 等待缓冲区空间
        tx_buffer.buffer[tx_buffer.head] = str[i];
        tx_buffer.head = next;
    }
    // 使能发送中断
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    __enable_irq();
}

// 接收数据（非阻塞）
uint16_t USART1_ReceiveData(uint8_t *buf, uint16_t len) {
    __disable_irq();
    uint16_t count = 0;
    while (rx_buffer.tail != rx_buffer.head && count < len) {
        buf[count++] = rx_buffer.buffer[rx_buffer.tail];
        rx_buffer.tail = (rx_buffer.tail + 1) % RX_BUFFER_SIZE;
    }
    __enable_irq();
    return count;
}

USART_ConfigTypeDef USART1_Config = {
    .USARTx = USART1,
    .BaudRate = 115200,
    .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
    .USART_IRQChannel = USART1_IRQn,
    .USART_CLK = RCC_APB2Periph_USART1,
    .NVIC_EnableIRQ = 0,
};


GPIO_ConfigTypeDef GPIO1_Config = {
    .GPIOx_TX = GPIOA,
    .GPIOx_RX = GPIOA,
    .GPIO_Pin_TX = GPIO_Pin_9,
    .GPIO_Pin_RX = GPIO_Pin_10,
    .GPIO_CLK = RCC_AHB1Periph_GPIOA,
    .GPIO_AF = GPIO_AF_USART1,
    .GPIO_PinSource_RX = GPIO_PinSource10,
    .GPIO_PinSource_TX = GPIO_PinSource9,
    .GPIO_Mode = GPIO_Mode_AF,
    .GPIO_Speed = GPIO_Speed_50MHz,
    .GPIO_OType = GPIO_OType_PP,
    .GPIO_PuPd = GPIO_PuPd_UP
};

