#include "pch.h"
#include "usart.h"
/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);               /* 等待上一个字符发送完成 */

    USART1->DR = (uint8_t)ch;                       /* 将要发送的字符 ch 写入到DR寄存器 */
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

void USARTx_Init(USART_ConfigTypeDef* USART_Config, GPIO_ConfigTypeDef* GPIO_Config) {


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
    if(ENABLE == USART_Config->NVIC_EnableIRQ) 
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

USART_ConfigTypeDef usart_instances[] ={
    {
        .USARTx = USART1,
        .BaudRate = 115200,
        .USART_CLK = RCC_APB2Periph_USART1,
        .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
        .USART_IRQChannel = USART1_IRQn,
        .NVIC_EnableIRQ = ENABLE,
        .NVIC_IRQChannelPreemptionPriority = 0,
        .NVIC_IRQChannelSubPriority = 0,
        .USART_IRQChannelCmd = ENABLE,
        .RxCallback = NULL,
        .TxCallback = NULL,
        .rx_ring = {0},
        .tx_ring = {0}
    },
    {
        .USARTx = USART2,
        .BaudRate = 115200,
        .USART_CLK = RCC_APB1Periph_USART2,
        .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
        .USART_IRQChannel = USART2_IRQn,
        .NVIC_EnableIRQ = ENABLE,
        .NVIC_IRQChannelPreemptionPriority = 0,
        .NVIC_IRQChannelSubPriority = 0,
        .USART_IRQChannelCmd = ENABLE,
        .RxCallback = NULL,
        .TxCallback = NULL,
        .rx_ring = {0},
        .tx_ring = {0}
    },
    {
        .USARTx = USART6,
        .BaudRate = 115200,
        .USART_CLK = RCC_APB2Periph_USART6,
        .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
        .USART_IRQChannel = USART6_IRQn,
        .NVIC_EnableIRQ = ENABLE,
        .NVIC_IRQChannelPreemptionPriority = 0,
        .NVIC_IRQChannelSubPriority = 0,
        .USART_IRQChannelCmd = ENABLE,
        .RxCallback = NULL, // 自定义接收回调函数
        .TxCallback = NULL,
        .rx_ring = {0},
        .tx_ring = {0}
    }
};
#define USART_MAX_NUM           sizeof(usart_instances) / sizeof(USART_ConfigTypeDef)
GPIO_ConfigTypeDef gpio_instances[] = {

    {
        .GPIOx_TX = GPIOA,
        .GPIOx_RX = GPIOA,
        .GPIO_Pin_TX = GPIO_Pin_9,
        .GPIO_Pin_RX = GPIO_Pin_10,
        .GPIO_CLK = RCC_AHB1Periph_GPIOA,
        .GPIO_AF = GPIO_AF_USART1,
        .GPIO_PinSource_TX = GPIO_PinSource9,
        .GPIO_PinSource_RX = GPIO_PinSource10,
        .GPIO_Mode = GPIO_Mode_AF,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd = GPIO_PuPd_UP
    }, // USART1
    {
        .GPIOx_TX = GPIOA,
        .GPIOx_RX = GPIOA,
        .GPIO_Pin_TX = GPIO_Pin_2,
        .GPIO_Pin_RX = GPIO_Pin_3,
        .GPIO_CLK = RCC_AHB1Periph_GPIOA,
        .GPIO_AF = GPIO_AF_USART2,
        .GPIO_PinSource_TX = GPIO_PinSource2,
        .GPIO_PinSource_RX = GPIO_PinSource3,
        .GPIO_Mode = GPIO_Mode_AF,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd = GPIO_PuPd_UP
    }, // USART2
    {
        .GPIOx_TX = GPIOC,
        .GPIOx_RX = GPIOC,
        .GPIO_Pin_TX = GPIO_Pin_6,
        .GPIO_Pin_RX = GPIO_Pin_7,
        .GPIO_CLK = RCC_AHB1Periph_GPIOC,
        .GPIO_AF = GPIO_AF_USART6,
        .GPIO_PinSource_TX = GPIO_PinSource6,
        .GPIO_PinSource_RX = GPIO_PinSource7,
        .GPIO_Mode = GPIO_Mode_AF,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd = GPIO_PuPd_UP
    } // USART6
};
#define GPIO_MAX_NUM            sizeof(gpio_instances) / sizeof(GPIO_ConfigTypeDef)
    
void My_USART_Init()
{
    for (int i = 0; i < USART_MAX_NUM; i++) {
        USARTx_Init(&usart_instances[i], &gpio_instances[i]);
    }
}


//void RingBuffer_Write(RingBuffer* rb, uint8_t* data, uint16_t len) {
//    for (uint16_t i = 0; i < len; i++) {
//        if ((rb->head + 1) % RX_BUFFER_SIZE != rb->tail) {
//            rb->buffer[rb->head] = data[i];
//            rb->head = (rb->head + 1) % RX_BUFFER_SIZE;
//        } else {
//            // 溢出处理：丢弃最旧数据
//            rb->tail = (rb->tail + 1) % RX_BUFFER_SIZE;
//        }
//    }
//}

//uint16_t RingBuffer_Read(RingBuffer* rb, uint8_t* output, uint16_t len) {
//    uint16_t read_len = 0;
//    while (read_len < len && rb->tail != rb->head) {
//        output[read_len++] = rb->buffer[rb->tail];
//        rb->tail = (rb->tail + 1) % RX_BUFFER_SIZE;
//    }
//    return read_len;
//}

//void USART_Generic_IRQHandler(USART_ConfigTypeDef* instance) {
//    if (USART_GetITStatus(instance->USARTx, USART_IT_RXNE) != RESET) {
//        uint8_t data = USART_ReceiveData(instance->USARTx);
//        // 写入接收缓冲区（带溢出保护）
//        if ((instance->rx_ring.head + 1) % RX_BUFFER_SIZE != instance->rx_ring.tail) {
//            instance->rx_ring.buffer[instance->rx_ring.head] = data;
//            instance->rx_ring.head = (instance->rx_ring.head + 1) % RX_BUFFER_SIZE;
//        }
//        // 触发回调（需外部实现）
//        if (instance->RxCallback) {
//            instance->RxCallback(instance->rx_ring.buffer + instance->rx_ring.tail, 1);  // 单字节接收
//        }
//    }
//    
//    // 发送中断处理
//    if (USART_GetITStatus(instance->USARTx, USART_IT_TXE) != RESET) {
//        uint8_t data;
//        if (RingBuffer_Read(&instance->tx_ring, &data, 1) > 0) {
//            USART_SendData(instance->USARTx, data);
//        } else {
//            USART_ITConfig(instance->USARTx, USART_IT_TXE, DISABLE);
//        }
//    }
//}

//void USART1_IRQHandler(void) { USART_Generic_IRQHandler(&usart_instances[0]); }
//void USART6_IRQHandler(void) { USART_Generic_IRQHandler(&usart_instances[2]); }
// 接收回调函数
// void My_RxHandler(uint8_t* data, uint16_t len) {
//     static uint8_t process_buf[RX_BUFFER_SIZE];
//     static uint16_t process_idx = 0;
    
//     // 将新数据追加到处理缓冲区
//     if (process_idx + len < sizeof(process_buf)) {
//         memcpy(process_buf + process_idx, data, len);
//         process_idx += len;
        
//         // 示例：每接收10字节触发一次处理
//         if (process_idx >= 10) {
// //            ProcessReceivedData(process_buf, process_idx);
//             process_idx = 0;  // 重置索引
//         }
//     }
// }

//发送函数
//void SendData(USART_ConfigTypeDef* instance, uint8_t* data, uint16_t len) {
//    RingBuffer_Write(&instance->tx_ring, data, len);
//    USART_ITConfig(instance->USARTx, USART_IT_TXE, ENABLE);  // 启动发送中断
//}

// // 发送函数（应用层调用）
// void SendData(USART_ConfigTypeDef* instance, uint8_t* data, uint16_t len) {
//     if (instance->tx_ring.head + len < TX_BUFFER_SIZE) {
//         memcpy(instance->tx_ring.buffer + instance->tx_ring.head, data, len);
//         instance->tx_ring.head += len;
//         USART_ITConfig(instance->USARTx, USART_IT_TXE, ENABLE);  // 启动发送中断
//     }
// }
