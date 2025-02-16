#include "pch.h"
/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART_UX->SR & 0X40) == 0);               /* 等待上一个字符发送完成 */

    USART_UX->DR = (uint8_t)ch;                       /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}

#define RX_BUFFER_SIZE 2000  // 接收缓冲区大小
#define TX_BUFFER_SIZE 2000  // 发送缓冲区大小

// 接收缓冲区
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_buffer_index = 0; // 当前接收数据的索引
volatile uint8_t rx_complete_flag = 0; // 接收完成标志

// 发送缓冲区
uint8_t tx_buffer[TX_BUFFER_SIZE];
volatile uint16_t tx_buffer_index = 0; // 当前发送数据的索引
volatile uint8_t tx_complete_flag = 1; // 发送完成标志（初始为1，表示空闲）

typedef struct {
    USART_TypeDef* USARTx;       // USART 实例，例如 USART1, USART2 等
    GPIO_TypeDef* GPIOx;         // GPIO 端口，例如 GPIOA, GPIOB 等
    uint16_t TxPin;              // 发送引脚，例如 GPIO_Pin_9
    uint16_t RxPin;              // 接收引脚，例如 GPIO_Pin_10
    uint32_t RCC_APB2Periph;     // 时钟使能，例如 RCC_APB2Periph_USART1
    uint32_t RCC_APB2Periph_GPIO;// GPIO 时钟使能，例如 RCC_APB2Periph_GPIOA
    uint8_t IRQChannel;          // 中断通道，例如 USART1_IRQn
    uint8_t Priority;           // 中断优先级
    uint32_t BaudRate;           // 波特率，例如 9600
} USART_Config;

void MyUSART_Init(USART_Config* config) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能 GPIO 和 USART 时钟
    RCC_APB2PeriphClockCmd(config->RCC_APB2Periph_GPIO | config->RCC_APB2Periph, ENABLE);

    // 配置 Tx 引脚为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = config->TxPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(config->GPIOx, &GPIO_InitStructure);

    // 配置 Rx 引脚为浮空输入
    GPIO_InitStructure.GPIO_Pin = config->RxPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(config->GPIOx, &GPIO_InitStructure);

    // 配置 USART
    USART_InitStructure.USART_BaudRate = config->BaudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = config->Priority;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(config->USARTx, &USART_InitStructure);

    // 使能接收中断
    USART_ITConfig(config->USARTx, USART_IT_RXNE, ENABLE);

    // 配置 USART 中断
    NVIC_InitStructure.NVIC_IRQChannel = config->IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 使能 USART
    USART_Cmd(config->USARTx, ENABLE);
}
void USART_SendChar(USART_TypeDef* USARTx, uint8_t ch) {
    // 等待发送数据寄存器为空
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
    // 发送数据
    USART_SendData(USARTx, ch);
}
void myUSART_SendData(USART_TypeDef* USARTx, uint8_t* data, uint16_t len) {
    // 等待发送完成
    while (!tx_complete_flag);
    tx_complete_flag = 0; // 标记发送中

    // 将数据复制到发送缓冲区
    memcpy(tx_buffer, data, len);
    tx_buffer_index = 0;

    // 启动发送
    USART_SendChar(USARTx, tx_buffer[tx_buffer_index++]);
}




void USART_IRQHandler(USART_TypeDef* USARTx) {
    if (USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET) {
        // 读取接收到的数据
        uint8_t receivedData = USART_ReceiveData(USARTx);

        // 将数据存入接收缓冲区
        if (rx_buffer_index < RX_BUFFER_SIZE - 1) {
            rx_buffer[rx_buffer_index++] = receivedData;
        }

        // 如果接收到换行符（或其他结束符），标记接收完成
        if (receivedData == '\n' || receivedData == '\r') {
            rx_buffer[rx_buffer_index] = '\0'; // 添加字符串结束符
            rx_complete_flag = 1; // 标记接收完成
            rx_buffer_index = 0;  // 重置索引
        }

        // 清除中断标志
        USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
    }

    if (USART_GetITStatus(USARTx, USART_IT_TXE) != RESET) {
        // 发送下一个字节
        if (tx_buffer_index < TX_BUFFER_SIZE && tx_buffer[tx_buffer_index] != '\0') {
            USART_SendData(USARTx, tx_buffer[tx_buffer_index++]);
        } else {
            // 发送完成
            tx_complete_flag = 1; // 标记发送完成
            USART_ITConfig(USARTx, USART_IT_TXE, DISABLE); // 关闭发送中断
        }
    }
}
// 配置 USART1
USART_Config usart1_config = {
    .USARTx = USART1,
    .GPIOx = GPIOA,
    .TxPin = GPIO_Pin_9,
    .RxPin = GPIO_Pin_10,
    .RCC_APB2Periph = RCC_APB2Periph_USART1,
    .RCC_APB2Periph_GPIO = RCC_AHB1Periph_GPIOA,
    .IRQChannel = USART1_IRQn,
    .Priority = 0,
    .BaudRate = 115200,
};

// 初始化 USART1
void USART1_Init(void) {
    MyUSART_Init(&usart1_config);
}

// USART1 中断处理函数
void USART1_IRQHandler(void) {
    USART_IRQHandler(USART1);
}

