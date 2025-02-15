#include "./BSP/Inc/rs485.h"
#include "./BSP/Inc/pcf8574.h"
#include "./SYSTEM/Inc/delay.h"


UART_HandleTypeDef g_rs458_handle;          /* USART2句柄(用于RS485) */

#if EN_USART2_RX                            /* 如果使能了接收 */

uint8_t g_rs485_rx_buf[RS485_REC_LEN];      /* 接收缓冲,最大64个字节. */
uint8_t g_rs485_rx_cnt = 0;                 /* 接收到的数据长度 */

void USART2_IRQHandler(void)
{
    uint8_t res;

    if ((__HAL_UART_GET_FLAG(&g_rs458_handle, UART_FLAG_RXNE) != RESET))    /* 接收中断 */
    {
        HAL_UART_Receive(&g_rs458_handle, &res, 1, 1000);
        if (g_rs485_rx_cnt < RS485_REC_LEN)
        {
            g_rs485_rx_buf[g_rs485_rx_cnt] = res;                           /* 记录接收到的值 */
            g_rs485_rx_cnt++;                                               /* 接收数据增加1  */
        } 
    } 
}
#endif

/**
 * @brief       RS485初始化函数
 * @note        该函数主要是初始化串口
 * @param       bound   : 波特率, 根据自己需要设置波特率值
 * @retval      无
 */
void rs485_init(uint32_t bound)
{
    GPIO_InitTypeDef gpio_init_struct;
    RS485_TX_GPIO_CLK_ENABLE();                                     /* 使能GPIOA时钟 */
    RS485_UX_CLK_ENABLE();                                          /* 使能USART2时钟 */

    pcf8574_init();                                                 /* 初始化PCF8574，用于控制RE脚 */

    gpio_init_struct.Pin = RS485_TX_GPIO_PIN | RS485_RX_GPIO_PIN; 
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                        /* 复用推挽 */
    gpio_init_struct.Pull = GPIO_PULLUP;                            /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_HIGH;                       /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF7_USART2;                   /* 复用为USART2 */
    HAL_GPIO_Init(RS485_TX_GPIO_PORT, &gpio_init_struct);           /* 初始化PA2,3 */

    /* USART 初始化设置 */
    g_rs458_handle.Instance = RS485_UX;                             /* USART2 */
    g_rs458_handle.Init.BaudRate = bound;                           /* 波特率 */
    g_rs458_handle.Init.WordLength = UART_WORDLENGTH_8B;            /* 字长为8位数据格式 */
    g_rs458_handle.Init.StopBits = UART_STOPBITS_1;                 /* 一个停止位 */
    g_rs458_handle.Init.Parity = UART_PARITY_NONE;                  /* 无奇偶校验位 */
    g_rs458_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;            /* 无硬件流控 */
    g_rs458_handle.Init.Mode = UART_MODE_TX_RX;                     /* 收发模式 */
    HAL_UART_Init(&g_rs458_handle);                                 /* HAL_UART_Init()会使能USART2 */

    __HAL_UART_DISABLE_IT(&g_rs458_handle, UART_IT_TC);

#if EN_USART2_RX
    __HAL_UART_ENABLE_IT(&g_rs458_handle, UART_IT_RXNE);            /* 开启接收中断 */
    HAL_NVIC_EnableIRQ(RS485_UX_IRQn);                              /* 使能USART2中断 */
    HAL_NVIC_SetPriority(RS485_UX_IRQn, 3, 3);                      /* 抢占优先级3，子优先级3 */
#endif

    rs485_tx_set(0);                                                /* 设置为接收模式 */
}

/**
 * @brief       RS485发送len个字节
 * @param       buf      : 发送区首地址
 * @param       len      : 发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
 * @retval      无
 */
void rs485_send_data(uint8_t *buf, uint8_t len)
{
    rs485_tx_set(1);                                                /* 设置为发送模式 */
    HAL_UART_Transmit(&g_rs458_handle, buf, len, 1000);             /* 串口2发送数据 */
    g_rs485_rx_cnt = 0;
    rs485_tx_set(0);                                                /* 设置为接收模式 */
}

/**
 * @brief       RS485查询接收到的数据
 * @param       buf  : 接收缓存首地址
 * @param       len  : 读到的数据长度
 * @retval      无
 */
void rs485_receive_data(uint8_t *buf, uint8_t *len)
{
    uint8_t rxlen = g_rs485_rx_cnt;
    uint8_t i = 0;
    *len = 0;                               /* 默认为0 */
    delay_ms(10);                           /* 等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束 */

    if (rxlen == g_rs485_rx_cnt && rxlen)   /* 接收到了数据,且接收完成了 */
    {
        for (i = 0; i < rxlen; i++)
        {
            buf[i] = g_rs485_rx_buf[i];
        }

        *len = g_rs485_rx_cnt;              /* 记录本次数据长度 */
        g_rs485_rx_cnt = 0;                 /* 清零 */
    }
}

/**
 * @brief       RS485模式控制.
 * @param       en   : 0,接收;1,发送
 * @retval      无
 */
void rs485_tx_set(uint8_t en)
{
    pcf8574_write_bit(RS485_RE_IO, en);
}




