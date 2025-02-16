/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-26 15:38:11
 * @LastEditors: Elapse userszy@163.com
 * @LastEditTime: 2024-12-21 17:19:09
 * @FilePath: \Demo\User\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pch.h"


/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
    NVIC_SetPriorityGrouping(2);     
    SystemClock_Config(360, 25, 2, 8);    /* 设置时钟,180Mhz */  
    delay_init(180);                         /* 延时初始化 */
    led_init();                              /* 初始化LED */
//    key_init();                              /* 初始化按键 */
//    extix_init();                            /* 初始化外部中断输入 */
//    LED0(0);                                 /* 先点亮红灯 */
    // 配置 USART1
    USART_ConfigTypeDef USART1_Config = {
        .USARTx = USART1,
        .BaudRate = 115200,
        .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
        .USART_Parity = USART_Parity_No,
        .USART_StopBits = USART_StopBits_1,
        .USART_WordLength = USART_WordLength_8b,
        .USART_IRQChannel = USART1_IRQn,
        .NVIC_Priority = 0,
        .GPIOx = GPIOA,
        .GPIO_Pin_TX = GPIO_Pin_9,
        .GPIO_Pin_RX = GPIO_Pin_10,
        .GPIO_AF = GPIO_AF_USART1
    };
    USART_InitWithInterrupt(&USART1_Config);

    // // 配置 USART2
    // USART_ConfigTypeDef USART2_Config = {
    //     .USARTx = USART2,
    //     .BaudRate = 9600,
    //     .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
    //     .USART_Parity = USART_Parity_No,
    //     .USART_StopBits = USART_StopBits_1,
    //     .USART_WordLength = USART_WordLength_8b,
    //     .USART_IRQChannel = USART2_IRQn,
    //     .NVIC_Priority = 1,
    //     .GPIOx = GPIOA,
    //     .GPIO_Pin_TX = GPIO_Pin_2,
    //     .GPIO_Pin_RX = GPIO_Pin_3,
    //     .GPIO_AF = GPIO_AF_USART2
    // };
    // USART_InitWithInterrupt(&USART2_Config);
    // 初始化USART1，波特率115200
    // USART1_Init(115200);

    // 测试消息
    printf("USART1 Initialized!\r\n");

    while(1) 
    {
        // 接收数据处理（回显）
        uint8_t buf[RX_BUFFER_SIZE];
        uint16_t len = USART1_ReceiveData(buf, sizeof(buf));
        if (len > 0) 
        {
            printf("Received: ");
            for (uint16_t i = 0; i < len; i++) 
            {
                printf("%c", buf[i]); // 回显接收到的字符
            }
            printf("\r\n");
        }
    }
}


