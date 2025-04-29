/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-26 15:38:11
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-27 23:57:24
 * @FilePath: \Demo\User\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pch.h"
const uint8_t g_text_buf[] = {"STM32 IIC TEST iic_device1"}; 
#define TEXT_SIZE   sizeof(g_text_buf)      /* TEXT 字符串长度 */
static void BSP_Init(void) 
{
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

    SystemClock_Config(360, 25, 2, 8);    /* 设置时钟,180Mhz */  
    delay_init(180);                         /* 延时初始化 */
    led_init();                              /* 初始化LED */
    key_init();                              /* 初始化按键 */
    // IWDG_Init(IWDG_Prescaler_64, 500);       /* 预分频数为64,重载值为500,溢出时间约为1s */
    TIM6_Init(5000 - 1, 9000 - 1);              /* 90 000 000 / 9000 = 10KHz 10KHz的计数频率，计数5K次为500ms */
    My_USART_Init();                  /* 初始化串口2 */
    IIC_INIT();                /* 初始化IIC */
    SPI_Flash_Init(&flash_cfg);
//    rs485_init();
}

  //  5. 接收数据（任务中）
void Serial_RxTask(void *pvParameters) {
    Serial_Device_t *dev = (Serial_Device_t *)pvParameters;
    Serial_RxData_t rx_data;
    while (1) {
        if (Serial_Operations.ReceiveFromBuffer(dev, &rx_data, portMAX_DELAY) == SERIAL_OK) {
            // 处理 rx_data.data
        }
    }
}
  //  6. 读取错误日志（任务中）
void Error_Log_Task(void *pvParameters) {
    Serial_ErrorLog_t log;
    while (1) {
        if (Serial_Operations.GetErrorLog(&log, portMAX_DELAY) == SERIAL_OK) {
            // 处理 log（type, timestamp, instance）
        }
    }
}
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
   BSP_Init();
   printf("BSP_Init success!\r\n");
   delay_ms(100);                           /* 延时100ms再初始化看门狗,LED0的变化"可见" */


  //  1. 定义RS485设备（TIM2）
   Serial_Device_t RS485_Device = {
       .instance = USART2,
       .tx_port = GPIOA,
       .tx_pin = GPIO_Pin_2,
       .rx_port = GPIOA,
       .rx_pin = GPIO_Pin_3,
       .de_port = GPIOG,
       .de_pin = GPIO_Pin_8,
       .baudrate = 115200,
       .af = GPIO_AF_USART2,
       .irqn = USART2_IRQn,
       .slave_addr = 0x01,
       .mode = RS485_MODE,
       .timer = TIM2,
       .timer_irqn = TIM2_IRQn,
       .rx_buffer = {0}
   };
  
  //  2. 定义UART设备
   Serial_Device_t UART_Device = {
       .instance = USART1,
       .tx_port = GPIOA,
       .tx_pin = GPIO_Pin_9,
       .rx_port = GPIOA,
       .rx_pin = GPIO_Pin_10,
       .de_port = NULL,
       .de_pin = 0,
       .baudrate = 115200,
       .af = GPIO_AF_USART1,
       .irqn = USART1_IRQn,
       .slave_addr = 0,
       .mode = UART_MODE,
       .timer = NULL,
       .timer_irqn = 0,
       .rx_buffer = {0}
   };
  
  //  3. 初始化
   Serial_Operations.Init(&RS485_Device);
   Serial_Operations.Init(&UART_Device);
  
  //  4. 发送数据
   uint8_t rs485_data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B}; // Modbus
   Serial_Operations.SendData(&RS485_Device, rs485_data, 8);
   uint8_t uart_data[] = {0xAA, 0x05, 0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x55}; // [0xAA][5]["Hello"][0x55]
   Serial_Operations.SendData(&UART_Device, uart_data, 8);
  

   xTaskCreate(Serial_RxTask, "RS485_Rx", 256, &RS485_Device, 1, NULL);
   xTaskCreate(Serial_RxTask, "UART_Rx", 256, &UART_Device, 1, NULL);
  

   xTaskCreate(Error_Log_Task, "Error_Log", 256, NULL, 1, NULL);
  
  //  7. 释放资源
   Serial_Operations.Deinit(&RS485_Device);
   Serial_Operations.Deinit(&UART_Device);



   // 创建队列
  //  cmdQueue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(char[64]));

   // 创建任务
  //  xTaskCreate(vWifiTask, "WifiTask", WIFI_TASK_STACK_SIZE, NULL, 1, NULL);
   
   // 启动调度器
   vTaskStartScheduler();
   // freertos_demo();  
   /* 运行 FreeRTOS 例程 */ 

}


