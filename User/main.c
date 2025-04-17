/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-26 15:38:11
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-13 13:50:49
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
    Flash_init(&flash_cfg);
    rs485_init();
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
   // 创建队列
   cmdQueue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(char[64]));

   // 创建任务
   xTaskCreate(vWifiTask, "WifiTask", WIFI_TASK_STACK_SIZE, NULL, 1, NULL);
   
   // 启动调度器
   vTaskStartScheduler();
   // freertos_demo();  
   /* 运行 FreeRTOS 例程 */ 

}


