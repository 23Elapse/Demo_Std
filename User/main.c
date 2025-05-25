/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-26 15:38:11
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-25 18:14:15
 * @FilePath: \Demo\User\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pch.h"
#include "app_tasks.h"
#include "serial_interface.h"
static void BSP_Init(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  SystemClock_Config(360, 25, 2, 8); /* 设置时钟,180Mhz */
  delay_init(180);                   /* 延时初始化 */
  led_init();                        /* 初始化LED */
  key_init();                        /* 初始化按键 */
  // IWDG_Init(IWDG_Prescaler_64, 500);       /* 预分频数为64,重载值为500,溢出时间约为1s */
  TIM6_Init(5000 - 1, 9000 - 1); /* 90 000 000 / 9000 = 10KHz 10KHz的计数频率，计数5K次为500ms */

  // Serial_Init(&UART_Device);
  Serial_Driver_Init(&RS485_Device); /* 初始化串口设备 */
  printf("[App] UART Device Initialized: %s\r\n", RS485_Device.instance == USART2 ? "USART2" : "USART1");

}


/**
 * @brief  主函数
 * @param  无
 * @retval 无
 */
int main(void)
{
    // 设置全局 RTOS 操作接口
  g_rtos_ops = &FreeRTOS_Ops;
  if (!g_rtos_ops)
  {
      Log_Message(LOG_LEVEL_ERROR, "[App] RTOS ops not initialized");
  }
  BSP_Init();
  App_Init();
}

