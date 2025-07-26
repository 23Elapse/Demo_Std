/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-26 15:38:11
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-07-26 16:47:05
 * @FilePath: \Demo_Std_F407\User\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pch.h"
#include <stdio.h>
#include "app_tasks.h"
#include "serial_interface.h"
#include "dev_config.h" // 包含设备定义
#include "core_cm4.h"
  // static void BSP_Init(void)
  // {
  //   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  //   SystemClock_Config(360, 25, 2, 8); /* 设置时钟,180Mhz */
  //   delay_init(180);                   /* 延时初始化 */
  //   led_init();                        /* 初始化LED */
  //   key_init();                        /* 初始化按键 */
  //   // IWDG_Init(IWDG_Prescaler_64, 500);       /* 预分频数为64,重载值为500,溢出时间约为1s */
  //   TIM6_Init(5000 - 1, 9000 - 1); /* 90 000 000 / 9000 = 10KHz 10KHz的计数频率，计数5K次为500ms */

  //   // Serial_Init(&UART_Device);
  //   Serial_Driver_Init(&RS485_Device); /* 初始化串口设备 */
  //   printf("[App] UART Device Initialized: %s\r\n", RS485_Device.instance == USART2 ? "USART2" : "USART1");

  // }


// 重定义 fputc，用于 printf 输出
int fputc(int ch, FILE *f) {
    // 选择一个串口作为调试输出口，例如 g_uart_dev
    while ((g_uart_dev.instance->SR & 0X40) == 0);
    g_uart_dev.instance->DR = (uint8_t)ch;
    return ch;
}

/**
 * @brief  执行基础的板级支持包(BSP)初始化
 * @note   这部分只包含独立于RTOS和应用逻辑的最底层硬件初始化。
 */
static void BSP_Init(void)
{
    // 1. NVIC中断优先级分组设置 (必须在所有中断配置之前)
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    // 2. 配置系统时钟
    if (SystemClock_Config(8, 336, 2, 7) != 0)
    {
        // 配置失败，做错误处理
        while(1);
    }
//    NVIC_SetPriority(SysTick_IRQn, configKERNEL_INTERRUPT_PRIORITY);
    SystemCoreClockUpdate();

    delay_init(); // 初始化延时函数
    // 其他非常基础的硬件初始化，如LED、KEY等可以保留
    led_init();
    key_init();
    Serial_Driver_Init(&g_uart_dev); /* 初始化串口设备 */
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
    // 步骤 1: 执行最底层的硬件初始化
    BSP_Init();

    // 步骤 2: 将所有应用层的初始化和任务创建工作，全部交给 App_Init()。
    // App_Init 内部会负责设置RTOS抽象层、初始化所有驱动和创建任务。
    App_Init();

    // App_Init() 内部会启动RTOS调度器，代码不应执行到这里。
    while (1)
    {
    }
}

