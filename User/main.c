/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-26 15:38:11
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 01:24:20
 * @FilePath: \Demo\User\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pch.h"
const uint8_t g_text_buf[] = {"STM32 IIC TEST iic_device1"};
#define TEXT_SIZE sizeof(g_text_buf) /* TEXT 字符串长度 */
static void BSP_Init(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  SystemClock_Config(360, 25, 2, 8); /* 设置时钟,180Mhz */
  delay_init(180);                   /* 延时初始化 */
  led_init();                        /* 初始化LED */
  key_init();                        /* 初始化按键 */
  // IWDG_Init(IWDG_Prescaler_64, 500);       /* 预分频数为64,重载值为500,溢出时间约为1s */
  TIM6_Init(5000 - 1, 9000 - 1); /* 90 000 000 / 9000 = 10KHz 10KHz的计数频率，计数5K次为500ms */
  // My_USART_Init();                  /* 初始化串口2 */
  IIC_INIT();                 /* 初始化IIC */
  W25Qxx_Init(&flash_config); // Initialize Flash with the correct configuration
  Serial_Operations.Init(&RS485_Device);
  Serial_Operations.Init(&UART_Device);
}
void Serial_RxTask(void *pvParameters);
void Error_Log_Task(void *pvParameters);
void RTOS_Init(const RTOS_Ops_t *rtos_ops)
{
  // 1. 初始化 RTOS 抽象层
  RTOS_SetOps(&FreeRTOS_Ops);

  // 2. 使用 RTOS_Ops_t 接口
  rtos_ops = RTOS_GetOps();
  rtos_ops->Delay(100); // 延时 100 ticks
}

TaskHandle_t Serial_RxTask_Handle = NULL;
TaskHandle_t Serial_TxTask_Handle = NULL;
TaskHandle_t Error_Log_Task_Handle = NULL;

/**
 * @brief  主函数
 * @param  无
 * @retval 无
 */
int main(void)
{
  const RTOS_Ops_t *rtos_ops;
  RTOS_Init(rtos_ops); // 初始化 RTOS 抽象层

  BSP_Init();
  printf("BSP_Init success!\r\n");
  rtos_ops->Delay(100); /* 延时100ms再初始化看门狗,LED0的变化"可见" */

  //  4. 发送数据
  uint8_t rs485_data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B}; // Modbus
  Serial_Operations.SendData(&RS485_Device, rs485_data, 8);
  uint8_t uart_data[] = {0xAA, 0x05, 0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x55}; // [0xAA][5]["Hello"][0x55]
  Serial_Operations.SendData(&UART_Device, uart_data, 8);

  Serial_RxTask_Handle = rtos_ops->TaskCreate(Serial_RxTask, "RS485_Rx", 256, &RS485_Device, 1);
  Serial_TxTask_Handle = rtos_ops->TaskCreate(Serial_RxTask, "UART_Rx", 256, &UART_Device, 1);

  rtos_ops->TaskCreate(Error_Log_Task, "Error_Log", 256, NULL, 1);

  // 创建队列
  //  cmdQueue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(char[64]));

  // 创建任务
  //  xTaskCreate(vWifiTask, "WifiTask", WIFI_TASK_STACK_SIZE, NULL, 1, NULL);

  // 启动调度器
  vTaskStartScheduler();
  // freertos_demo();
  /* 运行 FreeRTOS 例程 */
}
void Serial_RxTask(void *pvParameters)
{
  Serial_Device_t *dev = (Serial_Device_t *)pvParameters;
  Serial_RxData_t rx_data;
  while (1)
  {
    if (Serial_Operations.ReceiveFromBuffer(dev, &rx_data, portMAX_DELAY) == SERIAL_OK)
    {
    }
  }
}

void Error_Log_Task(void *pvParameters)
{
  Serial_ErrorLog_t log;
  while (1)
  {
    if (Serial_Operations.GetErrorLog(&log, portMAX_DELAY) == SERIAL_OK)
    {
    }
  }
}
