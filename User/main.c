/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-26 15:38:11
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-03-30 20:27:08
 * @FilePath: \Demo\User\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pch.h"
const uint8_t g_text_buf[] = {"STM32 IIC TEST iic_device1"}; 
#define TEXT_SIZE   sizeof(g_text_buf)      /* TEXT 字符串长度 */
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
    uint8_t key; 
    uint16_t i = 0; 
    uint8_t datatemp[TEXT_SIZE]; 
    NVIC_SetPriorityGrouping(2);     
    SystemClock_Config(360, 25, 2, 8);    /* 设置时钟,180Mhz */  
    delay_init(180);                         /* 延时初始化 */
    led_init();                              /* 初始化LED */
    key_init();                              /* 初始化按键 */
    delay_ms(100);                           /* 延时100ms再初始化看门狗,LED0的变化"可见" */
    // IWDG_Init(IWDG_Prescaler_64, 500);       /* 预分频数为64,重载值为500,溢出时间约为1s */
    TIM6_Init(5000 - 1, 9000 - 1);              /* 90 000 000 / 9000 = 10KHz 10KHz的计数频率，计数5K次为500ms */
    USART_InitWithInterrupt(&USART1_Config, &GPIO1_Config); /* 初始化串口1 */
    printf("USART1 Initialized!\r\n");

    IIC_INIT();                /* 初始化IIC */

    printf("AT24CXX Initialized!\r\n");
    Flash_init(&flash_cfg);
    printf("SPI FLASH Initialized!\r\n");
    rs485_init();
    while (IIC_Check(&IIC1_EEPROM))  /* 检测不到 24c02 */ 
    {   
        LED0_TOGGLE();       /* 红灯闪烁 */ 
    } 
    printf("AT24CXX Check OK!\r\n");
    while (1)
    {
        key = key_scan(0);
        if (key == KEY1_PRES)   /* KEY1 按下,写入 24C02 */ 
        { 
            IIC_Write(&IIC1_EEPROM, 0, (uint8_t *)g_text_buf, TEXT_SIZE); 
            printf("AT24CXX Write Data: %s\r\n", g_text_buf);
        } 
        if (key == KEY0_PRES)                                                        /* KEY1按下,写入W25Q128 */
        {
            IIC_Read(&IIC1_EEPROM, 0, datatemp, TEXT_SIZE); 
            printf("AT24CXX Read Data: %s\r\n", datatemp); 
        }
        if (key == WKUP_PRES)                                                        /* KEY1按下,写入W25Q128 */
        {
            printf("AT24CXX Read Data: %s\r\n", datatemp); 
        }
        i++;
        delay_ms(10);

        if (i == 20)
        {
            LED0_TOGGLE();            /* 提示系统正在运行 */
            i = 0;
        }
    }
}


