/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-26 15:38:11
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-03-23 18:56:14
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
    key_init();                              /* 初始化按键 */
    delay_ms(100);                           /* 延时100ms再初始化看门狗,LED0的变化"可见" */
    // IWDG_Init(IWDG_Prescaler_64, 500);       /* 预分频数为64,重载值为500,溢出时间约为1s */
    TIM6_Init(5000 - 1, 9000 - 1);              /* 90 000 000 / 9000 = 10KHz 10KHz的计数频率，计数5K次为500ms */
    USART_InitWithInterrupt(&USART1_Config, &GPIO1_Config); /* 初始化串口1 */
    printf("USART1 Initialized!\r\n");
    LED0(0);                                    /* 点亮LED0(红灯) */
    Flash_init(&flash_cfg);
    printf("SPI FLASH Initialized!\r\n");

    uint8_t data[256];

    uint8_t key;
    uint16_t i = 0;
    while (1)
    {
        key = key_scan(0);

        if (key == KEY1_PRES)                                                        /* KEY1按下,写入W25Q128 */
        {
            printf("read sector...\r\n");
            SPI_Flash_ReadData(&flash_cfg, data, 0x0000, 256);                    /* 从倒数第100个地址处开始,读出SIZE个字节 */
            for (uint16_t i = 0; i < 256; i++) {
                printf("%d ", data[i]);
            }
            printf("\r\n");
            printf("Erasing sector...\r\n");
            SPI_Flash_EraseSector(&flash_cfg, 0x0000);
            printf("Erasing sector finish\r\n");
            printf("read sector...\r\n");
            SPI_Flash_ReadData(&flash_cfg, data, 0x0000, 256);                    /* 从倒数第100个地址处开始,读出SIZE个字节 */
            for (uint16_t i = 0; i < 256; i++) {
                printf("%d ", data[i]);
            }
            printf("\r\n");

            printf("flash write 0 ~ 255\r\n");
            for (uint16_t i = 0; i < 256; i++) {
                data[i] = i;
                printf("%d ", data[i]);
            }
            printf("\r\n");
            printf("Writing page ...\r\n");
            SPI_Flash_WritePage(&flash_cfg, data, 0x0000, 256);
            printf("Writing page finish\r\n");
            printf("Data read:\r\n");
            SPI_Flash_ReadData(&flash_cfg, data, 0x0000, 256);                    /* 从倒数第100个地址处开始,读出SIZE个字节 */
            for (uint16_t i = 0; i < 256; i++) {
                printf("%d ", data[i]);
            }
            printf("\r\n");
        }
        if (key == KEY0_PRES)                                                        /* KEY0按下,读取字符串并显示 */
        {
            printf("read sector...\r\n");
            SPI_Flash_ReadData(&flash_cfg, data, 0x0000, 256);                    /* 从倒数第100个地址处开始,读出SIZE个字节 */
            for (uint16_t i = 0; i < 256; i++) {
                printf("%d ", data[i]);
            }
            printf("\r\n");
            printf("Erasing sector...\r\n");
            SPI_Flash_EraseSector(&flash_cfg, 0x0000);
            printf("Erasing sector finish\r\n");
            printf("read sector...\r\n");
            SPI_Flash_ReadData(&flash_cfg, data, 0x0000, 256);                    /* 从倒数第100个地址处开始,读出SIZE个字节 */
            for (uint16_t i = 0; i < 256; i++) {
                printf("%d ", data[i]);
            }
            printf("\r\n");
            printf("Writing page...\r\n");
            for (uint16_t i = 0; i < 256; i++) {
                data[i] = 255 - i;
                printf("%d ", data[i]);
            }
            printf("\r\n");
            SPI_Flash_WritePage(&flash_cfg, data, 0x0000, 256);
            printf("Writing page finish\r\n");
            printf("Data read:\r\n");
            SPI_Flash_ReadData(&flash_cfg, data, 0x0000, 256);                    /* 从倒数第100个地址处开始,读出SIZE个字节 */
            for (uint16_t i = 0; i < 256; i++) {
                printf("%d ", data[i]);
            }
            printf("\r\n");
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


