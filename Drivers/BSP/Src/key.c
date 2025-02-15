#include "pch.h"
/**
 * @brief       按键初始化函数
 * @param       无
 * @retval      无
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;                          /* GPIO配置参数存储变量*/
    KEY0_GPIO_CLK_ENABLE();                                     /* KEY0时钟使能 */
    KEY1_GPIO_CLK_ENABLE();                                     /* KEY1时钟使能 */
    KEY2_GPIO_CLK_ENABLE();                                     /* KEY2时钟使能 */
    WKUP_GPIO_CLK_ENABLE();                                     /* WKUP时钟使能 */

    gpio_init_struct.Pin = KEY0_GPIO_PIN;                       /* KEY0引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* 上拉­ */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);           /* KEY0引脚模式设置上拉输入 */

    gpio_init_struct.Pin = KEY1_GPIO_PIN;                       /* KEY1引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* 上拉­ */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct);           /* KEY1引脚模式设置上拉输入 */

    gpio_init_struct.Pin = KEY2_GPIO_PIN;                       /* KEY2引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* 上拉­ */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(KEY2_GPIO_PORT, &gpio_init_struct);           /* KEY2引脚模式设置上拉输入 */

    gpio_init_struct.Pin = WKUP_GPIO_PIN;                       /* WKUP引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLDOWN;                      /* 下拉­ */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(WKUP_GPIO_PORT, &gpio_init_struct);           /* WKUP引脚模式设置下拉输入 */

}

/**
 * @brief       °´¼üÉ¨Ãèº¯Êý
 * @note        ¸Ãº¯ÊýÓÐÏìÓ¦ÓÅÏÈ¼¶(Í¬Ê±°´ÏÂ¶à¸ö°´¼ü): WK_UP > KEY2 > KEY1 > KEY0!!
 * @param       mode:0 / 1, ¾ßÌåº¬ÒåÈçÏÂ:
 *   @arg       0,  ²»Ö§³ÖÁ¬Ðø°´(µ±°´¼ü°´ÏÂ²»·ÅÊ±, Ö»ÓÐµÚÒ»´Îµ÷ÓÃ»á·µ»Ø¼üÖµ,
 *                  ±ØÐëËÉ¿ªÒÔºó, ÔÙ´Î°´ÏÂ²Å»á·µ»ØÆäËû¼üÖµ)
 *   @arg       1,  Ö§³ÖÁ¬Ðø°´(µ±°´¼ü°´ÏÂ²»·ÅÊ±, Ã¿´Îµ÷ÓÃ¸Ãº¯Êý¶¼»á·µ»Ø¼üÖµ)
 * @retval      ¼üÖµ, ¶¨ÒåÈçÏÂ:
 *              KEY0_PRES, 1, KEY0°´ÏÂ
 *              KEY1_PRES, 2, KEY1°´ÏÂ
 *              KEY2_PRES, 3, KEY2°´ÏÂ
 *              WKUP_PRES, 4, WKUP°´ÏÂ
 */
uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;  /* 按键松开标志 */
    uint8_t keyval = 0;

    if (mode) key_up = 1;       /* 支持连按 */

    if (key_up && (KEY0 == 0 || KEY1 == 0 || KEY2 == 0 || WK_UP == 1))  /* 按键松开标志为1，且有任意按键按下了 */
    {
        delay_ms(10);           /* 去抖动 */
        key_up = 0;

        if (KEY0 == 0)  keyval = KEY0_PRES;

        if (KEY1 == 0)  keyval = KEY1_PRES;

        if (KEY2 == 0)  keyval = KEY2_PRES;

        if (WK_UP == 1) keyval = WKUP_PRES;
    }
    else if (KEY0 == 1 && KEY1 == 1 && KEY2 == 1 && WK_UP == 0)         /* 没有任何按键按下，标记按键松开 */
    {
        key_up = 1;
    }

    return keyval;              /* 返回键值 */
}

