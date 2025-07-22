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

    gpio_init_struct.GPIO_Pin = KEY0_GPIO_PIN;                       /* KEY0引脚 */
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;                    /* 输入 */
    gpio_init_struct.GPIO_PuPd = GPIO_PuPd_UP;                        /* 上拉­ */
    gpio_init_struct.GPIO_Speed = GPIO_High_Speed;              /* 高速 */
    GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);           /* KEY0引脚模式设置上拉输入 */

    gpio_init_struct.GPIO_Pin = KEY1_GPIO_PIN;                       /* KEY1引脚 */
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;                    /* 输入 */
    gpio_init_struct.GPIO_PuPd = GPIO_PuPd_UP;                        /* 上拉­ */
    gpio_init_struct.GPIO_Speed = GPIO_High_Speed;              /* 高速 */
    GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct);           /* KEY1引脚模式设置上拉输入 */

    gpio_init_struct.GPIO_Pin = KEY2_GPIO_PIN;                       /* KEY2引脚 */
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;                    /* 输入 */
    gpio_init_struct.GPIO_PuPd = GPIO_PuPd_UP;                        /* 上拉­ */
    gpio_init_struct.GPIO_Speed = GPIO_High_Speed;              /* 高速 */
    GPIO_Init(KEY2_GPIO_PORT, &gpio_init_struct);           /* KEY2引脚模式设置上拉输入 */

    gpio_init_struct.GPIO_Pin = WKUP_GPIO_PIN;                       /* WKUP引脚 */
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;                    /* 输入 */
    gpio_init_struct.GPIO_PuPd = GPIO_PuPd_DOWN;                      /* 下拉­ */
    gpio_init_struct.GPIO_Speed = GPIO_High_Speed;              /* 高速 */
    GPIO_Init(WKUP_GPIO_PORT, &gpio_init_struct);           /* WKUP引脚模式设置下拉输入 */

}

/**
 * @brief       按键扫描函数
 * @param       mode: 0，支持连按; 1，不支持连按
 * @retval      返回按键值
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

        if (KEY0 == 0)  
            keyval = KEY0_PRES;

        if (KEY1 == 0)  
            keyval = KEY1_PRES;

        if (KEY2 == 0)  
            keyval = KEY2_PRES;

        if (WK_UP == 1) 
            keyval = WKUP_PRES;
    }
    else if (KEY0 == 1 && KEY1 == 1 && KEY2 == 1 && WK_UP == 0)         /* 没有任何按键按下，标记按键松开 */
    {
        key_up = 1;
    }

    return keyval;              /* 返回键值 */
}

