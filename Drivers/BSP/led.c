#include "pch.h"

/**
 * @brief       初始化LED相关IO口，并使能时钟
 * @param       无
 * @retval      无
 */
void led_init(void)
{
    
    GPIO_InitTypeDef gpio_init_struct;
    LED0_GPIO_CLK_ENABLE();
    LED1_GPIO_CLK_ENABLE(); 
    gpio_init_struct.GPIO_Pin = LED0_GPIO_PIN;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_init_struct.GPIO_OType  = GPIO_OType_PP;
    gpio_init_struct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(LED0_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.GPIO_Pin = LED1_GPIO_PIN;
    GPIO_Init(LED1_GPIO_PORT,&gpio_init_struct);

    LED0(1);
    LED1(1);
}

