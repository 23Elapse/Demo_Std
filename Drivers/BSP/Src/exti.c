#include "pch.h"

void GPIO_IRQHandler(uint32_t pin);
/**
 * @brief       KEY0 外部中断服务程序
 * @param       无
 * @retval      无
 */
void KEY0_INT_IRQHandler(void)
{
    GPIO_IRQHandler(KEY0_INT_GPIO_PIN);         /* 调用中断处理公用函数，清除中断标志位 */
    EXTI->PR = KEY0_INT_GPIO_PIN;        /* 退出时再清一次中断，避免按键抖动误触发 */
}

/**
 * @brief       KEY1 外部中断服务程序
 * @param       无
 * @retval      无
 */
void KEY1_INT_IRQHandler(void)
{ 
    GPIO_IRQHandler(KEY1_INT_GPIO_PIN);         /* 调用中断处理公用函数，清除中断标志位 */
    EXTI->PR = KEY1_INT_GPIO_PIN;        /* 退出时再清一次中断，避免按键抖动误触发 */
}

/**
 * @brief       KEY2 外部中断服务程序
 * @param       无
 * @retval      无
 */
void KEY2_INT_IRQHandler(void)
{ 
    GPIO_IRQHandler(KEY2_INT_GPIO_PIN);        /* 调用中断处理公用函数，清除中断标志位 */
    EXTI->PR = KEY2_INT_GPIO_PIN;        /* 退出时再清一次中断，避免按键抖动误触发 */
}

/**
 * @brief       WK_UP 外部中断服务程序
 * @param       无
 * @retval      无
 */
void WKUP_INT_IRQHandler(void)
{ 
    GPIO_IRQHandler(WKUP_INT_GPIO_PIN);        /* 调用中断处理公用函数，清除中断标志位 */
    EXTI->PR = WKUP_INT_GPIO_PIN;         /* 退出时再清一次中断，避免按键抖动误触发 */
}

// 配置单个 GPIO 引脚
void GPIO_InitPin(GPIO_Config_TypeDef *gpio_config, EXTI_Config_TypeDef *exti_config, NVIC_Config_TypeDef *nvic_config) 
{
    if (gpio_config != NULL) {
        RCC_AHB1PeriphClockCmd(gpio_config->RCC_AHB1Periph_GPIOx, ENABLE);
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = gpio_config->pin;
        GPIO_InitStructure.GPIO_Mode = gpio_config->mode;
        GPIO_InitStructure.GPIO_PuPd = gpio_config->pull;
        GPIO_InitStructure.GPIO_Speed = gpio_config->speed;
        GPIO_InitStructure.GPIO_OType = gpio_config->otype;
        GPIO_Init(gpio_config->port, &GPIO_InitStructure);
    }
    if (exti_config != NULL) {
        // 使能 SYSCFG 时钟
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
        // 配置 EXTI 线路
        SYSCFG_EXTILineConfig(exti_config->EXTI_PortSourceGPIOx, exti_config->EXTI_PinSourcex);  // 配置中断线路
        EXTI_InitTypeDef EXTI_InitStructure;
        EXTI_InitStructure.EXTI_Line = exti_config->EXTI_Line;  // 使用传入的 pin 配置
        EXTI_InitStructure.EXTI_Mode = exti_config->EXTI_Mode;  // 中断模式
        EXTI_InitStructure.EXTI_Trigger = exti_config->EXTI_Trigger;  // 默认配置为上升沿触发
        EXTI_InitStructure.EXTI_LineCmd = exti_config->EXTI_LineCmd;  // 使能该中断线路
        EXTI_Init(&EXTI_InitStructure);
    }
    if (nvic_config != NULL) {
        // 配置 NVIC 中断
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel =  nvic_config->NVIC_IRQChannel;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = nvic_config->NVIC_IRQChannelPreemptionPriority;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = nvic_config->NVIC_IRQChannelSubPriority;
        NVIC_InitStructure.NVIC_IRQChannelCmd = nvic_config->NVIC_IRQChannelCmd;
        NVIC_Init(&NVIC_InitStructure);
    }
    
}
// 配置结构体数组
GPIO_Config_TypeDef gpio_configs[] = {
    { RCC_AHB1Periph_GPIOH, KEY0_INT_GPIO_PORT, KEY0_INT_GPIO_PIN, GPIO_Mode_IN, NULL, GPIO_PuPd_UP, GPIO_High_Speed},
    { RCC_AHB1Periph_GPIOH, KEY1_INT_GPIO_PORT, KEY1_INT_GPIO_PIN, GPIO_Mode_IN, NULL, GPIO_PuPd_UP, GPIO_High_Speed},
    { RCC_AHB1Periph_GPIOC, KEY2_INT_GPIO_PORT, KEY2_INT_GPIO_PIN, GPIO_Mode_IN, NULL, GPIO_PuPd_UP, GPIO_High_Speed},
    { RCC_AHB1Periph_GPIOA, WKUP_INT_GPIO_PORT, WKUP_INT_GPIO_PIN, GPIO_Mode_IN, NULL, GPIO_PuPd_DOWN, GPIO_High_Speed}
};
EXTI_Config_TypeDef exti_configs[] = {
    { KEY0_INT_EXTI_LINE, EXTI_Mode_Interrupt, EXTI_Trigger_Falling, ENABLE, KEY0_INT_EXTI_PORTSOURCE, KEY0_INT_EXTI_PINSOURCE},
    { KEY1_INT_EXTI_LINE, EXTI_Mode_Interrupt, EXTI_Trigger_Falling, ENABLE, KEY1_INT_EXTI_PORTSOURCE, KEY1_INT_EXTI_PINSOURCE},
    { KEY2_INT_EXTI_LINE, EXTI_Mode_Interrupt, EXTI_Trigger_Falling, ENABLE, KEY2_INT_EXTI_PORTSOURCE, KEY2_INT_EXTI_PINSOURCE},
    { WKUP_INT_EXTI_LINE, EXTI_Mode_Interrupt, EXTI_Trigger_Rising, ENABLE, WKUP_INT_EXTI_PORTSOURCE, WKUP_INT_EXTI_PINSOURCE}
};
NVIC_Config_TypeDef nvic_configs[] = {
    { KEY0_INT_IRQn, 0, 2, ENABLE},
    { KEY1_INT_IRQn, 1, 2, ENABLE},
    { KEY2_INT_IRQn, 2, 2, ENABLE},
    { WKUP_INT_IRQn, 3, 2, ENABLE}
};
uint32_t num_pins = sizeof(gpio_configs) / sizeof(GPIO_Config_TypeDef);

// 配置多个引脚
void GPIO_InitMultiplePins(GPIO_Config_TypeDef *gpio_config, EXTI_Config_TypeDef *exti_config, NVIC_Config_TypeDef *nvic_config, uint32_t num_pins) {

    for (uint32_t i = 0; i < num_pins; i++) {
        GPIO_InitPin(&gpio_config[i], &exti_config[i], &nvic_config[i]);
    }
}

void GPIO_IRQHandler(uint32_t GPIO_Pin)
{
    delay_ms(20);                                       /* 消抖 */
    switch (GPIO_Pin)
    {
        case KEY0_INT_GPIO_PIN:
            if (KEY0 == 0)
            {
                LED1_TOGGLE();                          /* LED1状态取反 */ 
                LED0_TOGGLE();                          /* LED0状态取反 */ 
            }
            break;

        case KEY1_INT_GPIO_PIN:
            if (KEY1 == 0)
            {
                LED1_TOGGLE();                          /* LED1 状态取反 */ 
            }
            break;

        case KEY2_INT_GPIO_PIN:
            if (KEY2 == 0)
            {
                LED0_TOGGLE();                          /* LED0 状态取反 */ 
            }
            break;

        case WKUP_INT_GPIO_PIN:
            if (WK_UP == 1)
            {
                LED1_TOGGLE();                          /* LED1状态取反 */

                if (GPIO_ReadInputDataBit(LED1_GPIO_PORT, LED1_GPIO_PIN) == 1)
                 {
                   LED0(0);
                 }
                 else 
                 {
                     LED0(1);
                 }
            }
            break;

        default : break;
    }
}

/**
 * @brief       外部中断初始化程序
 * @param       无
 * @retval      无
 */
void extix_init(void)
{
    GPIO_InitMultiplePins(gpio_configs, exti_configs, nvic_configs, num_pins);
}

