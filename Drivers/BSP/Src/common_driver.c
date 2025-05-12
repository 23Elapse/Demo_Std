/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 10:00:00
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-05 21:45:55
 * @FilePath: \Demo\Drivers\BSP\Src\common_driver.c
 * @Description: 通用驱动实现，供 RS485、UART、CAN、定时器等设备复用
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "common_driver.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "log_system.h"
#include "pch.h"
/**
 * @brief 获取 GPIO 引脚源编号
 * @param pin GPIO 引脚掩码（如 GPIO_Pin_9）
 * @return 引脚源编号 (0-15)
 */
static uint8_t GetPinSource(uint16_t pin)
{
    uint8_t source = 0;
    while (pin != 0)
    {
        pin >>= 1;
        source++;
    }
    return source - 1;
}

/**
 * @brief 初始化 GPIO
 */
Common_Status Common_GPIO_Init(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode,
                               GPIOOType_TypeDef otype, GPIOPuPd_TypeDef pupd, GPIOSpeed_TypeDef speed, uint8_t af)
{
    if (!port || !pin)
    {
        Log_Message(LOG_LEVEL_ERROR, "[Common] Invalid GPIO port or pin");
        return COMMON_ERR_INIT;
    }

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA << ((uint32_t)port - GPIOA_BASE) / 0x400, ENABLE);

    GPIO_InitTypeDef gpio_init = {
        .GPIO_Pin = pin,
        .GPIO_Mode = mode,
        .GPIO_OType = otype,
        .GPIO_PuPd = pupd,
        .GPIO_Speed = speed};
    GPIO_Init(port, &gpio_init);

    if (mode == GPIO_Mode_AF)
    {
        GPIO_PinAFConfig(port, GetPinSource(pin), af);
    }

    return COMMON_OK;
}

/**
 * @brief 初始化 USART
 */
Common_Status Common_USART_Init(USART_TypeDef *instance, uint32_t baudrate,
                                uint16_t word_length, uint16_t stop_bits, uint16_t parity)
{
    if (!instance)
    {
        Log_Message(LOG_LEVEL_ERROR, "[Common] Invalid USART instance");
        return COMMON_ERR_INIT;
    }

    if (instance == USART1 || instance == USART6)
    {
        RCC_APB2PeriphClockCmd((instance == USART1) ? RCC_APB2Periph_USART1 : RCC_APB2Periph_USART6, ENABLE);
    }
    else if (instance == USART2 || instance == USART3 || instance == UART4 || instance == UART5)
    {
        RCC_APB1PeriphClockCmd((instance == USART2) ? RCC_APB1Periph_USART2 : (instance == USART3) ? RCC_APB1Periph_USART3
                                                                          : (instance == UART4)    ? RCC_APB1Periph_UART4
                                                                                                   : RCC_APB1Periph_UART5,
                               ENABLE);
    }
    else
    {
        Log_Message(LOG_LEVEL_ERROR, "[Common] Unsupported USART instance");
        return COMMON_ERR_INIT;
    }

    USART_InitTypeDef usart_init = {
        .USART_BaudRate = baudrate,
        .USART_WordLength = word_length,
        .USART_StopBits = stop_bits,
        .USART_Parity = parity,
        .USART_Mode = USART_Mode_Tx | USART_Mode_Rx,
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None};
    USART_Init(instance, &usart_init);

    USART_Cmd(instance, ENABLE);
    return COMMON_OK;
}

/**
 * @brief 初始化定时器
 */
Common_Status Common_TIM_Init(TIM_TypeDef *instance, uint32_t period_us, uint8_t irqn)
{
    if (!instance)
    {
        Log_Message(LOG_LEVEL_ERROR, "[Common] Invalid TIM instance");
        return COMMON_ERR_INIT;
    }

    if (instance == TIM2 || instance == TIM3 || instance == TIM4)
    {
        RCC_APB1PeriphClockCmd((instance == TIM2) ? RCC_APB1Periph_TIM2 : (instance == TIM3) ? RCC_APB1Periph_TIM3
                                                                                             : RCC_APB1Periph_TIM4,
                               ENABLE);
    }
    else
    {
        Log_Message(LOG_LEVEL_ERROR, "[Common] Unsupported TIM instance");
        return COMMON_ERR_INIT;
    }

    TIM_TimeBaseInitTypeDef tim_init = {
        .TIM_Prescaler = SystemCoreClock / 1000000 - 1,
        .TIM_CounterMode = TIM_CounterMode_Up,
        .TIM_Period = period_us - 1,
        .TIM_ClockDivision = TIM_CKD_DIV1,
        .TIM_RepetitionCounter = 0};
    TIM_TimeBaseInit(instance, &tim_init);

    TIM_ITConfig(instance, TIM_IT_Update, ENABLE);
    NVIC_InitTypeDef nvic_init = {
        .NVIC_IRQChannel = irqn,
        .NVIC_IRQChannelPreemptionPriority = 0,
        .NVIC_IRQChannelSubPriority = 1,
        .NVIC_IRQChannelCmd = ENABLE};
    NVIC_Init(&nvic_init);

    return COMMON_OK;
}

