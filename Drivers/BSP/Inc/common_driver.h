/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 10:00:00
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-27 11:30:00
 * @FilePath: \Demo\Drivers\BSP\Inc\common_driver.h
 * @Description: 通用驱动接口，供RS485、UART、CAN、定时器等设备复用
 *
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved.
 */

#ifndef COMMON_DRIVER_H
#define COMMON_DRIVER_H

#include "stm32f4xx.h"

/**
 * @brief  通用状态枚举
 */
typedef enum {
    COMMON_OK = 0,          // 操作成功
    COMMON_ERR_INIT         // 初始化失败
} Common_Status;

/**
 * @brief  初始化GPIO
 * @param  port: GPIO端口（如GPIOA）
 * @param  pin: 引脚掩码（如GPIO_Pin_9）
 * @param  mode: 模式（如GPIO_Mode_AF、GPIO_Mode_OUT）
 * @param  otype: 输出类型（如GPIO_OType_PP）
 * @param  pupd: 上下拉（如GPIO_PuPd_UP）
 * @param  speed: 速度（如GPIO_Speed_50MHz）
 * @param  af: 复用功能（如GPIO_AF_USART1），仅mode为GPIO_Mode_AF时有效
 * @retval Common_Status 状态码
 */
//Common_Status Common_GPIO_Init(GPIO_TypeDef *port, uint16_t pin, uint32_t mode,
//                              uint32_t otype, uint32_t pupd, uint32_t speed, uint8_t af);
Common_Status Common_GPIO_Init(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode,
                              GPIOOType_TypeDef otype, GPIOPuPd_TypeDef pupd, GPIOSpeed_TypeDef speed, uint8_t af);

/**
 * @brief  初始化USART
 * @param  instance: USART实例（如USART1）
 * @param  baudrate: 波特率（如115200）
 * @param  word_length: 数据位（如USART_WordLength_8b）
 * @param  stop_bits: 停止位（如USART_StopBit_1）
 * @param  parity: 奇偶校验（如USART_Parity_No）
 * @retval Common_Status 状态码
 */
Common_Status Common_USART_Init(USART_TypeDef *instance, uint32_t baudrate,
                               uint16_t word_length, uint16_t stop_bits, uint16_t parity);

/**
 * @brief  初始化定时器
 * @param  instance: 定时器实例（如TIM2）
 * @param  period_us: 定时周期（微秒）
 * @param  irqn: 中断号（如TIM2_IRQn）
 * @retval Common_Status 状态码
 */
Common_Status Common_TIM_Init(TIM_TypeDef *instance, uint32_t period_us, uint8_t irqn);

#endif /* COMMON_DRIVER_H */

