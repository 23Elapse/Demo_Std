/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-05 21:24:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-02 21:35:03
 * @FilePath: \Demo\Drivers\BSP\Inc\pcf8574.h
 * @Description: PCF8574 IIC 扩展 IO 驱动头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __PCF8574_H
#define __PCF8574_H

#include "iic_core.h"
#include "log_system.h"

/* PCF8574 引脚定义 */
#define PCF8574_GPIO_PORT GPIOB
#define PCF8574_GPIO_PIN GPIO_Pin_12
#define PCF8574_GPIO_CLK_ENABLE()                             \
    do                                                        \
    {                                                         \
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); \
    } while (0)

/* PCF8574 INT 引脚读取 */
#define PCF8574_INT GPIO_ReadOutputDataBit(PCF8574_GPIO_PORT, PCF8574_GPIO_PIN)

/* PCF8574 设备地址（7位地址，左移一位后为 0x40） */
#define PCF8574_ADDR 0x40

/* PCF8574 IO 功能定义 */
#define BEEP_IO 0      /* 蜂鸣器控制引脚 */
#define AP_INT_IO 1    /* AP3216C 中断引脚 */
#define DCMI_PWDN_IO 2 /* DCMI 电源控制引脚 */
#define USB_PWR_IO 3   /* USB 电源控制引脚 */
#define EX_IO 4        /* 扩展 IO */
#define MPU_INT_IO 5   /* SH3001 中断引脚 */
#define RS485_RE_IO 6  /* RS485_RE 引脚 */
#define ETH_RESET_IO 7 /* 以太网复位引脚 */

/* 全局变量声明 */
extern const IIC_Ops_t IIC1_PCF8574;

/* 函数声明 */
/**
 * @brief 初始化 PCF8574 设备
 * @return 0: 成功, 1: 失败
 */
uint8_t pcf8574_init(void);

/**
 * @brief 设置 RS485 发送模式
 * @param en 1: 使能发送, 0: 禁用发送
 * @return IIC_Status 操作状态
 */
IIC_Status rs485_tx_set(uint8_t en);

/**
 * @brief 设置 PCF8574 某个 IO 的高低电平
 * @param bit 要设置的 IO 编号 (0~7)
 * @param sta IO 状态 (0: 低电平, 1: 高电平)
 * @return IIC_Status 操作状态
 */
IIC_Status PCF8574_WriteBit(uint8_t bit, uint8_t sta);

/**
 * @brief 读取 PCF8574 某个 IO 的状态
 * @param bit 要读取的 IO 编号 (0~7)
 * @param bit_val 读取到的状态值 (0: 低电平, 1: 高电平)
 * @return IIC_Status 操作状态
 */
IIC_Status PCF8574_ReadBit(uint8_t bit, uint8_t *bit_val);

/**
 * @brief 读取 PCF8574 的 8 位 IO 状态
 * @param val 读取到的 8 位状态值
 * @return 0: 成功, 1: 失败
 */
uint8_t PCF8574_ReadByte(uint8_t *val);

/**
 * @brief 设置 PCF8574 的 8 位 IO 状态
 * @param data 要写入的 8 位数据
 * @return IIC_Status 操作状态
 */
IIC_Status PCF8574_WriteByte(uint8_t data);

#endif /* __PCF8574_H */
