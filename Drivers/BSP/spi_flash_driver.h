/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-05-03 15:00:00
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 15:00:00
 * @FilePath: \Demo\Drivers\BSP\Inc\spi_flash_driver.h
 * @Description: SPI Flash 驱动接口头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __SPI_FLASH_DRIVER_H
#define __SPI_FLASH_DRIVER_H

#include "spi_flash.h"
#include "stm32f4xx.h"

/**
 * @brief SPI Flash 设备结构体
 */
typedef struct
{
    SPI_Flash_Config *config; // SPI Flash 配置
    uint16_t id;              // 设备 ID
} SPI_Flash_Device_t;

#endif /* __SPI_FLASH_DRIVER_H */
