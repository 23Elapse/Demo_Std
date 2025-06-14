/**
 * =====================================================================================
 * @file        dev_config.c
 * @brief       定义和初始化项目中所有硬件设备实例
 * @author      23Elapse & Gemini
 * @version     1.1
 * @date        2025-06-08
 * =====================================================================================
 */

#include "dev_config.h"
#include "stm32f4xx.h" // 需要外设基地址和GPIO定义
#include "pch.h"
#include "spi_flash.h"
/*
 * =====================================================================================
 * 设备实例定义
 * =====================================================================================
 */

// --- 串口设备定义 ---
Serial_Device_t g_uart_dev = {
    .instance = USART1, .tx_port = GPIOA, .tx_pin = GPIO_Pin_9,
    .rx_port = GPIOA, .rx_pin = GPIO_Pin_10, .de_port = NULL, .de_pin = 0,
    .baudrate = 115200, .af = GPIO_AF_USART1, .irqn = USART1_IRQn,
    .mode = UART_MODE
};

Serial_Device_t g_rs485_serial = { 
    .instance = USART2, .tx_port = GPIOA, .tx_pin = GPIO_Pin_2,
    .rx_port = GPIOA, .rx_pin = GPIO_Pin_3, .de_port = GPIOA, .de_pin = GPIO_Pin_8,
    .baudrate = 9600, .af = GPIO_AF_USART2, .irqn = USART2_IRQn,
    .mode = RS485_MODE
};

Serial_Device_t g_esp32_serial = {
    .instance = USART6, .tx_port = GPIOC, .tx_pin = GPIO_Pin_6,
    .rx_port = GPIOC, .rx_pin = GPIO_Pin_7, .de_port = NULL, .de_pin = 0,
    .baudrate = 115200, .af = GPIO_AF_USART6, .irqn = USART6_IRQn,
    .mode = UART_MODE
};

// --- CAN 设备定义 ---
CAN_Device_t g_can1_dev = {
    .instance = CAN1, .tx_port = GPIOA, .tx_pin = GPIO_Pin_12,
    .rx_port = GPIOA, .rx_pin = GPIO_Pin_11, .baudrate = 500000,
    .af = GPIO_AF_CAN1, .irqn = CAN1_RX0_IRQn
};

// --- I2C 总线定义 ---
I2C_Bus_t g_i2c1_bus = {
    .scl_port = GPIOH, .scl_pin = GPIO_Pin_4,
    .sda_port = GPIOH, .sda_pin = GPIO_Pin_5,
    .mutex = NULL // Mutex 将在驱动初始化时创建
};

// --- SPI Flash 设备定义 ---
// 假设 flash_config 在别处定义，如果它也是静态配置，也应移到此处
//extern SPI_Flash_Config_t flash_config;
SPI_Flash_Device_t g_spi_flash_dev = {
    .config = &flash_config,
    .id = 0
};

// --- 复合设备定义 ---
ESP32_Shared_Device_t g_esp32_dev = {
    .serial_dev = &g_esp32_serial,
    .mutex = NULL,
    .reset_port = GPIOA,
    .reset_pin = GPIO_Pin_4
};
