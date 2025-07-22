/**
 * =====================================================================================
 * @file        dev_config.c
 * @brief       定义和初始化项目中所有硬件设备实例
 * @author      23Elapse & Gemini
 * @version     1.2 (Refactored)
 * @date        2025-06-14
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
    .instance = CAN1, 
    .tx_port = GPIOA, .tx_pin = GPIO_Pin_12,
    .rx_port = GPIOA, .rx_pin = GPIO_Pin_11, .baudrate = 500000,
    .af = GPIO_AF_CAN1, .irqn = CAN1_RX0_IRQn
};

// --- I2C 总线定义 ---
I2C_Bus_t g_i2c1_bus = {
    .scl_port = GPIOB, .scl_pin = GPIO_Pin_8,
    .sda_port = GPIOB, .sda_pin = GPIO_Pin_9,
    .mutex = NULL // Mutex 将在驱动初始化时创建
};

// --- SPI Flash 配置定义 ---
// SPI Flash 的硬件配置应在此处定义
SPI_Flash_Config_t g_flash_config = {
    .SPIx = SPI1,
    .GPIO_Port = GPIOB,
    .SPI_Clk = RCC_APB2Periph_SPI1,
    .GPIO_Clk = RCC_AHB1Periph_GPIOB,
    .CS_Pin = GPIO_Pin_13,
    .SCK_Pin = GPIO_Pin_3,
    .MISO_Pin = GPIO_Pin_4,
    .MOSI_Pin = GPIO_Pin_5,
    .address_mode = FLASH_4BYTE_MODE,
    .mutex = NULL, // Mutex 将在驱动初始化时创建
    .hw_ops = NULL, // 硬件操作回调将由驱动内部设置默认值
    .hw_context = NULL // 硬件上下文将指向自身
};

// --- SPI Flash 设备实例定义 ---
// SPI Flash 设备实例，包含其配置
SPI_Flash_Device_t g_spi_flash_dev = {
    .config = &g_flash_config, // 指向上面定义的配置
    .chip_id = 0 // ID 将在初始化时读取
};

// --- 复合设备定义 ---
ESP32_Shared_Device_t g_esp32_dev = {
    .serial_dev = &g_esp32_serial,
    .mutex = NULL,
    .reset_port = GPIOA,
    .reset_pin = GPIO_Pin_4
};
