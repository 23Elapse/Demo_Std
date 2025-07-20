/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-19 00:03:34
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-14 18:00:00
 * @FilePath: \Demo\Drivers\BSP\Inc\spi_flash.h
 * @Description: SPI Flash 驱动头文件，支持 RTOS 抽象和优化 (Refactored)
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#include <stdint.h>
#include "stm32f4xx.h"
#include "rtos_abstraction.h"
#include "log_system.h"

// Flash 存储单元大小定义
#define PAGE_SIZE (256U)       // 页面大小
#define SECTOR_SIZE (4U * 1024U) // 扇区大小 (4KB)
#define BLOCK_SIZE (64U * 1024U) // 块大小 (64KB)
#define FLASH_TIMEOUT_CNT (1000000U) // 通用超时计数

/**
 * @brief Flash 操作状态枚举
 */
typedef enum
{
    FLASH_OK = 0,
    FLASH_READ_ERROR,
    FLASH_WRITE_DISABLED,
    FLASH_TIMEOUT,
    FLASH_VERIFY_ERROR,
    FLASH_WRITE_PROTECTED,
    FLASH_BUSYING,        // Flash 正在忙碌
    FLASH_INVALID_PARAM,  // 无效参数
    FLASH_UNSUPPORTED_CHIP, // 不支持的芯片类型
    FLASH_SPI_HW_ERROR,   // SPI 硬件错误
    FLASH_DMA_ERROR,      // DMA 传输错误
    FLASH_MUTEX_TIMEOUT   // 获取互斥锁超时
} Flash_Status_t;

// 状态寄存器位定义
#define STATUS_REG_BUSY (0x01U) // 忙碌位

/**
 * @brief SPI Flash 命令集
 */
typedef enum
{
    FLASH_WriteEnable = 0x06,         // 写使能
    FLASH_WriteDisable = 0x04,        // 写禁用
    FLASH_ReadStatusReg1 = 0x05,      // 读取状态寄存器1
    FLASH_ReadStatusReg2 = 0x35,      // 读取状态寄存器2
    FLASH_ReadStatusReg3 = 0x15,      // 读取状态寄存器3
    FLASH_WriteStatusReg1 = 0x01,     // 写入状态寄存器1
    FLASH_WriteStatusReg2 = 0x31,     // 写入状态寄存器2
    FLASH_WriteStatusReg3 = 0x11,     // 写入状态寄存器3
    FLASH_ReadData = 0x03,            // 读取数据
    FLASH_FastReadData = 0x0B,        // 快速读取数据
    FLASH_FastReadDual = 0x3B,        // 双SPI快速读取
    FLASH_FastReadQuad = 0x6B,        // 四SPI快速读取
    FLASH_PageProgram = 0x02,         // 页编程
    FLASH_PageProgramQuad = 0x32,     // 四SPI页编程
    FLASH_BlockErase = 0xD8,          // 块擦除 (64KB)
    FLASH_SectorErase = 0x20,         // 扇区擦除 (4KB)
    FLASH_ChipErase = 0xC7,           // 整片擦除
    FLASH_PowerDown = 0xB9,           // 进入掉电模式
    FLASH_ReleasePowerDown = 0xAB,    // 退出掉电模式
    FLASH_DeviceID = 0xAB,            // 读取设备ID (另一种方式)
    FLASH_ManufactDeviceID = 0x90,    // 读取制造商/设备ID
    FLASH_JedecDeviceID = 0x9F,       // 读取JEDEC ID
    FLASH_Enable4ByteAddr = 0xB7,     // 进入4字节地址模式
    FLASH_Exit4ByteAddr = 0xE9,       // 退出4字节地址模式
    FLASH_SetReadParam = 0xC0,        // 设置读取参数
    FLASH_EnterQIPMode = 0x38,        // 进入QPI模式
    FLASH_ExitQIPMode = 0xFF          // 退出QPI模式
} SPI_Flash_Cmd_t;

/**
 * @brief SPI Flash 芯片ID枚举
 */
typedef enum
{
    W25Q80 = 0xEF13,
    W25Q16 = 0xEF14,
    W25Q32 = 0xEF15,
    W25Q64 = 0xEF16,
    W25Q128 = 0xEF17,
    W25Q256 = 0xEF18,
    W25Q512 = 0xEF19,
    BY25Q64 = 0x6816,
    BY25Q128 = 0x6817,
    BY25Q256 = 0x6818,
    NM25Q64 = 0x5216,
    NM25Q128 = 0x5217
} SPI_Flash_ChipID_t;

/**
 * @brief Flash 状态寄存器编号
 */
typedef enum
{
    FLASH_StatusReg1 = 0,
    FLASH_StatusReg2,
    FLASH_StatusReg3
} Flash_StatusReg_t;

/**
 * @brief Flash 地址模式
 */
typedef enum
{
    FLASH_3BYTE_MODE,
    FLASH_4BYTE_MODE
} Flash_AddressMode_t;

/**
 * @brief SPI 硬件操作回调接口
 * @note  这是一个抽象层，允许SPI Flash驱动兼容不同SPI硬件实现
 */
typedef struct
{
    // 单字节传输回调函数
    // @param hw_context: 硬件上下文指针，通常指向SPI_Flash_Config_t
    // @param data: 要发送的字节
    // @return: 接收到的字节
    uint8_t (*transfer)(void *hw_context, uint8_t data);

    // 片选拉低回调函数
    // @param hw_context: 硬件上下文指针
    void (*cs_low)(void *hw_context);

    // 片选拉高回调函数
    // @param hw_context: 硬件上下文指针
    void (*cs_high)(void *hw_context);

    // DMA 传输回调函数 (可选，如果硬件支持DMA且需要高性能)
    // @param hw_context: 硬件上下文指针
    // @param tx_buf: 发送数据缓冲区 (如果为NULL表示只接收)
    // @param rx_buf: 接收数据缓冲区 (如果为NULL表示只发送)
    // @param len: 传输长度
    // @return: 传输状态
    Flash_Status_t (*dma_transfer)(void *hw_context, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len);
} SPI_Hw_Ops_t;

/**
 * @brief SPI Flash 硬件配置结构体
 * @note  包含SPI外设、GPIO、时钟等所有硬件相关配置
 */
typedef struct
{
    SPI_TypeDef *SPIx;              // SPI 外设实例 (如 SPI1, SPI2)
    GPIO_TypeDef *GPIO_Port;        // 用于CS, SCK, MISO, MOSI的GPIO端口
    uint32_t SPI_Clk;               // SPI 外设时钟 (RCC_APB1Periph_SPIx 或 RCC_APB2Periph_SPIx)
    uint32_t GPIO_Clk;              // GPIO 端口时钟 (RCC_AHB1Periph_GPIOx)
    uint16_t CS_Pin;                // 片选引脚
    uint16_t SCK_Pin;               // 时钟引脚
    uint16_t MISO_Pin;              // MISO 引脚
    uint16_t MOSI_Pin;              // MOSI 引脚
    Flash_AddressMode_t address_mode; // Flash 地址模式 (3字节或4字节)
    void *mutex;                    // 用于保护Flash操作的互斥锁
    SPI_Hw_Ops_t *hw_ops;           // 硬件操作回调函数集合
    void *hw_context;               // 硬件上下文指针，传递给hw_ops回调函数
} SPI_Flash_Config_t;

/**
 * @brief SPI Flash 设备句柄结构体
 * @note  包含Flash配置和运行时状态
 */
typedef struct
{
    SPI_Flash_Config_t *config; // 指向Flash硬件配置
    uint16_t chip_id;           // 读取到的芯片ID
} SPI_Flash_Device_t;

// 声明全局的Flash配置实例 (在 dev_config.c 中定义)
extern SPI_Flash_Config_t g_flash_config;

/**
 * @brief 初始化 SPI Flash 硬件接口 (仅SPI外设和GPIO)
 * @param config Flash 配置结构体指针
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_Hardware_Init(SPI_Flash_Config_t *config);

/**
 * @brief 初始化 SPI Flash 设备 (包括硬件和芯片特定初始化)
 * @param dev Flash 设备句柄指针
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_Device_Init(SPI_Flash_Device_t *dev);

/**
 * @brief SPI 单字节数据传输
 * @param config Flash 配置结构体指针
 * @param data 要发送的数据
 * @return 接收到的数据
 */
uint8_t SPI_Flash_TransferByte(SPI_Flash_Config_t *config, uint8_t data);

/**
 * @brief 读取 Flash 状态寄存器
 * @param config Flash 配置结构体指针
 * @param reg 状态寄存器编号 (Flash_StatusReg_t)
 * @return 寄存器值
 */
uint8_t SPI_Flash_ReadStatusReg(SPI_Flash_Config_t *config, Flash_StatusReg_t reg);

/**
 * @brief 写入 Flash 状态寄存器
 * @param config Flash 配置结构体指针
 * @param reg 状态寄存器编号 (Flash_StatusReg_t)
 * @param sr 要写入的值
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_WriteStatusReg(SPI_Flash_Config_t *config, Flash_StatusReg_t reg, uint8_t sr);

/**
 * @brief 擦除 Flash 扇区
 * @param dev Flash 设备句柄指针
 * @param sector_addr 扇区地址（字节地址，必须是扇区起始地址）
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_EraseSector(SPI_Flash_Device_t *dev, uint32_t sector_addr);

/**
 * @brief 擦除整个 Flash 芯片
 * @param dev Flash 设备句柄指针
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_EraseChip(SPI_Flash_Device_t *dev);

/**
 * @brief 写入 Flash 页面
 * @param dev Flash 设备句柄指针
 * @param p_buffer 数据缓冲区
 * @param write_addr 写入地址
 * @param num_byte_to_write 写入字节数 (应 <= PAGE_SIZE)
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_WritePage(SPI_Flash_Device_t *dev, const uint8_t *p_buffer, uint32_t write_addr, uint16_t num_byte_to_write);

/**
 * @brief 读取 Flash 数据
 * @param dev Flash 设备句柄指针
 * @param p_buffer 数据缓冲区
 * @param read_addr 读取地址
 * @param num_byte_to_read 读取字节数
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_ReadData(SPI_Flash_Device_t *dev, uint8_t *p_buffer, uint32_t read_addr, uint16_t num_byte_to_read);

/**
 * @brief 快速读取 Flash 数据 (带一个哑字节)
 * @param dev Flash 设备句柄指针
 * @param p_buffer 数据缓冲区
 * @param read_addr 读取地址
 * @param num_byte_to_read 读取字节数
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_FastReadData(SPI_Flash_Device_t *dev, uint8_t *p_buffer, uint32_t read_addr, uint16_t num_byte_to_read);

/**
 * @brief 读取 Flash 设备 ID (Manufacturer ID + Device ID)
 * @param config Flash 配置结构体指针
 * @return 16位设备ID
 */
uint16_t SPI_Flash_ReadChipId(SPI_Flash_Config_t *config);

/**
 * @brief 带扇区擦除的 Flash 写入操作
 * @param dev Flash 设备句柄指针
 * @param p_buffer 数据缓冲区
 * @param addr 写入起始地址
 * @param data_len 数据长度
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_WriteWithErase(SPI_Flash_Device_t *dev, const uint8_t *p_buffer, uint32_t addr, uint32_t data_len);

#endif /* __SPI_FLASH_H */
