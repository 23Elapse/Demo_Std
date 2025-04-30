/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-19 00:03:34
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-30 09:05:08
 * @FilePath: \Demo\Drivers\BSP\Inc\spi_flash.h
 * @Description: SPI Flash 驱动头文件，支持 RTOS 抽象
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */

#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#include <stdint.h>
#include "stm32f4xx.h"
#include "rtos_abstraction.h"

/* Flash 参数定义 */
#define PAGE_SIZE (256U)             // 页面大小（256字节）
#define SECTOR_SIZE (4U * 1024U)     // 扇区大小（4KB）
#define BLOCK_SIZE (64U * 1024U)     // 块大小（64KB）
#define FLASH_TIMEOUT_CNT (1000000U) // 超时计数器（循环次数）

/**
 * @brief Flash 操作状态枚举
 */
typedef enum
{
    FLASH_OK = 0,          // 操作成功
    FLASH_READ_ERROR,      // 状态寄存器读取失败
    FLASH_WRITE_DISABLED,  // 写使能失败
    FLASH_TIMEOUT,         // 操作超时
    FLASH_VERIFY_ERROR,    // 数据验证失败
    FLASH_WRITE_PROTECTED, // 写保护
    FLASH_BUSYING,         // Flash 忙碌
    FLASH_INVALID_PARAM,   // 参数无效
    FLASH_UNSUPPORTED_CHIP // 不支持的芯片型号
} Flash_Status;

/**
 * @brief Flash 状态寄存器忙碌标志
 */
#define STATUS_REG_BUSY (0x01U)

/**
 * @brief SPI Flash 指令集
 */
typedef enum
{
    FLASH_WriteEnable = 0x06,      // 写使能
    FLASH_WriteDisable = 0x04,     // 写禁止
    FLASH_ReadStatusReg1 = 0x05,   // 读状态寄存器1
    FLASH_ReadStatusReg2 = 0x35,   // 读状态寄存器2
    FLASH_ReadStatusReg3 = 0x15,   // 读状态寄存器3
    FLASH_WriteStatusReg1 = 0x01,  // 写状态寄存器1
    FLASH_WriteStatusReg2 = 0x31,  // 写状态寄存器2
    FLASH_WriteStatusReg3 = 0x11,  // 写状态寄存器3
    FLASH_ReadData = 0x03,         // 读数据
    FLASH_FastReadData = 0x0B,     // 快速读数据
    FLASH_FastReadDual = 0x3B,     // 双通道快速读
    FLASH_FastReadQuad = 0x6B,     // 四通道快速读
    FLASH_PageProgram = 0x02,      // 页面编程
    FLASH_PageProgramQuad = 0x32,  // 四通道页面编程
    FLASH_BlockErase = 0xD8,       // 块擦除
    FLASH_SectorErase = 0x20,      // 扇区擦除
    FLASH_ChipErase = 0xC7,        // 芯片擦除
    FLASH_PowerDown = 0xB9,        // 进入低功耗模式
    FLASH_ReleasePowerDown = 0xAB, // 退出低功耗模式
    FLASH_DeviceID = 0xAB,         // 读设备ID
    FLASH_ManufactDeviceID = 0x90, // 读制造商和设备ID
    FLASH_JedecDeviceID = 0x9F,    // 读JEDEC ID
    FLASH_Enable4ByteAddr = 0xB7,  // 使能4字节地址模式
    FLASH_Exit4ByteAddr = 0xE9,    // 退出4字节地址模式
    FLASH_SetReadParam = 0xC0,     // 设置读参数
    FLASH_EnterQIPMode = 0x38,     // 进入QPI模式
    FLASH_ExitQIPMode = 0xFF       // 退出QPI模式
} SPI_Flash_Cmd_t;

/**
 * @brief 支持的 Flash 芯片型号
 */
typedef enum
{
    W25Q80 = 0xEF13,   // Winbond W25Q80
    W25Q16 = 0xEF14,   // Winbond W25Q16
    W25Q32 = 0xEF15,   // Winbond W25Q32
    W25Q64 = 0xEF16,   // Winbond W25Q64
    W25Q128 = 0xEF17,  // Winbond W25Q128
    W25Q256 = 0xEF18,  // Winbond W25Q256
    BY25Q64 = 0x6816,  // Boya BY25Q64
    BY25Q128 = 0x6817, // Boya BY25Q128
    BY25Q256 = 0x6818, // Boya BY25Q256
    NM25Q64 = 0x5216,  // Nanya NM25Q64
    NM25Q128 = 0x5217  // Nanya NM25Q128
} SPI_Flash_ChipID_t;

/**
 * @brief Flash 状态寄存器编号
 */
typedef enum
{
    FLASH_StatusReg1 = 0, // 状态寄存器1
    FLASH_StatusReg2,     // 状态寄存器2
    FLASH_StatusReg3      // 状态寄存器3
} Flash_StatusReg;

/**
 * @brief Flash 地址模式
 */
typedef enum
{
    FLASH_3BYTE_MODE, // 3字节地址模式
    FLASH_4BYTE_MODE  // 4字节地址模式
} Flash_AddressMode;

/**
 * @brief SPI Flash 配置结构体
 */
typedef struct
{
    SPI_TypeDef *SPIx;              // SPI 外设
    GPIO_TypeDef *GPIO_Port;        // GPIO 端口
    uint32_t SPI_Clk;               // SPI 时钟
    uint32_t GPIO_Clk;              // GPIO 时钟
    uint16_t CS_Pin;                // 片选引脚
    uint16_t SCK_Pin;               // 时钟引脚
    uint16_t MISO_Pin;              // MISO 引脚
    uint16_t MOSI_Pin;              // MOSI 引脚
    Flash_AddressMode address_mode; // 地址模式
} SPI_Flash_Config;

/* 全局命令码数组 */
extern const uint8_t g_flash_write_reg_cmd[]; // 写状态寄存器命令
extern const uint8_t g_flash_read_reg_cmd[];  // 读状态寄存器命令

/* 全局配置变量 */
extern SPI_Flash_Config flash_cfg;
extern SPI_Flash_Config flash_config;
/* 函数声明 */

/**
 * @brief 初始化 SPI Flash 硬件接口
 * @param config Flash 配置结构体指针
 * @return Flash 操作状态
 */
Flash_Status SPI_Flash_Init(SPI_Flash_Config *config);

/**
 * @brief 初始化 W25Qxx 系列 Flash
 * @param config Flash 配置结构体指针
 * @return Flash 操作状态
 */
Flash_Status W25Qxx_Init(SPI_Flash_Config *config);

/**
 * @brief SPI 数据传输
 * @param config Flash 配置结构体指针
 * @param data 要发送的数据
 * @return 接收到的数据
 */
uint8_t SPI_Flash_Transfer(SPI_Flash_Config *config, uint8_t data);

/**
 * @brief 读取 Flash 状态寄存器
 * @param config Flash 配置结构体指针
 * @param reg 状态寄存器编号
 * @return 寄存器值
 */
uint8_t SPI_Flash_ReadStatusReg(SPI_Flash_Config *config, Flash_StatusReg reg);

/**
 * @brief 写入 Flash 状态寄存器
 * @param config Flash 配置结构体指针
 * @param reg 状态寄存器编号
 * @param sr 要写入的值
 * @return Flash 操作状态
 */
Flash_Status SPI_Flash_WriteStatusReg(SPI_Flash_Config *config, Flash_StatusReg reg, uint8_t sr);

/**
 * @brief 擦除 Flash 扇区
 * @param config Flash 配置结构体指针
 * @param sectorAddr 扇区地址（字节地址）
 * @return Flash 操作状态
 */
Flash_Status SPI_Flash_EraseSector(SPI_Flash_Config *config, uint32_t sectorAddr);

/**
 * @brief 擦除整个 Flash 芯片
 * @param config Flash 配置结构体指针
 * @return Flash 操作状态
 */
Flash_Status SPI_Flash_EraseChip(SPI_Flash_Config *config);

/**
 * @brief 写入 Flash 页面
 * @param config Flash 配置结构体指针
 * @param pBuffer 数据缓冲区
 * @param writeAddr 写入地址
 * @param numByteToWrite 写入字节数
 * @return Flash 操作状态
 */
Flash_Status SPI_Flash_WritePage(SPI_Flash_Config *config, uint8_t *pBuffer, uint32_t writeAddr, uint16_t numByteToWrite);

/**
 * @brief 读取 Flash 数据
 * @param config Flash 配置结构体指针
 * @param pBuffer 数据缓冲区
 * @param readAddr 读取地址
 * @param numByteToRead 读取字节数
 * @return Flash 操作状态
 */
Flash_Status SPI_Flash_ReadData(SPI_Flash_Config *config, uint8_t *pBuffer, uint32_t readAddr, uint16_t numByteToRead);

/**
 * @brief 读取 Flash 设备 ID
 * @param config Flash 配置结构体指针
 * @return 设备 ID
 */
uint16_t Flash_Read_Id(SPI_Flash_Config *config);

/**
 * @brief 带擦除的 Flash 写入操作
 * @param config Flash 配置结构体指针
 * @param pbuf 数据缓冲区
 * @param addr 写入地址
 * @param datalen 数据长度
 * @return 0: 成功; -1: 失败
 */
int8_t SPI_Flash_Write_With_Erase(SPI_Flash_Config *config, uint8_t *pbuf, uint32_t addr, uint16_t datalen);
#endif /* __SPI_FLASH_H */
