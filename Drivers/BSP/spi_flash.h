/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-19 00:03:34
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 16:13:22
 * @FilePath: \Demo\Drivers\BSP\Inc\spi_flash.h
 * @Description: SPI Flash 驱动头文件，支持 RTOS 抽象和优化
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#include <stdint.h>
#include "stm32f4xx.h"
#include "rtos_abstraction.h"
#include "log_system.h"

#define PAGE_SIZE (256U)
#define SECTOR_SIZE (4U * 1024U)
#define BLOCK_SIZE (64U * 1024U)
#define FLASH_TIMEOUT_CNT (1000000U)

typedef enum
{
    FLASH_OK = 0,
    FLASH_READ_ERROR,
    FLASH_WRITE_DISABLED,
    FLASH_TIMEOUT,
    FLASH_VERIFY_ERROR,
    FLASH_WRITE_PROTECTED,
    FLASH_BUSYING,
    FLASH_INVALID_PARAM,
    FLASH_UNSUPPORTED_CHIP,
    FLASH_SPI_HW_ERROR,
    FLASH_DMA_ERROR
} Flash_Status;

#define STATUS_REG_BUSY (0x01U)

typedef enum
{
    FLASH_WriteEnable = 0x06,
    FLASH_WriteDisable = 0x04,
    FLASH_ReadStatusReg1 = 0x05,
    FLASH_ReadStatusReg2 = 0x35,
    FLASH_ReadStatusReg3 = 0x15,
    FLASH_WriteStatusReg1 = 0x01,
    FLASH_WriteStatusReg2 = 0x31,
    FLASH_WriteStatusReg3 = 0x11,
    FLASH_ReadData = 0x03,
    FLASH_FastReadData = 0x0B,
    FLASH_FastReadDual = 0x3B,
    FLASH_FastReadQuad = 0x6B,
    FLASH_PageProgram = 0x02,
    FLASH_PageProgramQuad = 0x32,
    FLASH_BlockErase = 0xD8,
    FLASH_SectorErase = 0x20,
    FLASH_ChipErase = 0xC7,
    FLASH_PowerDown = 0xB9,
    FLASH_ReleasePowerDown = 0xAB,
    FLASH_DeviceID = 0xAB,
    FLASH_ManufactDeviceID = 0x90,
    FLASH_JedecDeviceID = 0x9F,
    FLASH_Enable4ByteAddr = 0xB7,
    FLASH_Exit4ByteAddr = 0xE9,
    FLASH_SetReadParam = 0xC0,
    FLASH_EnterQIPMode = 0x38,
    FLASH_ExitQIPMode = 0xFF
} SPI_Flash_Cmd_t;

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

typedef enum
{
    FLASH_StatusReg1 = 0,
    FLASH_StatusReg2,
    FLASH_StatusReg3
} Flash_StatusReg;

typedef enum
{
    FLASH_3BYTE_MODE,
    FLASH_4BYTE_MODE
} Flash_AddressMode;

/**
 * @brief SPI 硬件操作回调接口
 */
typedef struct
{
    uint8_t (*transfer)(void *hw_context, uint8_t data);                                            // SPI 单字节传输
    void (*cs_low)(void *hw_context);                                                               // 片选拉低
    void (*cs_high)(void *hw_context);                                                              // 片选拉高
    Flash_Status (*dma_transfer)(void *hw_context, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len); // DMA 传输
} SPI_Hw_Ops_t;

typedef struct
{
    SPI_TypeDef *SPIx;              // SPI 外设实例
    GPIO_TypeDef *GPIO_Port;        // GPIO 端口
    uint32_t SPI_Clk;               // SPI 时钟
    uint32_t GPIO_Clk;              // GPIO 时钟
    uint16_t CS_Pin;                // 片选引脚
    uint16_t SCK_Pin;               // 时钟引脚
    uint16_t MISO_Pin;              // MISO 引脚
    uint16_t MOSI_Pin;              // MOSI 引脚
    Flash_AddressMode address_mode; // 地址模式
    void *mutex;                    // 互斥锁
    SPI_Hw_Ops_t *hw_ops;           // 硬件操作回调
    void *hw_context;               // 硬件上下文（传递给回调）
} SPI_Flash_Config;

extern const uint8_t g_flash_write_reg_cmd[];
extern const uint8_t g_flash_read_reg_cmd[];
extern SPI_Flash_Config flash_config;

/**
 * @brief 初始化 SPI Flash 硬件接口
 * @param config Flash 配置结构体指针
 * @return Flash_Status 操作状态
 */
Flash_Status SPI_Flash_Init(SPI_Flash_Config *config);

/**
 * @brief 初始化 W25Qxx 系列 Flash
 * @param config Flash 配置结构体指针
 * @return Flash_Status 操作状态
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
 * @return Flash_Status 操作状态
 */
Flash_Status SPI_Flash_WriteStatusReg(SPI_Flash_Config *config, Flash_StatusReg reg, uint8_t sr);

/**
 * @brief 擦除 Flash 扇区
 * @param config Flash 配置结构体指针
 * @param sectorAddr 扇区地址（字节地址）
 * @return Flash_Status 操作状态
 */
Flash_Status SPI_Flash_EraseSector(SPI_Flash_Config *config, uint32_t sectorAddr);

/**
 * @brief 擦除整个 Flash 芯片
 * @param config Flash 配置结构体指针
 * @return Flash_Status 操作状态
 */
Flash_Status SPI_Flash_EraseChip(SPI_Flash_Config *config);

/**
 * @brief 写入 Flash 页面
 * @param config Flash 配置结构体指针
 * @param pBuffer 数据缓冲区
 * @param writeAddr 写入地址
 * @param numByteToWrite 写入字节数
 * @return Flash_Status 操作状态
 */
Flash_Status SPI_Flash_WritePage(SPI_Flash_Config *config, uint8_t *pBuffer, uint32_t writeAddr, uint16_t numByteToWrite);

/**
 * @brief 读取 Flash 数据
 * @param config Flash 配置结构体指针
 * @param pBuffer 数据缓冲区
 * @param readAddr 读取地址
 * @param numByteToRead 读取字节数
 * @return Flash_Status 操作状态
 */
Flash_Status SPI_Flash_ReadData(SPI_Flash_Config *config, uint8_t *pBuffer, uint32_t readAddr, uint16_t numByteToRead);

/**
 * @brief 快速读取 Flash 数据
 * @param config Flash 配置结构体指针
 * @param pBuffer 数据缓冲区
 * @param readAddr 读取地址
 * @param numByteToRead 读取字节数
 * @return Flash_Status 操作状态
 */
Flash_Status SPI_Flash_FastReadData(SPI_Flash_Config *config, uint8_t *pBuffer, uint32_t readAddr, uint16_t numByteToRead);

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
