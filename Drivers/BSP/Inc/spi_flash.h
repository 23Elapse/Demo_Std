/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-19 00:03:34
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-29 09:59:35
 * @FilePath: \Demo\Drivers\BSP\Inc\spi_flash.h
 * @Description: 
 * 
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved. 
 */

#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

/* FLASH参数定义 */
#define PAGE_SIZE             (256U)         // 页大小（256字节）
#define SECTOR_SIZE           (4U * 1024U)   // 扇区大小（4KB）
#define BLOCK_SIZE            (64U * 1024U)  // 块大小（64KB）
/* 超时时间定义（单位循环次数，可调） */
#define FLASH_TIMEOUT_CNT     (1000000U)

/* FLASH状态码 */
typedef enum {
    FLASH_OK = 0,
    FLASH_READ_ERROR,      // 状态寄存器读取失败
    FLASH_WRITE_DISABLED,  // 写使能失败
    FLASH_TIMEOUT,
    FLASH_VERIFY_ERROR,
    FLASH_WRITE_PROTECTED,
    FLASH_BUSYING,
    FLASH_INVALID_PARAM,
    FLASH_UNSUPPORTED_CHIP
} Flash_Status;

/* 状态寄存器宏 */
#define STATUS_REG_BUSY       (0x01U)
/* SPI指令枚举 */
typedef enum {
    FLASH_WriteEnable       = 0x06,
    FLASH_WriteDisable      = 0x04,
    FLASH_ReadStatusReg1    = 0x05,
    FLASH_ReadStatusReg2    = 0x35,
    FLASH_ReadStatusReg3    = 0x15,
    FLASH_WriteStatusReg1   = 0x01,
    FLASH_WriteStatusReg2   = 0x31,
    FLASH_WriteStatusReg3   = 0x11,
    FLASH_ReadData          = 0x03,
    FLASH_FastReadData      = 0x0B,
    FLASH_FastReadDual      = 0x3B,
    FLASH_FastReadQuad      = 0x6B,
    FLASH_PageProgram       = 0x02,
    FLASH_PageProgramQuad   = 0x32,
    FLASH_BlockErase        = 0xD8,
    FLASH_SectorErase       = 0x20,
    FLASH_ChipErase         = 0xC7,
    FLASH_PowerDown         = 0xB9,
    FLASH_ReleasePowerDown  = 0xAB,
    FLASH_DeviceID          = 0xAB,
    FLASH_ManufactDeviceID  = 0x90,
    FLASH_JedecDeviceID     = 0x9F,
    FLASH_Enable4ByteAddr   = 0xB7,
    FLASH_Exit4ByteAddr     = 0xE9,
    FLASH_SetReadParam      = 0xC0,
    FLASH_EnterQIPMode      = 0x38,
    FLASH_ExitQIPMode       = 0xFF
} SPI_Flash_Cmd_t;
/* 芯片型号枚举 */
typedef enum {
    W25Q80   = 0xEF13,  // W25Q80 芯片 ID
    W25Q16   = 0xEF14,  // W25Q16 芯片 ID
    W25Q32   = 0xEF15,  // W25Q32 芯片 ID
    W25Q64   = 0xEF16,  // W25Q64 芯片 ID
    W25Q128  = 0xEF17,  // W25Q128 芯片 ID
    W25Q256  = 0xEF18,  // W25Q256 芯片 ID
    BY25Q64  = 0x6816,  // BY25Q64 芯片 ID
    BY25Q128 = 0x6817,  // BY25Q128 芯片 ID
    BY25Q256 = 0x6818,  // BY25Q256 芯片 ID
    NM25Q64  = 0x5216,  // NM25Q64 芯片 ID
    NM25Q128 = 0x5217   // NM25Q128 芯片 ID
} SPI_Flash_ChipID_t;

typedef enum {
    FLASH_StatusReg1 = 0,  // 状态寄存器1
    FLASH_StatusReg2,       // 状态寄存器2
    FLASH_StatusReg3        // 状态寄存器3
} Flash_StatusReg;

/* 命令码全局常量数组（与枚举顺序严格对应） */
static const uint8_t g_flash_write_reg_cmd[] = {
    FLASH_WriteStatusReg1,  // 对应FLASH_StatusReg1
    FLASH_WriteStatusReg2,   // 对应FLASH_StatusReg2
    FLASH_WriteStatusReg3    // 对应FLASH_StatusReg3
};
static const uint8_t g_flash_read_reg_cmd[] = {
    FLASH_ReadStatusReg1,
    FLASH_ReadStatusReg2,
    FLASH_ReadStatusReg3
};
typedef enum {
    FLASH_3BYTE_MODE,
    FLASH_4BYTE_MODE
} Flash_AddressMode;
/* SPI Flash配置结构体 */
typedef struct {
    SPI_TypeDef* SPIx;
    GPIO_TypeDef* GPIO_Port;
    uint32_t SPI_Clk;
    uint32_t GPIO_Clk;
    uint16_t CS_Pin;
    uint16_t SCK_Pin;
    uint16_t MISO_Pin;
    uint16_t MOSI_Pin;
    Flash_AddressMode address_mode; // 新增地址模式字段
} SPI_Flash_Config;

/* 初始化与基础函数 */
uint8_t SPI_Flash_Transfer(SPI_Flash_Config* config, uint8_t data);
static void CS_Low(SPI_Flash_Config* config);
static void CS_High(SPI_Flash_Config* config);
static void SendAddress(SPI_Flash_Config* config, uint32_t addr);
static Flash_Status WriteEnable(SPI_Flash_Config* config);

static Flash_Status CheckBusy(SPI_Flash_Config* config);

void SPI_Flash_Init(SPI_Flash_Config* config);

Flash_Status SPI_Flash_EraseSector(SPI_Flash_Config* config, uint32_t sectorAddr);
Flash_Status SPI_Flash_EraseChip(SPI_Flash_Config* config);
Flash_Status SPI_Flash_WritePage(SPI_Flash_Config* config, uint8_t* pBuffer, uint32_t writeAddr, uint16_t numByteToWrite);
Flash_Status SPI_Flash_ReadData(SPI_Flash_Config* config, uint8_t* pBuffer, uint32_t readAddr, uint16_t numByteToRead);
uint16_t Flash_Read_Id(SPI_Flash_Config* config);
int8_t SPI_Flash_Write_With_Erase(SPI_Flash_Config* config, uint8_t *pbuf, uint32_t addr, uint16_t datalen);
Flash_Status SPI_Flash_WriteStatusReg(SPI_Flash_Config* config, Flash_StatusReg reg, uint8_t sr);
uint8_t SPI_Flash_ReadStatusReg(SPI_Flash_Config* config, Flash_StatusReg reg);

extern SPI_Flash_Config flash_cfg;






#endif
