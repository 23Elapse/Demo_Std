/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-19 00:03:34
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-25 16:39:29
 * @FilePath: \Demo\Drivers\BSP\Src\spi_flash.c
 * @Description: SPI Flash 驱动实现，支持 RTOS 抽象和优化
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "spi_flash.h"
#include <string.h>
#include "log_system.h"
#include "rtos_abstraction.h"
#include "stm32f4xx_dma.h"
#include "pch.h"

const uint8_t g_flash_write_reg_cmd[] = {
    FLASH_WriteStatusReg1,
    FLASH_WriteStatusReg2,
    FLASH_WriteStatusReg3};

const uint8_t g_flash_read_reg_cmd[] = {
    FLASH_ReadStatusReg1,
    FLASH_ReadStatusReg2,
    FLASH_ReadStatusReg3};

uint16_t g_norflash_type = BY25Q256;

static uint8_t sector_buf[SECTOR_SIZE];

SPI_Flash_Config flash_config = {
    .SPIx = SPI5,
    .GPIO_Port = GPIOF,
    .SPI_Clk = RCC_APB2Periph_SPI5,
    .GPIO_Clk = RCC_AHB1Periph_GPIOF,
    .CS_Pin = GPIO_Pin_6,
    .SCK_Pin = GPIO_Pin_7,
    .MISO_Pin = GPIO_Pin_8,
    .MOSI_Pin = GPIO_Pin_9,
    .address_mode = FLASH_4BYTE_MODE,
    .mutex = NULL,
    .hw_ops = NULL,
    .hw_context = NULL};

/**
 * @brief 默认 SPI 传输回调
 */
static uint8_t default_transfer(void *hw_context, uint8_t data)
{
    SPI_Flash_Config *config = (SPI_Flash_Config *)hw_context;
    while (SPI_I2S_GetFlagStatus(config->SPIx, SPI_I2S_FLAG_TXE) == RESET)
        ;
    SPI_I2S_SendData(config->SPIx, data);
    while (SPI_I2S_GetFlagStatus(config->SPIx, SPI_I2S_FLAG_RXNE) == RESET)
        ;
    return SPI_I2S_ReceiveData(config->SPIx);
}

/**
 * @brief 默认片选拉低回调
 */
static void default_cs_low(void *hw_context)
{
    SPI_Flash_Config *config = (SPI_Flash_Config *)hw_context;
    GPIO_ResetBits(config->GPIO_Port, config->CS_Pin);
}

/**
 * @brief 默认片选拉高回调
 */
static void default_cs_high(void *hw_context)
{
    SPI_Flash_Config *config = (SPI_Flash_Config *)hw_context;
    GPIO_SetBits(config->GPIO_Port, config->CS_Pin);
}

/**
 * @brief 默认 DMA 传输回调（占位）
 */
static Flash_Status default_dma_transfer(void *hw_context, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len)
{
    // TODO: 实现 DMA 传输
    Log_Message(LOG_LEVEL_WARNING, "[FLASH] DMA not implemented");
    return FLASH_DMA_ERROR;
}

/**
 * @brief 发送 Flash 地址
 */
static void send_address(SPI_Flash_Config *config, uint32_t addr)
{
    if (g_norflash_type == W25Q256 || g_norflash_type == BY25Q256 || g_norflash_type == W25Q512)
    {
        SPI_Flash_Transfer(config, (uint8_t)(addr >> 24));
    }
    SPI_Flash_Transfer(config, (addr >> 16) & 0xFF);
    SPI_Flash_Transfer(config, (addr >> 8) & 0xFF);
    SPI_Flash_Transfer(config, addr & 0xFF);
}

/**
 * @brief 检查 Flash 是否忙碌
 */
static Flash_Status check_busy(SPI_Flash_Config *config)
{
    uint32_t timeout = FLASH_TIMEOUT_CNT / 100; // 每次检查 1/100 的总超时
    uint8_t retry = 3;

    do
    {
        config->hw_ops->cs_low(config->hw_context);
        SPI_Flash_Transfer(config, FLASH_ReadStatusReg1);
        uint8_t status = SPI_Flash_Transfer(config, 0xFF);
        config->hw_ops->cs_high(config->hw_context);

        if (!(status & STATUS_REG_BUSY))
        {
            return FLASH_OK;
        }

        g_rtos_ops->Delay(10); // 10ms 延时，减少 CPU 占用
    } while (--retry && timeout--);

    Log_Message(LOG_LEVEL_ERROR, "[FLASH] Operation timeout");
    return FLASH_TIMEOUT;
}

/**
 * @brief 启用 Flash 写入
 */
static Flash_Status write_enable(SPI_Flash_Config *config)
{
    uint8_t retry = 3;
    do
    {
        config->hw_ops->cs_low(config->hw_context);
        SPI_Flash_Transfer(config, FLASH_WriteEnable);
        config->hw_ops->cs_high(config->hw_context);

        if (SPI_Flash_ReadStatusReg(config, FLASH_StatusReg1) & 0x02)
        {
            return FLASH_OK;
        }
        g_rtos_ops->Delay(1);
    } while (--retry);

    Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write enable failed");
    return FLASH_WRITE_PROTECTED;
}

Flash_Status SPI_Flash_Init(SPI_Flash_Config *config)
{
    if (!config || !config->SPIx || !config->GPIO_Port)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid config parameters");
        return FLASH_INVALID_PARAM;
    }

    // 设置默认硬件操作回调
    static SPI_Hw_Ops_t default_ops = {
        .transfer = default_transfer,
        .cs_low = default_cs_low,
        .cs_high = default_cs_high,
        .dma_transfer = default_dma_transfer};
    if (!config->hw_ops)
    {
        config->hw_ops = &default_ops;
        config->hw_context = config;
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    SPI_InitTypeDef SPI_InitStruct = {0};

    RCC_AHB1PeriphClockCmd(config->GPIO_Clk, ENABLE);
    if (config->SPIx == SPI2 || config->SPIx == SPI3)
    {
        RCC_APB1PeriphClockCmd(config->SPI_Clk, ENABLE);
    }
    else
    {
        RCC_APB2PeriphClockCmd(config->SPI_Clk, ENABLE);
    }

    GPIO_InitStruct.GPIO_Pin = config->CS_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(config->GPIO_Port, &GPIO_InitStruct);
    GPIO_SetBits(config->GPIO_Port, config->CS_Pin);

    GPIO_PinAFConfig(config->GPIO_Port, GPIO_PinSource7, GPIO_AF_SPI5);
    GPIO_PinAFConfig(config->GPIO_Port, GPIO_PinSource8, GPIO_AF_SPI5);
    GPIO_PinAFConfig(config->GPIO_Port, GPIO_PinSource9, GPIO_AF_SPI5);

    GPIO_InitStruct.GPIO_Pin = config->SCK_Pin | config->MISO_Pin | config->MOSI_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(config->GPIO_Port, &GPIO_InitStruct);

    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    SPI_Init(config->SPIx, &SPI_InitStruct);
    SPI_Cmd(config->SPIx, ENABLE);

    Log_Message(LOG_LEVEL_INFO, "[FLASH] SPI hardware initialized");
    return FLASH_OK;
}

Flash_Status W25Qxx_Init(SPI_Flash_Config *config)
{
    if (!config->mutex)
    {
        config->mutex = g_rtos_ops->SemaphoreCreate();
        if (!config->mutex)
        {
            Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to create mutex");
            return FLASH_INVALID_PARAM;
        }
    }

    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to take mutex");
        return FLASH_TIMEOUT;
    }

    Flash_Status status = SPI_Flash_Init(config);
    if (status != FLASH_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] SPI init failed: %d", status);
        g_rtos_ops->SemaphoreGive(config->mutex);
        return status;
    }

    g_norflash_type = Flash_Read_Id(config);
    Log_Message(LOG_LEVEL_INFO, "[FLASH] Flash ID: 0x%04X", g_norflash_type);

    if (g_norflash_type == W25Q256 || g_norflash_type == BY25Q256 || g_norflash_type == W25Q512)
    {
        uint8_t temp = SPI_Flash_ReadStatusReg(config, FLASH_StatusReg3);
        Log_Message(LOG_LEVEL_INFO, "[FLASH] Status Register 3: 0x%02X", temp);
        if ((temp & 0x01) == 0)
        {
            status = write_enable(config);
            if (status != FLASH_OK)
            {
                Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write enable failed: %d", status);
                g_rtos_ops->SemaphoreGive(config->mutex);
                return status;
            }
            temp |= 1 << 1;
            status = SPI_Flash_WriteStatusReg(config, FLASH_StatusReg3, temp);
            if (status != FLASH_OK)
            {
                Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write status reg failed: %d", status);
                g_rtos_ops->SemaphoreGive(config->mutex);
                return status;
            }
            g_rtos_ops->Delay(20);
            config->hw_ops->cs_low(config->hw_context);
            SPI_Flash_Transfer(config, FLASH_Enable4ByteAddr);
            config->hw_ops->cs_high(config->hw_context);
        }
    }

    Log_Message(LOG_LEVEL_INFO, "[FLASH] SPI Flash initialized successfully");
    g_rtos_ops->SemaphoreGive(config->mutex);
    return FLASH_OK;
}

uint8_t SPI_Flash_Transfer(SPI_Flash_Config *config, uint8_t data)
{
    if (!config || !config->hw_ops)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid config or hw_ops");
        return 0;
    }
    return config->hw_ops->transfer(config->hw_context, data);
}

uint8_t SPI_Flash_ReadStatusReg(SPI_Flash_Config *config, Flash_StatusReg reg)
{
    if (!config || reg > FLASH_StatusReg3)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid config or register");
        return 0;
    }

    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_Transfer(config, g_flash_read_reg_cmd[reg]);
    uint8_t status = SPI_Flash_Transfer(config, 0xFF);
    config->hw_ops->cs_high(config->hw_context);
    return status;
}

Flash_Status SPI_Flash_WriteStatusReg(SPI_Flash_Config *config, Flash_StatusReg reg, uint8_t sr)
{
    if (!config || reg > FLASH_StatusReg3)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid config or register");
        return FLASH_INVALID_PARAM;
    }

    Flash_Status status = write_enable(config);
    if (status != FLASH_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write enable failed: %d", status);
        return status;
    }

    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_Transfer(config, g_flash_write_reg_cmd[reg]);
    SPI_Flash_Transfer(config, sr);
    config->hw_ops->cs_high(config->hw_context);
    return check_busy(config);
}

Flash_Status SPI_Flash_EraseSector(SPI_Flash_Config *config, uint32_t sectorAddr)
{
    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to take mutex");
        return FLASH_TIMEOUT;
    }

    Flash_Status status = write_enable(config);
    if (status != FLASH_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write enable failed: %d", status);
        g_rtos_ops->SemaphoreGive(config->mutex);
        return status;
    }

    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_Transfer(config, FLASH_SectorErase);
    send_address(config, sectorAddr);
    config->hw_ops->cs_high(config->hw_context);

    status = check_busy(config);
    Log_Message(status == FLASH_OK ? LOG_LEVEL_INFO : LOG_LEVEL_ERROR,
                "[FLASH] Erase Sector 0x%08lX %s", sectorAddr, status == FLASH_OK ? "OK" : "FAIL");
    g_rtos_ops->SemaphoreGive(config->mutex);
    return status;
}

Flash_Status SPI_Flash_EraseChip(SPI_Flash_Config *config)
{
    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to take mutex");
        return FLASH_TIMEOUT;
    }

    Flash_Status status = write_enable(config);
    if (status != FLASH_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write enable failed: %d", status);
        g_rtos_ops->SemaphoreGive(config->mutex);
        return status;
    }

    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_Transfer(config, FLASH_ChipErase);
    config->hw_ops->cs_high(config->hw_context);

    status = check_busy(config);
    Log_Message(status == FLASH_OK ? LOG_LEVEL_INFO : LOG_LEVEL_ERROR,
                "[FLASH] Erase Chip %s", status == FLASH_OK ? "OK" : "FAIL");
    g_rtos_ops->SemaphoreGive(config->mutex);
    return status;
}

Flash_Status SPI_Flash_WritePage(SPI_Flash_Config *config, uint8_t *pBuffer, uint32_t writeAddr, uint16_t numByteToWrite)
{
    if (!config || !pBuffer || numByteToWrite == 0 || numByteToWrite > PAGE_SIZE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid parameters");
        return FLASH_INVALID_PARAM;
    }

    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to take mutex");
        return FLASH_TIMEOUT;
    }

    Flash_Status status = write_enable(config);
    if (status != FLASH_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write enable failed: %d", status);
        g_rtos_ops->SemaphoreGive(config->mutex);
        return status;
    }

    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_Transfer(config, FLASH_PageProgram);
    send_address(config, writeAddr);
    while (numByteToWrite--)
    {
        SPI_Flash_Transfer(config, *pBuffer++);
    }
    config->hw_ops->cs_high(config->hw_context);

    status = check_busy(config);
    Log_Message(status == FLASH_OK ? LOG_LEVEL_INFO : LOG_LEVEL_ERROR,
                "[FLASH] Write Page Addr:0x%08lX Len:%d %s", writeAddr, numByteToWrite, status == FLASH_OK ? "OK" : "FAIL");
    g_rtos_ops->SemaphoreGive(config->mutex);
    return status;
}

Flash_Status SPI_Flash_ReadData(SPI_Flash_Config *config, uint8_t *pBuffer, uint32_t readAddr, uint16_t numByteToRead)
{
    if (!config || !pBuffer || numByteToRead == 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid parameters");
        return FLASH_INVALID_PARAM;
    }

    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to take mutex");
        return FLASH_TIMEOUT;
    }

    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_Transfer(config, FLASH_ReadData);
    send_address(config, readAddr);
    while (numByteToRead--)
    {
        *pBuffer++ = SPI_Flash_Transfer(config, 0xFF);
    }
    config->hw_ops->cs_high(config->hw_context);

    Log_Message(LOG_LEVEL_INFO, "[FLASH] Read Addr:0x%08lX Len:%d", readAddr, numByteToRead);
    g_rtos_ops->SemaphoreGive(config->mutex);
    return FLASH_OK;
}

Flash_Status SPI_Flash_FastReadData(SPI_Flash_Config *config, uint8_t *pBuffer, uint32_t readAddr, uint16_t numByteToRead)
{
    if (!config || !pBuffer || numByteToRead == 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid parameters");
        return FLASH_INVALID_PARAM;
    }

    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to take mutex");
        return FLASH_TIMEOUT;
    }

    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_Transfer(config, FLASH_FastReadData);
    send_address(config, readAddr);
    SPI_Flash_Transfer(config, 0xFF); // 哑字节
    while (numByteToRead--)
    {
        *pBuffer++ = SPI_Flash_Transfer(config, 0xFF);
    }
    config->hw_ops->cs_high(config->hw_context);

    Log_Message(LOG_LEVEL_INFO, "[FLASH] Fast Read Addr:0x%08lX Len:%d", readAddr, numByteToRead);
    g_rtos_ops->SemaphoreGive(config->mutex);
    return FLASH_OK;
}

uint16_t Flash_Read_Id(SPI_Flash_Config *config)
{
    if (!config)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid config");
        return 0;
    }

    uint16_t deviceid;
    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_Transfer(config, FLASH_ManufactDeviceID);
    SPI_Flash_Transfer(config, 0);
    SPI_Flash_Transfer(config, 0);
    SPI_Flash_Transfer(config, 0);
    deviceid = SPI_Flash_Transfer(config, 0xFF) << 8;
    deviceid |= SPI_Flash_Transfer(config, 0xFF);
    config->hw_ops->cs_high(config->hw_context);
    return deviceid;
}

int8_t SPI_Flash_Write_With_Erase(SPI_Flash_Config *config, uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    if (!config || !pbuf || datalen == 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid parameters for write with erase");
        return -1;
    }

    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to take mutex");
        return -1;
    }

    uint32_t start_sector = addr / SECTOR_SIZE;
    uint32_t end_sector = (addr + datalen - 1) / SECTOR_SIZE;

    for (uint32_t sector = start_sector; sector <= end_sector; sector++)
    {
        uint32_t sector_addr = sector * SECTOR_SIZE;
        uint32_t sector_end = sector_addr + SECTOR_SIZE - 1;

        uint32_t data_start = (addr > sector_addr) ? addr : sector_addr;
        uint32_t data_end = (addr + datalen - 1 < sector_end) ? (addr + datalen - 1) : sector_end;
        uint16_t data_len = data_end - data_start + 1;
        uint32_t pbuf_offset = data_start - addr;

        Flash_Status status = SPI_Flash_ReadData(config, sector_buf, sector_addr, SECTOR_SIZE);
        if (status != FLASH_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[FLASH] Sector read failed: %d", status);
            g_rtos_ops->SemaphoreGive(config->mutex);
            return -1;
        }

        // 仅在需要时擦除（数据不同）
        if (memcmp(sector_buf + (data_start - sector_addr), pbuf + pbuf_offset, data_len) != 0)
        {
            status = SPI_Flash_EraseSector(config, sector_addr);
            if (status != FLASH_OK)
            {
                Log_Message(LOG_LEVEL_ERROR, "[FLASH] Sector erase failed: %d", status);
                g_rtos_ops->SemaphoreGive(config->mutex);
                return -1;
            }

            memcpy(sector_buf + (data_start - sector_addr), pbuf + pbuf_offset, data_len);

            uint32_t remain = SECTOR_SIZE;
            uint8_t *src = sector_buf;
            uint32_t write_addr = sector_addr;

            while (remain > 0)
            {
                uint16_t write_len = (remain >= PAGE_SIZE) ? PAGE_SIZE : remain;
                status = SPI_Flash_WritePage(config, src, write_addr, write_len);
                if (status != FLASH_OK)
                {
                    Log_Message(LOG_LEVEL_ERROR, "[FLASH] Page write failed: %d", status);
                    g_rtos_ops->SemaphoreGive(config->mutex);
                    return -1;
                }
                write_addr += write_len;
                src += write_len;
                remain -= write_len;
            }
        }
    }

    Log_Message(LOG_LEVEL_INFO, "[FLASH] Write with erase completed successfully");
    g_rtos_ops->SemaphoreGive(config->mutex);
    return 0;
}
