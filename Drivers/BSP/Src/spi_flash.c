/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-19 00:03:34
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 00:20:40
 * @FilePath: \Demo\Drivers\BSP\Src\spi_flash.c
 * @Description: SPI Flash 驱动实现，支持 RTOS 抽象
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "spi_flash.h"
#include <string.h>
#include "log_system.h"
#include "rtos_abstraction.h"
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
    .mutex = NULL};

static void CS_Low(SPI_Flash_Config *config)
{
    GPIO_ResetBits(config->GPIO_Port, config->CS_Pin);
}

static void CS_High(SPI_Flash_Config *config)
{
    GPIO_SetBits(config->GPIO_Port, config->CS_Pin);
}

uint8_t SPI_Flash_Transfer(SPI_Flash_Config *config, uint8_t data)
{
    while (SPI_I2S_GetFlagStatus(config->SPIx, SPI_I2S_FLAG_TXE) == RESET)
        ;
    SPI_I2S_SendData(config->SPIx, data);
    while (SPI_I2S_GetFlagStatus(config->SPIx, SPI_I2S_FLAG_RXNE) == RESET)
        ;
    return SPI_I2S_ReceiveData(config->SPIx);
}

static void SendAddress(SPI_Flash_Config *config, uint32_t addr)
{
    if (g_norflash_type == W25Q256 || g_norflash_type == BY25Q256)
    {
        SPI_Flash_Transfer(config, (uint8_t)(addr >> 24));
    }
    SPI_Flash_Transfer(config, (addr >> 16) & 0xFF);
    SPI_Flash_Transfer(config, (addr >> 8) & 0xFF);
    SPI_Flash_Transfer(config, addr & 0xFF);
}

static Flash_Status WriteEnable(SPI_Flash_Config *config)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid RTOS ops");
        return FLASH_INVALID_PARAM;
    }

    uint8_t retry = 3;
    do
    {
        CS_Low(config);
        SPI_Flash_Transfer(config, FLASH_WriteEnable);
        CS_High(config);

        if (SPI_Flash_ReadStatusReg(config, FLASH_StatusReg1) & 0x02)
        {
            return FLASH_OK;
        }
        rtos_ops->Delay(1);
    } while (retry--);

    Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write enable failed");
    return FLASH_WRITE_PROTECTED;
}

static Flash_Status CheckBusy(SPI_Flash_Config *config)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid RTOS ops");
        return FLASH_INVALID_PARAM;
    }

    uint32_t timeout = FLASH_TIMEOUT_CNT;
    uint8_t retry = 3;

    do
    {
        CS_Low(config);
        SPI_Flash_Transfer(config, FLASH_ReadStatusReg1);

        uint32_t inner_timeout = timeout;
        do
        {
            uint8_t status = SPI_Flash_Transfer(config, 0xFF);
            if (!(status & STATUS_REG_BUSY))
            {
                CS_High(config);
                return FLASH_OK;
            }
        } while (inner_timeout--);

        CS_High(config);
        rtos_ops->Delay(1);
    } while (retry--);

    Log_Message(LOG_LEVEL_ERROR, "[FLASH] Operation timeout");
    return FLASH_TIMEOUT;
}

Flash_Status SPI_Flash_Init(SPI_Flash_Config *config)
{
    if (!config || !config->SPIx || !config->GPIO_Port)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid config parameters");
        return FLASH_INVALID_PARAM;
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
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !config)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid RTOS or config");
        return FLASH_INVALID_PARAM;
    }

    if (!config->mutex)
    {
        config->mutex = rtos_ops->SemaphoreCreate();
        if (!config->mutex)
        {
            Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to create mutex");
            return FLASH_INVALID_PARAM;
        }
    }

    if (!rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to take mutex");
        return FLASH_TIMEOUT;
    }

    Flash_Status status = SPI_Flash_Init(config);
    if (status != FLASH_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] SPI init failed: %d", status);
        rtos_ops->SemaphoreGive(config->mutex);
        return status;
    }

    g_norflash_type = Flash_Read_Id(config);
    Log_Message(LOG_LEVEL_INFO, "[FLASH] Flash ID: 0x%04X", g_norflash_type);

    if (g_norflash_type == W25Q256 || g_norflash_type == BY25Q256)
    {
        uint8_t temp = SPI_Flash_ReadStatusReg(config, FLASH_StatusReg3);
        Log_Message(LOG_LEVEL_INFO, "[FLASH] Status Register 3: 0x%02X", temp);
        if ((temp & 0x01) == 0)
        {
            status = WriteEnable(config);
            if (status != FLASH_OK)
            {
                Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write enable failed: %d", status);
                rtos_ops->SemaphoreGive(config->mutex);
                return status;
            }
            temp |= 1 << 1;
            status = SPI_Flash_WriteStatusReg(config, FLASH_StatusReg3, temp);
            if (status != FLASH_OK)
            {
                Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write status reg failed: %d", status);
                rtos_ops->SemaphoreGive(config->mutex);
                return status;
            }
            rtos_ops->Delay(20);
            CS_Low(config);
            SPI_Flash_Transfer(config, FLASH_Enable4ByteAddr);
            CS_High(config);
        }
    }

    Log_Message(LOG_LEVEL_INFO, "[FLASH] SPI Flash initialized successfully");
    rtos_ops->SemaphoreGive(config->mutex);
    return FLASH_OK;
}

uint8_t SPI_Flash_ReadStatusReg(SPI_Flash_Config *config, Flash_StatusReg reg)
{
    if (!config || reg > FLASH_StatusReg3)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid config or register");
        return 0;
    }

    CS_Low(config);
    SPI_Flash_Transfer(config, g_flash_read_reg_cmd[reg]);
    uint8_t status = SPI_Flash_Transfer(config, 0xFF);
    CS_High(config);
    return status;
}

Flash_Status SPI_Flash_WriteStatusReg(SPI_Flash_Config *config, Flash_StatusReg reg, uint8_t sr)
{
    if (!config || reg > FLASH_StatusReg3)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid config or register");
        return FLASH_INVALID_PARAM;
    }

    Flash_Status status = WriteEnable(config);
    if (status != FLASH_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write enable failed: %d", status);
        return status;
    }

    CS_Low(config);
    SPI_Flash_Transfer(config, g_flash_write_reg_cmd[reg]);
    SPI_Flash_Transfer(config, sr);
    CS_High(config);
    return CheckBusy(config);
}

Flash_Status SPI_Flash_EraseSector(SPI_Flash_Config *config, uint32_t sectorAddr)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !config)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid RTOS or config");
        return FLASH_INVALID_PARAM;
    }

    if (rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF))
    {
        Flash_Status status = WriteEnable(config);
        if (status != FLASH_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write enable failed: %d", status);
            rtos_ops->SemaphoreGive(config->mutex);
            return status;
        }

        CS_Low(config);
        SPI_Flash_Transfer(config, FLASH_SectorErase);
        SendAddress(config, sectorAddr);
        CS_High(config);

        status = CheckBusy(config);
        Log_Message(status == FLASH_OK ? LOG_LEVEL_INFO : LOG_LEVEL_ERROR,
                    "[FLASH] Erase Sector 0x%08lX %s", sectorAddr, status == FLASH_OK ? "OK" : "FAIL");
        rtos_ops->SemaphoreGive(config->mutex);
        return status;
    }

    Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to take mutex");
    return FLASH_TIMEOUT;
}

Flash_Status SPI_Flash_EraseChip(SPI_Flash_Config *config)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !config)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid RTOS or config");
        return FLASH_INVALID_PARAM;
    }

    if (rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF))
    {
        Flash_Status status = WriteEnable(config);
        if (status != FLASH_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write enable failed: %d", status);
            rtos_ops->SemaphoreGive(config->mutex);
            return status;
        }

        CS_Low(config);
        SPI_Flash_Transfer(config, FLASH_ChipErase);
        CS_High(config);

        status = CheckBusy(config);
        Log_Message(status == FLASH_OK ? LOG_LEVEL_INFO : LOG_LEVEL_ERROR,
                    "[FLASH] Erase Chip %s", status == FLASH_OK ? "OK" : "FAIL");
        rtos_ops->SemaphoreGive(config->mutex);
        return status;
    }

    Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to take mutex");
    return FLASH_TIMEOUT;
}

Flash_Status SPI_Flash_WritePage(SPI_Flash_Config *config, uint8_t *pBuffer, uint32_t writeAddr, uint16_t numByteToWrite)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !config || !pBuffer || numByteToWrite == 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid parameters");
        return FLASH_INVALID_PARAM;
    }

    if (rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF))
    {
        Flash_Status status = WriteEnable(config);
        if (status != FLASH_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write enable failed: %d", status);
            rtos_ops->SemaphoreGive(config->mutex);
            return status;
        }

        CS_Low(config);
        SPI_Flash_Transfer(config, FLASH_PageProgram);
        SendAddress(config, writeAddr);
        while (numByteToWrite--)
        {
            SPI_Flash_Transfer(config, *pBuffer++);
        }
        CS_High(config);

        status = CheckBusy(config);
        Log_Message(status == FLASH_OK ? LOG_LEVEL_INFO : LOG_LEVEL_ERROR,
                    "[FLASH] Write Page Addr:0x%08lX Len:%d %s", writeAddr, numByteToWrite, status == FLASH_OK ? "OK" : "FAIL");
        rtos_ops->SemaphoreGive(config->mutex);
        return status;
    }

    Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to take mutex");
    return FLASH_TIMEOUT;
}

Flash_Status SPI_Flash_ReadData(SPI_Flash_Config *config, uint8_t *pBuffer, uint32_t readAddr, uint16_t numByteToRead)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !config || !pBuffer || numByteToRead == 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid parameters");
        return FLASH_INVALID_PARAM;
    }

    if (rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF))
    {
        CS_Low(config);
        SPI_Flash_Transfer(config, FLASH_ReadData);
        SendAddress(config, readAddr);
        while (numByteToRead--)
        {
            *pBuffer++ = SPI_Flash_Transfer(config, 0xFF);
        }
        CS_High(config);

        Log_Message(LOG_LEVEL_INFO, "[FLASH] Read Addr:0x%08lX Len:%d", readAddr, numByteToRead);
        rtos_ops->SemaphoreGive(config->mutex);
        return FLASH_OK;
    }

    Log_Message(LOG_LEVEL_ERROR, "[FLASH] Failed to take mutex");
    return FLASH_TIMEOUT;
}

uint16_t Flash_Read_Id(SPI_Flash_Config *config)
{
    if (!config)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid config");
        return 0;
    }

    uint16_t deviceid;
    CS_Low(config);
    SPI_Flash_Transfer(config, FLASH_ManufactDeviceID);
    SPI_Flash_Transfer(config, 0);
    SPI_Flash_Transfer(config, 0);
    SPI_Flash_Transfer(config, 0);
    deviceid = SPI_Flash_Transfer(config, 0xFF) << 8;
    deviceid |= SPI_Flash_Transfer(config, 0xFF);
    CS_High(config);
    return deviceid;
}

int8_t SPI_Flash_Write_With_Erase(SPI_Flash_Config *config, uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    if (!config || !pbuf || datalen == 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Invalid parameters for write with erase");
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

        if (data_start == sector_addr && data_len == SECTOR_SIZE)
        {
            Flash_Status status = SPI_Flash_EraseSector(config, sector_addr);
            if (status != FLASH_OK)
            {
                Log_Message(LOG_LEVEL_ERROR, "[FLASH] Sector erase failed: %d", status);
                return -1;
            }

            uint32_t remain = SECTOR_SIZE;
            uint8_t *src = pbuf + pbuf_offset;
            uint32_t write_addr = sector_addr;

            while (remain > 0)
            {
                uint16_t write_len = (remain >= PAGE_SIZE) ? PAGE_SIZE : remain;
                status = SPI_Flash_WritePage(config, src, write_addr, write_len);
                if (status != FLASH_OK)
                {
                    Log_Message(LOG_LEVEL_ERROR, "[FLASH] Page write failed: %d", status);
                    return -1;
                }
                write_addr += write_len;
                src += write_len;
                remain -= write_len;
            }
        }
        else
        {
            Flash_Status status = SPI_Flash_ReadData(config, sector_buf, sector_addr, SECTOR_SIZE);
            if (status != FLASH_OK)
            {
                Log_Message(LOG_LEVEL_ERROR, "[FLASH] Sector read failed: %d", status);
                return -1;
            }

            memcpy(sector_buf + (data_start - sector_addr), pbuf + pbuf_offset, data_len);

            status = SPI_Flash_EraseSector(config, sector_addr);
            if (status != FLASH_OK)
            {
                Log_Message(LOG_LEVEL_ERROR, "[FLASH] Sector erase failed: %d", status);
                return -1;
            }

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
                    return -1;
                }
                write_addr += write_len;
                src += write_len;
                remain -= write_len;
            }
        }
    }

    Log_Message(LOG_LEVEL_INFO, "[FLASH] Write with erase completed successfully");
    return 0;
}
