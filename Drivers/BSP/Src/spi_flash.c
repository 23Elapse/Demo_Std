/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-19 00:03:34
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-30 09:11:10
 * @FilePath: \Demo\Drivers\BSP\Src\spi_flash.c
 * @Description: SPI Flash 驱动实现，支持 RTOS 抽象
 * 
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved. 
 */

 #include "spi_flash.h"
 #include <string.h>
 #include <stdio.h>
 #include "pch.h"
 
 /**
  * @brief 全局命令码数组 - 写状态寄存器命令
  */
 const uint8_t g_flash_write_reg_cmd[] = {
     FLASH_WriteStatusReg1,
     FLASH_WriteStatusReg2,
     FLASH_WriteStatusReg3
 };
 
 /**
  * @brief 全局命令码数组 - 读状态寄存器命令
  */
 const uint8_t g_flash_read_reg_cmd[] = {
     FLASH_ReadStatusReg1,
     FLASH_ReadStatusReg2,
     FLASH_ReadStatusReg3
 };
 
 /**
  * @brief 默认 Flash 类型
  */
 uint16_t g_norflash_type = BY25Q256;
 
 /**
  * @brief 扇区缓冲区
  */
 static uint8_t sector_buf[SECTOR_SIZE];
 
 /**
  * @brief Flash 配置结构体
  */
 SPI_Flash_Config flash_cfg;
 
 /**
  * @brief Flash 互斥锁
  */
 static void *xFlashMutex = NULL;
 
 /**
  * @brief 设置片选信号为低电平
  * @param config Flash 配置结构体指针
  */
 static void CS_Low(SPI_Flash_Config* config)
 {
     GPIO_ResetBits(config->GPIO_Port, config->CS_Pin);
 }
 
 /**
  * @brief 设置片选信号为高电平
  * @param config Flash 配置结构体指针
  */
 static void CS_High(SPI_Flash_Config* config)
 {
     GPIO_SetBits(config->GPIO_Port, config->CS_Pin);
 }
 
 /**
  * @brief SPI 数据传输函数
  * @param config Flash 配置结构体指针
  * @param data 要发送的数据
  * @return 接收到的数据
  */
 uint8_t SPI_Flash_Transfer(SPI_Flash_Config* config, uint8_t data)
 {
     // 等待发送缓冲区空
     while (SPI_I2S_GetFlagStatus(config->SPIx, SPI_I2S_FLAG_TXE) == RESET);
     SPI_I2S_SendData(config->SPIx, data);
     
     // 等待接收缓冲区非空
     while (SPI_I2S_GetFlagStatus(config->SPIx, SPI_I2S_FLAG_RXNE) == RESET);
     return SPI_I2S_ReceiveData(config->SPIx);
 }
 
 /**
  * @brief 发送 Flash 地址（支持3字节或4字节地址模式）
  * @param config Flash 配置结构体指针
  * @param addr 地址值
  */
 static void SendAddress(SPI_Flash_Config* config, uint32_t addr)
 {
     if (g_norflash_type == W25Q256 || g_norflash_type == BY25Q256) {
         SPI_Flash_Transfer(config, (uint8_t)(addr >> 24)); // 发送最高字节
     }
     SPI_Flash_Transfer(config, (addr >> 16) & 0xFF); // 发送高字节
     SPI_Flash_Transfer(config, (addr >> 8) & 0xFF);  // 发送中字节
     SPI_Flash_Transfer(config, addr & 0xFF);         // 发送低字节
 }
 
 /**
  * @brief 使能 Flash 写操作
  * @param config Flash 配置结构体指针
  * @return Flash 操作状态
  */
 static Flash_Status WriteEnable(SPI_Flash_Config* config)
 {
     const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
     if (!rtos_ops) return FLASH_INVALID_PARAM;
 
     uint8_t retry = 3;
     do {
         CS_Low(config);
         SPI_Flash_Transfer(config, FLASH_WriteEnable);
         CS_High(config);
 
         if (SPI_Flash_ReadStatusReg(config, FLASH_StatusReg1) & 0x02) {
             return FLASH_OK;
         }
         rtos_ops->Delay(1); // 延时 1ms
     } while (retry--);
 
     return FLASH_WRITE_PROTECTED;
 }
 
 /**
  * @brief 检查 Flash 是否忙碌
  * @param config Flash 配置结构体指针
  * @return Flash 操作状态
  */
 static Flash_Status CheckBusy(SPI_Flash_Config* config)
 {
     const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
     if (!rtos_ops) return FLASH_INVALID_PARAM;
 
     uint32_t timeout = FLASH_TIMEOUT_CNT;
     uint32_t retry = 3;
 
     do {
         CS_Low(config);
         SPI_Flash_Transfer(config, FLASH_ReadStatusReg1);
         
         uint32_t inner_timeout = timeout;
         do {
             uint8_t status = SPI_Flash_Transfer(config, 0xFF);
             if (!(status & STATUS_REG_BUSY)) {
                 CS_High(config);
                 return FLASH_OK;
             }
         } while (inner_timeout--);
         
         CS_High(config);
         rtos_ops->Delay(1); // 延时 1ms
     } while (retry--);
     
     return FLASH_TIMEOUT;
 }
 
 /**
  * @brief 初始化 SPI Flash 的硬件接口
  * @param config Flash 配置结构体指针
  * @return Flash 操作状态
  */
 Flash_Status SPI_Flash_Init(SPI_Flash_Config* config)
 {
     if (!config || !config->SPIx || !config->GPIO_Port) {
         return FLASH_INVALID_PARAM;
     }
 
     GPIO_InitTypeDef GPIO_InitStruct = {0};
     SPI_InitTypeDef SPI_InitStruct = {0};
 
     // 使能 GPIO 和 SPI 时钟
     RCC_AHB1PeriphClockCmd(config->GPIO_Clk, ENABLE);
     if (config->SPIx == SPI2 || config->SPIx == SPI3) {
         RCC_APB1PeriphClockCmd(config->SPI_Clk, ENABLE);
     } else {
         RCC_APB2PeriphClockCmd(config->SPI_Clk, ENABLE);
     }
 
     // 配置 CS 引脚为推挽输出
     GPIO_InitStruct.GPIO_Pin = config->CS_Pin;
     GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
     GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
     GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
     GPIO_Init(config->GPIO_Port, &GPIO_InitStruct);
     GPIO_SetBits(config->GPIO_Port, config->CS_Pin);
 
     // 配置 SCK/MISO/MOSI 为复用功能
     GPIO_PinAFConfig(config->GPIO_Port, GPIO_PinSource7, GPIO_AF_SPI5); // SCK
     GPIO_PinAFConfig(config->GPIO_Port, GPIO_PinSource8, GPIO_AF_SPI5); // MISO
     GPIO_PinAFConfig(config->GPIO_Port, GPIO_PinSource9, GPIO_AF_SPI5); // MOSI
 
     GPIO_InitStruct.GPIO_Pin = config->SCK_Pin | config->MISO_Pin | config->MOSI_Pin;
     GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
     GPIO_Init(config->GPIO_Port, &GPIO_InitStruct);
 
     // 配置 SPI 参数
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
 
     return FLASH_OK;
 }
 
 /**
  * @brief 初始化 W25Qxx 系列 Flash
  * @param config Flash 配置结构体指针
  * @return Flash 操作状态
  */
 Flash_Status W25Qxx_Init(SPI_Flash_Config* config)
 {
     const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
     if (!rtos_ops || !config) {
         printf("[FLASH] Invalid RTOS or config\r\n");
         return FLASH_INVALID_PARAM;
     }
 
     // 创建互斥锁
     if (xFlashMutex == NULL) {
         xFlashMutex = rtos_ops->SemaphoreCreate();
         if (xFlashMutex == NULL) {
             printf("[FLASH] Failed to create mutex\r\n");
             return FLASH_INVALID_PARAM;
         }
     }
 
     // 获取互斥锁
     if (!rtos_ops->SemaphoreTake(xFlashMutex, 0xFFFFFFFF)) {
         printf("[FLASH] Failed to take mutex\r\n");
         return FLASH_TIMEOUT;
     }
 
     Flash_Status status = SPI_Flash_Init(config);
     if (status != FLASH_OK) {
         rtos_ops->SemaphoreGive(xFlashMutex);
         return status;
     }
 
     g_norflash_type = Flash_Read_Id(config);
     printf("[FLASH] Flash ID: 0x%04X\r\n", g_norflash_type);
 
     // 配置4字节地址模式（针对 W25Q256/BY25Q256）
     if (g_norflash_type == W25Q256 || g_norflash_type == BY25Q256) {
         uint8_t temp = SPI_Flash_ReadStatusReg(config, FLASH_StatusReg3);
         printf("[FLASH] Status Register 3: 0x%02X\r\n", temp);
         if ((temp & 0x01) == 0) {
             status = WriteEnable(config);
             if (status != FLASH_OK) {
                 rtos_ops->SemaphoreGive(xFlashMutex);
                 return status;
             }
             temp |= 1 << 1;
             status = SPI_Flash_WriteStatusReg(config, FLASH_StatusReg3, temp);
             if (status != FLASH_OK) {
                 rtos_ops->SemaphoreGive(xFlashMutex);
                 return status;
             }
             rtos_ops->Delay(20); // 延时 20ms
             CS_Low(config);
             SPI_Flash_Transfer(config, FLASH_Enable4ByteAddr);
             CS_High(config);
         }
     }
 
     printf("[FLASH] SPI Flash initialized successfully!\r\n");
     rtos_ops->SemaphoreGive(xFlashMutex);
     return FLASH_OK;
 }
 
 /**
  * @brief 读取 Flash 状态寄存器
  * @param config Flash 配置结构体指针
  * @param reg 状态寄存器编号
  * @return 寄存器值
  */
 uint8_t SPI_Flash_ReadStatusReg(SPI_Flash_Config* config, Flash_StatusReg reg)
 {
     if (!config || reg > FLASH_StatusReg3) return 0;
 
     CS_Low(config);
     SPI_Flash_Transfer(config, g_flash_read_reg_cmd[reg]);
     uint8_t status = SPI_Flash_Transfer(config, 0xFF);
     CS_High(config);
     return status;
 }
 
 /**
  * @brief 写入 Flash 状态寄存器
  * @param config Flash 配置结构体指针
  * @param reg 状态寄存器编号
  * @param sr 要写入的值
  * @return Flash 操作状态
  */
 Flash_Status SPI_Flash_WriteStatusReg(SPI_Flash_Config* config, Flash_StatusReg reg, uint8_t sr)
 {
     if (!config || reg > FLASH_StatusReg3) return FLASH_INVALID_PARAM;
 
     Flash_Status status = WriteEnable(config);
     if (status != FLASH_OK) return status;
 
     CS_Low(config);
     SPI_Flash_Transfer(config, g_flash_write_reg_cmd[reg]);
     SPI_Flash_Transfer(config, sr);
     CS_High(config);
     return CheckBusy(config);
 }
 
 /**
  * @brief 擦除 Flash 扇区
  * @param config Flash 配置结构体指针
  * @param sectorAddr 扇区地址（字节地址）
  * @return Flash 操作状态
  */
 Flash_Status SPI_Flash_EraseSector(SPI_Flash_Config* config, uint32_t sectorAddr)
 {
     const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
     if (!rtos_ops || !config) return FLASH_INVALID_PARAM;
 
     if (rtos_ops->SemaphoreTake(xFlashMutex, 0xFFFFFFFF)) {
         Flash_Status status = WriteEnable(config);
         if (status != FLASH_OK) {
             rtos_ops->SemaphoreGive(xFlashMutex);
             return status;
         }
 
         CS_Low(config);
         SPI_Flash_Transfer(config, FLASH_SectorErase);
         SendAddress(config, sectorAddr);
         CS_High(config);
 
         status = CheckBusy(config);
         printf("[FLASH] Erase Sector 0x%08lX %s\r\n", sectorAddr, status == FLASH_OK ? "OK" : "FAIL");
         rtos_ops->SemaphoreGive(xFlashMutex);
         return status;
     }
 
     return FLASH_TIMEOUT;
 }
 
 /**
  * @brief 擦除整个 Flash 芯片
  * @param config Flash 配置结构体指针
  * @return Flash 操作状态
  */
 Flash_Status SPI_Flash_EraseChip(SPI_Flash_Config* config)
 {
     const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
     if (!rtos_ops || !config) return FLASH_INVALID_PARAM;
 
     if (rtos_ops->SemaphoreTake(xFlashMutex, 0xFFFFFFFF)) {
         Flash_Status status = WriteEnable(config);
         if (status != FLASH_OK) {
             rtos_ops->SemaphoreGive(xFlashMutex);
             return status;
         }
 
         CS_Low(config);
         SPI_Flash_Transfer(config, FLASH_ChipErase);
         CS_High(config);
 
         status = CheckBusy(config);
         printf("[FLASH] Erase Chip %s\r\n", status == FLASH_OK ? "OK" : "FAIL");
         rtos_ops->SemaphoreGive(xFlashMutex);
         return status;
     }
 
     return FLASH_TIMEOUT;
 }
 
 /**
  * @brief 写入 Flash 页面
  * @param config Flash 配置结构体指针
  * @param pBuffer 数据缓冲区
  * @param writeAddr 写入地址
  * @param numByteToWrite 写入字节数
  * @return Flash 操作状态
  */
 Flash_Status SPI_Flash_WritePage(SPI_Flash_Config* config, uint8_t* pBuffer, uint32_t writeAddr, uint16_t numByteToWrite)
 {
     const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
     if (!rtos_ops || !config || !pBuffer || numByteToWrite == 0) return FLASH_INVALID_PARAM;
 
     if (rtos_ops->SemaphoreTake(xFlashMutex, 0xFFFFFFFF)) {
         Flash_Status status = WriteEnable(config);
         if (status != FLASH_OK) {
             rtos_ops->SemaphoreGive(xFlashMutex);
             return status;
         }
 
         CS_Low(config);
         SPI_Flash_Transfer(config, FLASH_PageProgram);
         SendAddress(config, writeAddr);
         while (numByteToWrite--) {
             SPI_Flash_Transfer(config, *pBuffer++);
         }
         CS_High(config);
 
         status = CheckBusy(config);
         printf("[FLASH] Write Page Addr:0x%08lX Len:%d %s\r\n", writeAddr, numByteToWrite, status == FLASH_OK ? "OK" : "FAIL");
         rtos_ops->SemaphoreGive(xFlashMutex);
         return status;
     }
 
     return FLASH_TIMEOUT;
 }
 
 /**
  * @brief 读取 Flash 数据
  * @param config Flash 配置结构体指针
  * @param pBuffer 数据缓冲区
  * @param readAddr 读取地址
  * @param numByteToRead 读取字节数
  * @return Flash 操作状态
  */
 Flash_Status SPI_Flash_ReadData(SPI_Flash_Config* config, uint8_t* pBuffer, uint32_t readAddr, uint16_t numByteToRead)
 {
     const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
     if (!rtos_ops || !config || !pBuffer || numByteToRead == 0) return FLASH_INVALID_PARAM;
 
     if (rtos_ops->SemaphoreTake(xFlashMutex, 0xFFFFFFFF)) {
         CS_Low(config);
         SPI_Flash_Transfer(config, FLASH_ReadData);
         SendAddress(config, readAddr);
         while (numByteToRead--) {
             *pBuffer++ = SPI_Flash_Transfer(config, 0xFF);
         }
         CS_High(config);
 
         printf("[FLASH] Read Addr:0x%08lX Len:%d\r\n", readAddr, numByteToRead);
         rtos_ops->SemaphoreGive(xFlashMutex);
         return FLASH_OK;
     }
 
     return FLASH_TIMEOUT;
 }
 
 /**
  * @brief 读取 Flash 设备 ID
  * @param config Flash 配置结构体指针
  * @return 设备 ID
  */
 uint16_t Flash_Read_Id(SPI_Flash_Config* config)
 {
     if (!config) return 0;
 
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
 
 /**
  * @brief 带擦除的 Flash 写入操作
  * @param config Flash 配置结构体指针
  * @param pbuf 数据缓冲区
  * @param addr 写入地址
  * @param datalen 数据长度
  * @return 0: 成功; -1: 失败
  */
 int8_t SPI_Flash_Write_With_Erase(SPI_Flash_Config* config, uint8_t *pbuf, uint32_t addr, uint16_t datalen)
 {
     if (!config || !pbuf || datalen == 0) {
         printf("[FLASH] Invalid parameters for write with erase\r\n");
         return -1;
     }
 
     uint32_t start_sector = addr / SECTOR_SIZE;
     uint32_t end_sector = (addr + datalen - 1) / SECTOR_SIZE;
 
     for (uint32_t sector = start_sector; sector <= end_sector; sector++) {
         uint32_t sector_addr = sector * SECTOR_SIZE;
         uint32_t sector_end = sector_addr + SECTOR_SIZE - 1;
 
         // 计算当前扇区需要写入的数据范围
         uint32_t data_start = (addr > sector_addr) ? addr : sector_addr;
         uint32_t data_end = (addr + datalen - 1 < sector_end) ? (addr + datalen - 1) : sector_end;
         uint16_t data_len = data_end - data_start + 1;
         uint32_t pbuf_offset = data_start - addr;
 
         // 完全覆盖整个扇区的优化情况
         if (data_start == sector_addr && data_len == SECTOR_SIZE) {
             // 直接擦除并写入新数据
             Flash_Status status = SPI_Flash_EraseSector(config, sector_addr);
             if (status != FLASH_OK) {
                 return -1;
             }
 
             uint32_t remain = SECTOR_SIZE;
             uint8_t* src = pbuf + pbuf_offset;
             uint32_t write_addr = sector_addr;
 
             while (remain > 0) {
                 uint16_t write_len = (remain >= PAGE_SIZE) ? PAGE_SIZE : remain;
                 status = SPI_Flash_WritePage(config, src, write_addr, write_len);
                 if (status != FLASH_OK) {
                     return -1;
                 }
                 write_addr += write_len;
                 src += write_len;
                 remain -= write_len;
             }
         } else {
             // 部分覆盖扇区，需要备份原始数据
             // 读取整个扇区原始数据
             Flash_Status status = SPI_Flash_ReadData(config, sector_buf, sector_addr, SECTOR_SIZE);
             if (status != FLASH_OK) {
                 return -1;
             }
 
             // 合并新数据到缓冲区
             memcpy(sector_buf + (data_start - sector_addr), pbuf + pbuf_offset, data_len);
 
             // 擦除扇区
             status = SPI_Flash_EraseSector(config, sector_addr);
             if (status != FLASH_OK) {
                 return -1;
             }
 
             // 写回整个扇区
             uint32_t remain = SECTOR_SIZE;
             uint8_t* src = sector_buf;
             uint32_t write_addr = sector_addr;
 
             while (remain > 0) {
                 uint16_t write_len = (remain >= PAGE_SIZE) ? PAGE_SIZE : remain;
                 status = SPI_Flash_WritePage(config, src, write_addr, write_len);
                 if (status != FLASH_OK) {
                     return -1;
                 }
                 write_addr += write_len;
                 src += write_len;
                 remain -= write_len;
             }
         }
     }
 
     return 0;
 }
// 2. 定义 SPI Flash 设备
SPI_Flash_Config flash_config = {
    .SPIx = SPI5,
    .GPIO_Port = GPIOF,
    .SPI_Clk = RCC_APB2Periph_SPI5,
    .GPIO_Clk = RCC_AHB1Periph_GPIOF,
    .CS_Pin = GPIO_Pin_6,
    .SCK_Pin = GPIO_Pin_7,
    .MISO_Pin = GPIO_Pin_8,
    .MOSI_Pin = GPIO_Pin_9,
    .address_mode = FLASH_4BYTE_MODE
};
 /*
  * 示例用法：
  * 1. 设置 RTOS 抽象层
  * RTOS_SetOps(&FreeRTOS_Ops); // 或 RTThread_Ops
  *
  * 2. 定义 SPI Flash 设备
  * SPI_Flash_Config flash_config = {
  *     .SPIx = SPI5,
  *     .GPIO_Port = GPIOF,
  *     .SPI_Clk = RCC_APB2Periph_SPI5,
  *     .GPIO_Clk = RCC_AHB1Periph_GPIOF,
  *     .CS_Pin = GPIO_Pin_6,
  *     .SCK_Pin = GPIO_Pin_7,
  *     .MISO_Pin = GPIO_Pin_8,
  *     .MOSI_Pin = GPIO_Pin_9,
  *     .address_mode = FLASH_4BYTE_MODE
  * };
  *
  * 3. 初始化 Flash
  * W25Qxx_Init(&flash_config);
  *
  * 4. 写入数据
  * uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
  * SPI_Flash_Write_With_Erase(&flash_config, data, 0x00000000, sizeof(data));
  *
  * 5. 读取数据
  * uint8_t read_buffer[4];
  * SPI_Flash_ReadData(&flash_config, read_buffer, 0x00000000, sizeof(read_buffer));
  */

