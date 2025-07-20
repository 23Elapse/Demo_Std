/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-19 00:03:34
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-14 18:25:17
 * @FilePath: \Demo_backup\Drivers\BSP\spi_flash.c
 * @Description: SPI Flash 驱动实现，支持 RTOS 抽象和优化 (Refactored)
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "spi_flash.h"
#include <string.h> // For memcpy, memcmp
#include "log_system.h"
#include "rtos_abstraction.h"
#include "stm32f4xx_dma.h" // 如果使用DMA，需要此头文件
#include "pch.h"          // 包含FreeRTOS相关的宏，如 pdFALSE, portYIELD_FROM_ISR
#include "common_driver.h" // For Common_GPIO_Init

// 静态缓冲区，用于扇区读-修改-写操作
static uint8_t s_sector_buffer[SECTOR_SIZE];

// 全局的Flash芯片类型，用于内部逻辑判断，如地址模式
// 可以在Flash_Device_Init时读取并更新到 SPI_Flash_Device_t 结构体中
static uint16_t s_current_flash_chip_id = 0;

// Flash 状态寄存器写命令数组
const uint8_t g_flash_write_reg_cmd[] = {
    FLASH_WriteStatusReg1,
    FLASH_WriteStatusReg2,
    FLASH_WriteStatusReg3
};

// Flash 状态寄存器读命令数组
const uint8_t g_flash_read_reg_cmd[] = {
    FLASH_ReadStatusReg1,
    FLASH_ReadStatusReg2,
    FLASH_ReadStatusReg3
};

/*
 * =====================================================================================
 * 内部 SPI 硬件操作回调函数的默认实现
 * 这些函数将在 SPI_Flash_Hardware_Init 中设置为默认回调，除非用户提供自定义实现
 * =====================================================================================
 */

/**
 * @brief 默认 SPI 单字节传输回调
 * @param hw_context 硬件上下文指针，此处指向 SPI_Flash_Config_t
 * @param data 要发送的字节
 * @return 接收到的字节
 */
static uint8_t default_spi_transfer(void *hw_context, uint8_t data)
{
    SPI_Flash_Config_t *config = (SPI_Flash_Config_t *)hw_context;
    // 等待发送缓冲区空
    while (SPI_I2S_GetFlagStatus(config->SPIx, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(config->SPIx, data); // 发送数据
    // 等待接收缓冲区非空
    while (SPI_I2S_GetFlagStatus(config->SPIx, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(config->SPIx); // 返回接收到的数据
}

/**
 * @brief 默认片选拉低回调
 * @param hw_context 硬件上下文指针
 */
static void default_cs_low(void *hw_context)
{
    SPI_Flash_Config_t *config = (SPI_Flash_Config_t *)hw_context;
    GPIO_ResetBits(config->GPIO_Port, config->CS_Pin);
}

/**
 * @brief 默认片选拉高回调
 * @param hw_context 硬件上下文指针
 */
static void default_cs_high(void *hw_context)
{
    SPI_Flash_Config_t *config = (SPI_Flash_Config_t *)hw_context;
    GPIO_SetBits(config->GPIO_Port, config->CS_Pin);
}

/**
 * @brief 默认 DMA 传输回调 (占位符，需要根据实际DMA驱动实现)
 * @param hw_context 硬件上下文指针
 * @param tx_buf 发送数据缓冲区
 * @param rx_buf 接收数据缓冲区
 * @param len 传输长度
 * @return Flash_Status_t 传输状态
 */
static Flash_Status_t default_dma_transfer(void *hw_context, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len)
{
    Log_Message(LOG_LEVEL_WARNING, "[FLASH] DMA transfer not implemented or used.");
    // 简单的同步DMA模拟，实际DMA需要异步处理
    SPI_Flash_Config_t *config = (SPI_Flash_Config_t *)hw_context;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t tx_data = tx_buf ? tx_buf[i] : 0xFF; // 如果没有发送数据，发送0xFF
        uint8_t rx_data = default_spi_transfer(hw_context, tx_data);
        if (rx_buf) {
            rx_buf[i] = rx_data;
        }
    }
    return FLASH_OK;
}

/*
 * =====================================================================================
 * 内部辅助函数
 * =====================================================================================
 */

/**
 * @brief 发送 Flash 地址 (根据地址模式自动选择3字节或4字节)
 * @param config Flash 配置结构体指针
 * @param addr 要发送的地址
 */
static void send_address(SPI_Flash_Config_t *config, uint32_t addr)
{
    if (config->address_mode == FLASH_4BYTE_MODE) // 如果是4字节地址模式
    {
        // 某些芯片在进入4字节模式后，所有指令的地址都必须是4字节
        // 通常W25Q256FV和BY25Q256支持4字节地址
        SPI_Flash_TransferByte(config, (uint8_t)(addr >> 24));
    }
    SPI_Flash_TransferByte(config, (addr >> 16) & 0xFF);
    SPI_Flash_TransferByte(config, (addr >> 8) & 0xFF);
    SPI_Flash_TransferByte(config, addr & 0xFF);
}

/**
 * @brief 检查 Flash 是否忙碌
 * @param config Flash 配置结构体指针
 * @return Flash_Status_t 操作状态 (FLASH_OK 或 FLASH_TIMEOUT)
 */
static Flash_Status_t check_busy(SPI_Flash_Config_t *config)
{
    uint32_t timeout = FLASH_TIMEOUT_CNT;
    uint8_t status_reg;

    do
    {
        status_reg = SPI_Flash_ReadStatusReg(config, FLASH_StatusReg1);
        if (!(status_reg & STATUS_REG_BUSY)) // 检查BUSY位是否为0
        {
            return FLASH_OK;
        }
        g_rtos_ops->Delay(1); // 延时1ms，避免忙等
    } while (--timeout);

    Log_Message(LOG_LEVEL_ERROR, "[FLASH] Check busy timeout. Status: 0x%02X", status_reg);
    return FLASH_TIMEOUT;
}

/**
 * @brief 启用 Flash 写入
 * @param config Flash 配置结构体指针
 * @return Flash_Status_t 操作状态 (FLASH_OK 或 FLASH_WRITE_PROTECTED/FLASH_TIMEOUT)
 */
static Flash_Status_t write_enable(SPI_Flash_Config_t *config)
{
    uint32_t timeout = FLASH_TIMEOUT_CNT / 10; // 适当超时
    uint8_t status_reg;

    do
    {
        config->hw_ops->cs_low(config->hw_context);
        SPI_Flash_TransferByte(config, FLASH_WriteEnable);
        config->hw_ops->cs_high(config->hw_context);

        status_reg = SPI_Flash_ReadStatusReg(config, FLASH_StatusReg1);
        if (status_reg & 0x02) // 检查WEL (Write Enable Latch) 位是否置位
        {
            return FLASH_OK;
        }
        g_rtos_ops->Delay(1);
    } while (--timeout);

    Log_Message(LOG_LEVEL_ERROR, "[FLASH] Write enable failed or timeout. Status: 0x%02X", status_reg);
    return FLASH_WRITE_PROTECTED;
}

/*
 * =====================================================================================
 * 公共API函数实现
 * =====================================================================================
 */

/**
 * @brief 初始化 SPI Flash 硬件接口 (仅SPI外设和GPIO)
 * @param config Flash 配置结构体指针
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_Hardware_Init(SPI_Flash_Config_t *config)
{
    if (!config || !config->SPIx || !config->GPIO_Port) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Hardware Init: Invalid config parameters.");
        return FLASH_INVALID_PARAM;
    }

    // 设置默认硬件操作回调，如果用户没有提供自定义实现
    // 确保这些函数指针只设置一次，或者由外部统一管理
    if (!config->hw_ops) {
        static SPI_Hw_Ops_t default_ops = {
            .transfer = default_spi_transfer,
            .cs_low = default_cs_low,
            .cs_high = default_cs_high,
            .dma_transfer = default_dma_transfer
        };
        config->hw_ops = &default_ops;
        config->hw_context = config; // 默认上下文指向配置自身
    } else if (!config->hw_context) {
        config->hw_context = config; // 如果hw_ops已设置，但hw_context为空，则默认指向config
    }

    // 1. 使能时钟
    RCC_AHB1PeriphClockCmd(config->GPIO_Clk, ENABLE); // GPIO时钟
    if (config->SPIx == SPI1 || config->SPIx == SPI4 || config->SPIx == SPI5 || config->SPIx == SPI6) {
        RCC_APB2PeriphClockCmd(config->SPI_Clk, ENABLE); // APB2总线上的SPI
    } else if (config->SPIx == SPI2 || config->SPIx == SPI3) {
        RCC_APB1PeriphClockCmd(config->SPI_Clk, ENABLE); // APB1总线上的SPI
    } else {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Hardware Init: Invalid SPI instance.");
        return FLASH_INVALID_PARAM;
    }

    // 2. 初始化 CS 引脚 (推挽输出，默认拉高)
    if (Common_GPIO_Init(config->GPIO_Port, config->CS_Pin, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0) != COMMON_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Hardware Init: Failed to init CS GPIO.");
        return FLASH_SPI_HW_ERROR;
    }
    GPIO_SetBits(config->GPIO_Port, config->CS_Pin); // CS 默认拉高

    // 3. 配置 SPI GPIO 引脚复用功能
    // 假设这些引脚都属于同一个端口，且复用功能一致
    // 注意：GPIO_PinSourcex 需要根据实际引脚和SPI外设进行配置，这里可能需要更精确的查找表
    // 针对F407xx的SPI5 F6,F7,F8,F9:
    // PF6 -> SPI5_NSS (如果NSS是硬件管理)
    // PF7 -> SPI5_SCK
    // PF8 -> SPI5_MISO
    // PF9 -> SPI5_MOSI
    // 这里只初始化SCK, MISO, MOSI为复用功能
    GPIO_PinAFConfig(config->GPIO_Port, config->SCK_Pin, GPIO_AF_SPI5);
    GPIO_PinAFConfig(config->GPIO_Port, config->MISO_Pin, GPIO_AF_SPI5);
    GPIO_PinAFConfig(config->GPIO_Port, config->MOSI_Pin, GPIO_AF_SPI5);

    if (Common_GPIO_Init(config->GPIO_Port, config->SCK_Pin | config->MISO_Pin | config->MOSI_Pin,
                         GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0) != COMMON_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Hardware Init: Failed to init SPI AF GPIOs.");
        return FLASH_SPI_HW_ERROR;
    }

    // 4. 初始化 SPI 外设
    SPI_InitTypeDef SPI_InitStruct = {0};
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 全双工
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;                      // 主机模式
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;                  // 8位数据帧
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;                        // 时钟空闲时高电平
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;                       // 第二个时钟沿捕获数据
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;                          // 软件片选管理
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // 波特率预分频 (根据实际时钟和目标频率调整)
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;                 // MSB先行
    SPI_InitStruct.SPI_CRCPolynomial = 7;                           // CRC多项式
    SPI_Init(config->SPIx, &SPI_InitStruct);
    SPI_Cmd(config->SPIx, ENABLE); // 使能 SPI 外设

    Log_Message(LOG_LEVEL_INFO, "[FLASH] SPI hardware initialized successfully.");
    return FLASH_OK;
}

/**
 * @brief 初始化 SPI Flash 设备 (包括硬件和芯片特定初始化)
 * @param dev Flash 设备句柄指针
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_Device_Init(SPI_Flash_Device_t *dev)
{
    if (!dev || !dev->config) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Device Init: Invalid device or config pointer.");
        return FLASH_INVALID_PARAM;
    }

    SPI_Flash_Config_t *config = dev->config;

    // 1. 创建互斥锁 (如果尚未创建)
    if (config->mutex == NULL) {
        config->mutex = g_rtos_ops->SemaphoreCreate();
        if (config->mutex == NULL) {
            Log_Message(LOG_LEVEL_ERROR, "[FLASH] Device Init: Failed to create mutex.");
            return FLASH_MUTEX_TIMEOUT; // 使用更具体的错误码
        }
    }

    // 2. 获取互斥锁，确保初始化过程的原子性
    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF)) { // 永远等待
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Device Init: Failed to take mutex for initialization.");
        return FLASH_MUTEX_TIMEOUT;
    }

    // 3. 初始化 SPI 硬件
    Flash_Status_t status = SPI_Flash_Hardware_Init(config);
    if (status != FLASH_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Device Init: SPI hardware init failed (%d).", status);
        g_rtos_ops->SemaphoreGive(config->mutex);
        return status;
    }

    // 4. 读取芯片ID
    dev->chip_id = SPI_Flash_ReadChipId(config);
    s_current_flash_chip_id = dev->chip_id; // 更新内部全局变量
    Log_Message(LOG_LEVEL_INFO, "[FLASH] Device Init: Detected Flash ID: 0x%04X.", dev->chip_id);

    if (dev->chip_id == 0x0000 || dev->chip_id == 0xFFFF) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] Device Init: Invalid Flash ID detected.");
        g_rtos_ops->SemaphoreGive(config->mutex);
        return FLASH_UNSUPPORTED_CHIP;
    }

    // 5. 根据芯片ID进行特定配置 (例如W25Q256的4字节地址模式)
    if (dev->chip_id == W25Q256 || dev->chip_id == BY25Q256 || dev->chip_id == W25Q512) {
        config->address_mode = FLASH_4BYTE_MODE; // 强制设置为4字节地址模式
        uint8_t status_reg3 = SPI_Flash_ReadStatusReg(config, FLASH_StatusReg3);
        Log_Message(LOG_LEVEL_INFO, "[FLASH] Device Init: Status Register 3 (0x%02X).", status_reg3);

        // 检查QE位 (Quad Enable) 和 4BYTEADDR 位 (如果需要)
        // 注意：这里仅启用4字节地址模式，QE位的设置通常在StatusReg2中，并取决于具体使用QPI还是标准SPI
        // 以下代码仅为使能4字节地址模式
        if ((status_reg3 & 0x01) == 0) // 如果4BYTEADDR位未设置
        {
            status = write_enable(config);
            if (status != FLASH_OK) {
                Log_Message(LOG_LEVEL_ERROR, "[FLASH] Device Init: Write Enable failed for 4-byte address enable (%d).", status);
                g_rtos_ops->SemaphoreGive(config->mutex);
                return status;
            }
            // 发送进入4字节地址模式命令
            config->hw_ops->cs_low(config->hw_context);
            SPI_Flash_TransferByte(config, FLASH_Enable4ByteAddr);
            config->hw_ops->cs_high(config->hw_context);

            status = check_busy(config); // 等待操作完成
            if (status != FLASH_OK) {
                Log_Message(LOG_LEVEL_ERROR, "[FLASH] Device Init: Enable 4-byte address timeout (%d).", status);
                g_rtos_ops->SemaphoreGive(config->mutex);
                return status;
            }
            Log_Message(LOG_LEVEL_INFO, "[FLASH] Device Init: Enabled 4-byte address mode.");
        }
    } else {
        config->address_mode = FLASH_3BYTE_MODE; // 其他芯片默认3字节地址模式
    }

    g_rtos_ops->SemaphoreGive(config->mutex); // 释放互斥锁
    Log_Message(LOG_LEVEL_INFO, "[FLASH] SPI Flash device initialized successfully. Chip ID: 0x%04X.", dev->chip_id);
    return FLASH_OK;
}

/**
 * @brief SPI 单字节数据传输
 * @param config Flash 配置结构体指针
 * @param data 要发送的数据
 * @return 接收到的数据
 */
uint8_t SPI_Flash_TransferByte(SPI_Flash_Config_t *config, uint8_t data)
{
    if (!config || !config->hw_ops || !config->hw_ops->transfer) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] TransferByte: Invalid config or transfer callback.");
        return 0; // 返回0表示错误或无效
    }
    return config->hw_ops->transfer(config->hw_context, data);
}

/**
 * @brief 读取 Flash 状态寄存器
 * @param config Flash 配置结构体指针
 * @param reg 状态寄存器编号 (Flash_StatusReg_t)
 * @return 寄存器值
 */
uint8_t SPI_Flash_ReadStatusReg(SPI_Flash_Config_t *config, Flash_StatusReg_t reg)
{
    if (!config || reg > FLASH_StatusReg3) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] ReadStatusReg: Invalid config or register (%u).", reg);
        return 0xFF; // 返回FF表示错误
    }

    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_TransferByte(config, g_flash_read_reg_cmd[reg]);
    uint8_t status = SPI_Flash_TransferByte(config, 0xFF); // 发送哑字节以读取数据
    config->hw_ops->cs_high(config->hw_context);
    return status;
}

/**
 * @brief 写入 Flash 状态寄存器
 * @param config Flash 配置结构体指针
 * @param reg 状态寄存器编号 (Flash_StatusReg_t)
 * @param sr 要写入的值
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_WriteStatusReg(SPI_Flash_Config_t *config, Flash_StatusReg_t reg, uint8_t sr)
{
    if (!config || reg > FLASH_StatusReg3) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] WriteStatusReg: Invalid config or register (%u).", reg);
        return FLASH_INVALID_PARAM;
    }

    // 1. 尝试获取互斥锁
    if (!g_rtos_ops->SemaphoreTake(config->mutex, 100)) { // 100ms 超时
        Log_Message(LOG_LEVEL_WARNING, "[FLASH] WriteStatusReg: Failed to get mutex (timeout).");
        return FLASH_MUTEX_TIMEOUT;
    }

    // 2. 写使能
    Flash_Status_t status = write_enable(config);
    if (status != FLASH_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] WriteStatusReg: Write enable failed (%d).", status);
        g_rtos_ops->SemaphoreGive(config->mutex);
        return status;
    }

    // 3. 写入状态寄存器命令和数据
    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_TransferByte(config, g_flash_write_reg_cmd[reg]);
    SPI_Flash_TransferByte(config, sr);
    config->hw_ops->cs_high(config->hw_context);

    // 4. 等待Flash操作完成
    status = check_busy(config);
    if (status != FLASH_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] WriteStatusReg: Check busy timeout after write (%d).", status);
    } else {
        Log_Message(LOG_LEVEL_INFO, "[FLASH] Write Status Reg %u: 0x%02X OK.", reg, sr);
    }

    g_rtos_ops->SemaphoreGive(config->mutex); // 释放互斥锁
    return status;
}

/**
 * @brief 擦除 Flash 扇区
 * @param dev Flash 设备句柄指针
 * @param sector_addr 扇区地址（字节地址，必须是扇区起始地址）
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_EraseSector(SPI_Flash_Device_t *dev, uint32_t sector_addr)
{
    if (!dev || !dev->config || (sector_addr % SECTOR_SIZE != 0)) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] EraseSector: Invalid parameters (dev/config or address not sector aligned).");
        return FLASH_INVALID_PARAM;
    }

    SPI_Flash_Config_t *config = dev->config;

    // 1. 尝试获取互斥锁
    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF)) { // 永远等待
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] EraseSector: Failed to get mutex (timeout).");
        return FLASH_MUTEX_TIMEOUT;
    }

    // 2. 写使能
    Flash_Status_t status = write_enable(config);
    if (status != FLASH_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] EraseSector: Write enable failed (%d).", status);
        g_rtos_ops->SemaphoreGive(config->mutex);
        return status;
    }

    // 3. 发送扇区擦除命令和地址
    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_TransferByte(config, FLASH_SectorErase);
    send_address(config, sector_addr);
    config->hw_ops->cs_high(config->hw_context);

    // 4. 等待Flash操作完成
    status = check_busy(config);
    Log_Message(status == FLASH_OK ? LOG_LEVEL_INFO : LOG_LEVEL_ERROR,
                "[FLASH] Erase Sector 0x%08lX %s.", sector_addr, status == FLASH_OK ? "OK" : "FAIL");
    g_rtos_ops->SemaphoreGive(config->mutex);
    return status;
}

/**
 * @brief 擦除整个 Flash 芯片
 * @param dev Flash 设备句柄指针
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_EraseChip(SPI_Flash_Device_t *dev)
{
    if (!dev || !dev->config) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] EraseChip: Invalid device or config pointer.");
        return FLASH_INVALID_PARAM;
    }

    SPI_Flash_Config_t *config = dev->config;

    // 1. 尝试获取互斥锁
    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF)) { // 永远等待
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] EraseChip: Failed to get mutex (timeout).");
        return FLASH_MUTEX_TIMEOUT;
    }

    // 2. 写使能
    Flash_Status_t status = write_enable(config);
    if (status != FLASH_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] EraseChip: Write enable failed (%d).", status);
        g_rtos_ops->SemaphoreGive(config->mutex);
        return status;
    }

    // 3. 发送整片擦除命令
    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_TransferByte(config, FLASH_ChipErase);
    config->hw_ops->cs_high(config->hw_context);

    // 4. 等待Flash操作完成
    status = check_busy(config);
    Log_Message(status == FLASH_OK ? LOG_LEVEL_INFO : LOG_LEVEL_ERROR,
                "[FLASH] Erase Chip %s.", status == FLASH_OK ? "OK" : "FAIL");
    g_rtos_ops->SemaphoreGive(config->mutex);
    return status;
}

/**
 * @brief 写入 Flash 页面
 * @param dev Flash 设备句柄指针
 * @param p_buffer 数据缓冲区
 * @param write_addr 写入地址
 * @param num_byte_to_write 写入字节数 (应 <= PAGE_SIZE)
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_WritePage(SPI_Flash_Device_t *dev, const uint8_t *p_buffer, uint32_t write_addr, uint16_t num_byte_to_write)
{
    if (!dev || !dev->config || !p_buffer || num_byte_to_write == 0 || num_byte_to_write > PAGE_SIZE) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] WritePage: Invalid parameters (dev/config/buffer/length).");
        return FLASH_INVALID_PARAM;
    }
    // 检查地址是否页对齐，如果不是，需要分两次写入或报错
    if (write_addr % PAGE_SIZE + num_byte_to_write > PAGE_SIZE) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] WritePage: Data crosses page boundary. Use SPI_Flash_WriteWithErase instead.");
        return FLASH_INVALID_PARAM;
    }

    SPI_Flash_Config_t *config = dev->config;

    // 1. 尝试获取互斥锁
    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF)) { // 永远等待
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] WritePage: Failed to get mutex (timeout).");
        return FLASH_MUTEX_TIMEOUT;
    }

    // 2. 写使能
    Flash_Status_t status = write_enable(config);
    if (status != FLASH_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] WritePage: Write enable failed (%d).", status);
        g_rtos_ops->SemaphoreGive(config->mutex);
        return status;
    }

    // 3. 发送页编程命令、地址和数据
    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_TransferByte(config, FLASH_PageProgram);
    send_address(config, write_addr);
    for (uint16_t i = 0; i < num_byte_to_write; i++) {
        SPI_Flash_TransferByte(config, p_buffer[i]);
    }
    config->hw_ops->cs_high(config->hw_context);

    // 4. 等待Flash操作完成
    status = check_busy(config);
    Log_Message(status == FLASH_OK ? LOG_LEVEL_INFO : LOG_LEVEL_ERROR,
                "[FLASH] Write Page Addr:0x%08lX, Len:%u %s.", write_addr, num_byte_to_write, status == FLASH_OK ? "OK" : "FAIL");
    g_rtos_ops->SemaphoreGive(config->mutex);
    return status;
}

/**
 * @brief 读取 Flash 数据
 * @param dev Flash 设备句柄指针
 * @param p_buffer 数据缓冲区
 * @param read_addr 读取地址
 * @param num_byte_to_read 读取字节数
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_ReadData(SPI_Flash_Device_t *dev, uint8_t *p_buffer, uint32_t read_addr, uint16_t num_byte_to_read)
{
    if (!dev || !dev->config || !p_buffer || num_byte_to_read == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] ReadData: Invalid parameters (dev/config/buffer/length).");
        return FLASH_INVALID_PARAM;
    }

    SPI_Flash_Config_t *config = dev->config;

    // 1. 尝试获取互斥锁
    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF)) { // 永远等待
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] ReadData: Failed to get mutex (timeout).");
        return FLASH_MUTEX_TIMEOUT;
    }

    // 2. 发送读取命令、地址，并读取数据
    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_TransferByte(config, FLASH_ReadData);
    send_address(config, read_addr);
    for (uint16_t i = 0; i < num_byte_to_read; i++) {
        p_buffer[i] = SPI_Flash_TransferByte(config, 0xFF); // 发送哑字节以读取数据
    }
    config->hw_ops->cs_high(config->hw_context);

    Log_Message(LOG_LEVEL_INFO, "[FLASH] Read Addr:0x%08lX, Len:%u OK.", read_addr, num_byte_to_read);
    g_rtos_ops->SemaphoreGive(config->mutex);
    return FLASH_OK;
}

/**
 * @brief 快速读取 Flash 数据 (带一个哑字节)
 * @param dev Flash 设备句柄指针
 * @param p_buffer 数据缓冲区
 * @param read_addr 读取地址
 * @param num_byte_to_read 读取字节数
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_FastReadData(SPI_Flash_Device_t *dev, uint8_t *p_buffer, uint32_t read_addr, uint16_t num_byte_to_read)
{
    if (!dev || !dev->config || !p_buffer || num_byte_to_read == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] FastReadData: Invalid parameters (dev/config/buffer/length).");
        return FLASH_INVALID_PARAM;
    }

    SPI_Flash_Config_t *config = dev->config;

    // 1. 尝试获取互斥锁
    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF)) { // 永远等待
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] FastReadData: Failed to get mutex (timeout).");
        return FLASH_MUTEX_TIMEOUT;
    }

    // 2. 发送快速读取命令、地址、哑字节，并读取数据
    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_TransferByte(config, FLASH_FastReadData);
    send_address(config, read_addr);
    SPI_Flash_TransferByte(config, 0xFF); // 哑字节
    for (uint16_t i = 0; i < num_byte_to_read; i++) {
        p_buffer[i] = SPI_Flash_TransferByte(config, 0xFF);
    }
    config->hw_ops->cs_high(config->hw_context);

    Log_Message(LOG_LEVEL_INFO, "[FLASH] Fast Read Addr:0x%08lX, Len:%u OK.", read_addr, num_byte_to_read);
    g_rtos_ops->SemaphoreGive(config->mutex);
    return FLASH_OK;
}

/**
 * @brief 读取 Flash 设备 ID (Manufacturer ID + Device ID)
 * @param config Flash 配置结构体指针
 * @return 16位设备ID
 */
uint16_t SPI_Flash_ReadChipId(SPI_Flash_Config_t *config)
{
    if (!config) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] ReadChipId: Invalid config pointer.");
        return 0;
    }

    uint16_t chip_id;
    config->hw_ops->cs_low(config->hw_context);
    SPI_Flash_TransferByte(config, FLASH_ManufactDeviceID); // 发送读取制造商/设备ID命令
    SPI_Flash_TransferByte(config, 0x00); // 3个哑字节
    SPI_Flash_TransferByte(config, 0x00);
    SPI_Flash_TransferByte(config, 0x00);
    chip_id = SPI_Flash_TransferByte(config, 0xFF) << 8; // 读取制造商ID
    chip_id |= SPI_Flash_TransferByte(config, 0xFF);     // 读取设备ID
    config->hw_ops->cs_high(config->hw_context);
    return chip_id;
}

/**
 * @brief 带扇区擦除的 Flash 写入操作
 * @param dev Flash 设备句柄指针
 * @param p_buffer 数据缓冲区
 * @param addr 写入起始地址
 * @param data_len 数据长度
 * @return Flash_Status_t 操作状态
 */
Flash_Status_t SPI_Flash_WriteWithErase(SPI_Flash_Device_t *dev, const uint8_t *p_buffer, uint32_t addr, uint32_t data_len)
{
    if (!dev || !dev->config || !p_buffer || data_len == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] WriteWithErase: Invalid parameters (dev/config/buffer/length).");
        return FLASH_INVALID_PARAM;
    }

    SPI_Flash_Config_t *config = dev->config;

    // 1. 尝试获取互斥锁
    if (!g_rtos_ops->SemaphoreTake(config->mutex, 0xFFFFFFFF)) { // 永远等待
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] WriteWithErase: Failed to get mutex (timeout).");
        return FLASH_MUTEX_TIMEOUT;
    }

    uint32_t current_addr = addr;
    uint32_t bytes_remaining = data_len;
    Flash_Status_t status = FLASH_OK;

    while (bytes_remaining > 0)
    {
        uint32_t sector_start_addr = current_addr / SECTOR_SIZE * SECTOR_SIZE; // 当前地址所在扇区的起始地址
        uint32_t sector_end_addr = sector_start_addr + SECTOR_SIZE - 1;       // 当前扇区的结束地址

        // 计算当前操作在扇区内的起始偏移和长度
        uint32_t write_offset_in_sector = current_addr % SECTOR_SIZE;
        uint32_t write_len_in_sector = (bytes_remaining > (SECTOR_SIZE - write_offset_in_sector)) ?
                                       (SECTOR_SIZE - write_offset_in_sector) : bytes_remaining;

        // 1. 读取整个扇区到 s_sector_buffer
        status = SPI_Flash_ReadData(dev, s_sector_buffer, sector_start_addr, SECTOR_SIZE);
        if (status != FLASH_OK) {
            Log_Message(LOG_LEVEL_ERROR, "[FLASH] WriteWithErase: Failed to read sector 0x%08lX (%d).", sector_start_addr, status);
            break; // 退出循环
        }

        // 2. 将新数据拷贝到 s_sector_buffer 的对应位置
        memcpy(s_sector_buffer + write_offset_in_sector, p_buffer + (current_addr - addr), write_len_in_sector);

        // 3. 擦除扇区
        status = SPI_Flash_EraseSector(dev, sector_start_addr);
        if (status != FLASH_OK) {
            Log_Message(LOG_LEVEL_ERROR, "[FLASH] WriteWithErase: Failed to erase sector 0x%08lX (%d).", sector_start_addr, status);
            break;
        }

        // 4. 将修改后的扇区数据按页写入
        uint32_t page_offset = 0;
        while (page_offset < SECTOR_SIZE)
        {
            uint32_t page_write_addr = sector_start_addr + page_offset;
            uint16_t len_to_write_this_page = (SECTOR_SIZE - page_offset > PAGE_SIZE) ? PAGE_SIZE : (SECTOR_SIZE - page_offset);

            status = SPI_Flash_WritePage(dev, s_sector_buffer + page_offset, page_write_addr, len_to_write_this_page);
            if (status != FLASH_OK) {
                Log_Message(LOG_LEVEL_ERROR, "[FLASH] WriteWithErase: Failed to write page 0x%08lX (%d).", page_write_addr, status);
                break; // 退出内部循环
            }
            page_offset += len_to_write_this_page;
        }

        if (status != FLASH_OK) { // 如果页写入失败，也退出外层循环
            break;
        }

        current_addr += write_len_in_sector;
        bytes_remaining -= write_len_in_sector;
    }

    g_rtos_ops->SemaphoreGive(config->mutex); // 释放互斥锁

    if (status == FLASH_OK) {
        Log_Message(LOG_LEVEL_INFO, "[FLASH] WriteWithErase completed successfully for Addr:0x%08lX, Len:%lu.", addr, data_len);
    } else {
        Log_Message(LOG_LEVEL_ERROR, "[FLASH] WriteWithErase failed for Addr:0x%08lX, Len:%lu. Final status: %d.", addr, data_len, status);
    }
    return status;
}
