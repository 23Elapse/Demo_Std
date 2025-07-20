/**
 * =====================================================================================
 * @file        api_eeprom.c
 * @brief       通用 AT24Cxx EEPROM 驱动实现 (Refactored)
 * @author      23Elapse & Gemini
 * @version     2.1 (Refactored)
 * @date        2025-06-14
 * @note        本驱动通过高层I2C总线接口操作，可用于任意I2C总线上的EEPROM。
 * =====================================================================================
 */
#include "api_eeprom.h"
#include "log_system.h"
#include <string.h> // For memcpy

// 注意：EEPROM_WRITE_DELAY_MS 宏已移至 api_eeprom.h

/**
 * @brief 内部辅助函数：根据EEPROM容量和内存地址构造并发送地址字节。
 * @param dev 指向EEPROM设备句柄。
 * @param mem_addr EEPROM内部的存储地址。
 * @param addr_buf 存储地址字节的缓冲区。
 * @return 构造的地址字节长度。
 */
static uint8_t _EEPROM_GetAddressBytes(EEPROM_Device_t* dev, uint16_t mem_addr, uint8_t* addr_buf) {
    uint8_t len = 0;
    // 对于容量 >= 2048bit (256字节) 的EEPROM (如 AT24C32 及以上)，需要发送2个字节的地址
    if (dev->capacity >= AT24C32_CAPACITY) { // 检查是否需要2字节地址
        addr_buf[len++] = (uint8_t)(mem_addr >> 8);   // 高地址字节
    }
    addr_buf[len++] = (uint8_t)(mem_addr & 0xFF); // 低地址字节
    return len;
}

/**
 * @brief 初始化EEPROM设备句柄。
 * @param dev 指向要初始化的EEPROM设备句柄。
 * @param bus EEPROM连接的I2C总线句柄。
 * @param dev_addr EEPROM的7位I2C设备地址。
 * @param capacity EEPROM的容量 (例如 AT24C02_CAPACITY)。
 * @param page_size EEPROM的页大小 (例如 8)。
 * @return EEPROM_Status_t 操作状态。
 */
EEPROM_Status_t EEPROM_Device_Init(EEPROM_Device_t* dev, I2C_Bus_t* bus, uint8_t dev_addr, uint16_t capacity, uint8_t page_size) {
    if (!dev || !bus || capacity == 0 || page_size == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Device Init: Invalid parameters.");
        return I2C_ERR_PARAM;
    }
    dev->bus = bus;
    dev->dev_addr = dev_addr;
    dev->capacity = capacity;
    dev->page_size = page_size;

    // 可以在这里进行一次读写测试来验证EEPROM是否在线和功能正常
    uint8_t test_data = 0xAA;
    uint8_t read_back = 0x00;
    EEPROM_Status_t status;

    // 写入一个字节进行测试
    status = EEPROM_WriteByte(dev, 0x0000, test_data); // 写入地址0
    if (status != I2C_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Device Init: Write test failed (Addr 0x%02X). Status: %d", dev_addr, status);
        return status;
    }
    // 读取回来验证
    status = EEPROM_ReadByte(dev, 0x0000, &read_back);
    if (status != I2C_OK || read_back != test_data) {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Device Init: Read test failed or data mismatch (Addr 0x%02X, Read: 0x%02X). Status: %d", dev_addr, read_back, status);
        return (status != I2C_OK) ? status : I2C_ERR_UNKNOWN; // 如果数据不匹配也视为错误
    }

    Log_Message(LOG_LEVEL_INFO, "[EEPROM] Device 0x%02X (Capacity: %u bytes, Page: %u bytes) initialized successfully.", dev_addr, capacity, page_size);
    return I2C_OK;
}

/**
 * @brief 向EEPROM指定地址写入一个字节。
 * @param dev 指向EEPROM设备句柄。
 * @param mem_addr EEPROM内部的存储地址。
 * @param data 要写入的字节。
 * @return EEPROM_Status_t 通信状态。
 */
EEPROM_Status_t EEPROM_WriteByte(EEPROM_Device_t* dev, uint16_t mem_addr, uint8_t data) {
    if (!dev || !dev->bus || mem_addr >= dev->capacity) {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] WriteByte: Invalid parameters (mem_addr: 0x%04X, capacity: %u).", mem_addr, dev->capacity);
        return I2C_ERR_PARAM;
    }

    uint8_t write_buf[3]; // 最大2字节地址 + 1字节数据
    uint8_t len = _EEPROM_GetAddressBytes(dev, mem_addr, write_buf);
    write_buf[len++] = data;

    I2C_Status_t status = g_i2c_bus_ops.Write(dev->bus, dev->dev_addr, write_buf, len);
    if (status == I2C_OK) {
        g_rtos_ops->Delay(EEPROM_WRITE_DELAY_MS); // 写入后等待内部操作完成
        Log_Message(LOG_LEVEL_DEBUG, "[EEPROM] WriteByte: Addr 0x%04X, Data 0x%02X OK.", mem_addr, data);
    } else {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] WriteByte: Failed for Addr 0x%04X, Data 0x%02X. Status: %d.", mem_addr, data, status);
    }
    return status;
}

/**
 * @brief 从EEPROM指定地址读取一个字节。
 * @param dev 指向EEPROM设备句柄。
 * @param mem_addr EEPROM内部的存储地址。
 * @param p_data 指向存储读取数据的变量。
 * @return EEPROM_Status_t 通信状态。
 */
EEPROM_Status_t EEPROM_ReadByte(EEPROM_Device_t* dev, uint16_t mem_addr, uint8_t* p_data) {
    if (!dev || !dev->bus || !p_data || mem_addr >= dev->capacity) {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] ReadByte: Invalid parameters (mem_addr: 0x%04X, capacity: %u).", mem_addr, dev->capacity);
        return I2C_ERR_PARAM;
    }

    uint8_t addr_buf[2]; // 最大2字节地址
    uint8_t addr_len = _EEPROM_GetAddressBytes(dev, mem_addr, addr_buf);

    // 使用I2C的复合读写操作：先写入要读取的地址，然后不释放总线，再读取数据
    I2C_Status_t status = g_i2c_bus_ops.WriteThenRead(dev->bus, dev->dev_addr, addr_buf, addr_len, p_data, 1);
    if (status == I2C_OK) {
        Log_Message(LOG_LEVEL_DEBUG, "[EEPROM] ReadByte: Addr 0x%04X, Data 0x%02X OK.", mem_addr, *p_data);
    } else {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] ReadByte: Failed for Addr 0x%04X. Status: %d.", mem_addr, status);
    }
    return status;
}

/**
 * @brief 向EEPROM写入一页数据 (优化写入速度)。
 * @param dev 指向EEPROM设备句柄。
 * @param mem_addr EEPROM内部的起始存储地址 (必须是页边界，或页内起始)。
 * @param data 指向要写入的数据缓冲区。
 * @param len 要写入的字节数 (不能超过一页大小)。
 * @return EEPROM_Status_t 通信状态。
 */
EEPROM_Status_t EEPROM_WritePage(EEPROM_Device_t* dev, uint16_t mem_addr, const uint8_t* data, uint16_t len) {
    if (!dev || !dev->bus || !data || len == 0 || len > dev->page_size || mem_addr >= dev->capacity) {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] WritePage: Invalid parameters (len: %u, page_size: %u, mem_addr: 0x%04X, capacity: %u).",
                    len, dev->page_size, mem_addr, dev->capacity);
        return I2C_ERR_PARAM;
    }
    // 检查写入长度是否会跨页 (如果mem_addr不是页边界)
    if ((mem_addr % dev->page_size) + len > dev->page_size) {
        Log_Message(LOG_LEVEL_WARNING, "[EEPROM] WritePage: Data crosses page boundary (Addr: 0x%04X, Len: %u). Consider multiple writes.", mem_addr, len);
        return I2C_ERR_PARAM; // 拒绝跨页写入，需要调用者分拆
    }

    uint8_t write_buf[2 + 64]; // 最大2字节地址 + 最大页大小64字节 (AT24C256)
    uint8_t addr_len = _EEPROM_GetAddressBytes(dev, mem_addr, write_buf);
    
    memcpy(write_buf + addr_len, data, len); // 拷贝数据到发送缓冲区

    I2C_Status_t status = g_i2c_bus_ops.Write(dev->bus, dev->dev_addr, write_buf, len + addr_len);
    if (status == I2C_OK) {
        g_rtos_ops->Delay(EEPROM_WRITE_DELAY_MS); // 写入后等待内部操作完成
        Log_Message(LOG_LEVEL_DEBUG, "[EEPROM] WritePage: Addr 0x%04X, Len %u OK.", mem_addr, len);
    } else {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] WritePage: Failed for Addr 0x%04X, Len %u. Status: %d.", mem_addr, len, status);
    }
    return status;
}

/**
 * @brief 从EEPROM连续读取多个字节。
 * @param dev 指向EEPROM设备句柄。
 * @param mem_addr EEPROM内部的起始存储地址。
 * @param p_data 指向存储读取数据的缓冲区。
 * @param len 要读取的字节数。
 * @return EEPROM_Status_t 通信状态。
 */
EEPROM_Status_t EEPROM_ReadBytes(EEPROM_Device_t* dev, uint16_t mem_addr, uint8_t* p_data, uint16_t len) {
    if (!dev || !dev->bus || !p_data || len == 0 || mem_addr >= dev->capacity || (mem_addr + len) > dev->capacity) {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] ReadBytes: Invalid parameters (mem_addr: 0x%04X, len: %u, capacity: %u).", mem_addr, len, dev->capacity);
        return I2C_ERR_PARAM;
    }

    uint8_t addr_buf[2]; // 最大2字节地址
    uint8_t addr_len = _EEPROM_GetAddressBytes(dev, mem_addr, addr_buf);
    
    // 使用I2C的复合读写操作：先写入要读取的地址，然后不释放总线，再读取数据
    I2C_Status_t status = g_i2c_bus_ops.WriteThenRead(dev->bus, dev->dev_addr, addr_buf, addr_len, p_data, len);
    if (status == I2C_OK) {
        Log_Message(LOG_LEVEL_DEBUG, "[EEPROM] ReadBytes: Addr 0x%04X, Len %u OK.", mem_addr, len);
    } else {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] ReadBytes: Failed for Addr 0x%04X, Len %u. Status: %d.", mem_addr, len, status);
    }
    return status;
}
