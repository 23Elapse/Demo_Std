/**
 * =====================================================================================
 * @file        api_eeprom.c
 * @brief       通用 AT24Cxx EEPROM 驱动实现 (已解耦)
 * @author      23Elapse & Gemini
 * @version     2.0 (Refactored)
 * @date        2025-06-08
 * =====================================================================================
 */
#include "api_eeprom.h"
#include "log_system.h"
#include <string.h>

// EEPROM写操作后需要延时等待内部处理完成
#define EEPROM_WRITE_DELAY_MS 5

// 辅助函数，用于处理不同容量EEPROM的地址发送方式
static I2C_Status_t EEPROM_SendAddress(I2C_Bus_t* bus, uint8_t dev_addr, uint16_t mem_addr) {
    uint8_t addr_buf[2];
    uint8_t len = 0;

    // 对于大于2Kbit (256字节)的EEPROM，需要发送2个字节的地址
    if (AT24C02_CAPACITY < 2048) { // AT24C02, AT24C04, AT24C08, AT24C16
        // 地址位被编码到设备地址中
        dev_addr |= (uint8_t)((mem_addr / 256) << 1);
        addr_buf[0] = (uint8_t)(mem_addr & 0xFF);
        len = 1;
    } else { // AT24C32, AT24C64...
        addr_buf[0] = (uint8_t)(mem_addr >> 8);   // 高地址
        addr_buf[1] = (uint8_t)(mem_addr & 0xFF); // 低地址
        len = 2;
    }
    
    // 调用高层I2C接口写入地址
    return g_i2c_bus_ops.Write(bus, dev_addr, addr_buf, len);
}


I2C_Status_t EEPROM_WriteByte(I2C_Bus_t* bus, uint8_t dev_addr, uint16_t mem_addr, uint8_t data) {
    uint8_t write_buf[3];
    uint8_t len = 0;

    // 构造写入序列: [MemAddr_High], [MemAddr_Low], [Data]
    if (AT24C02_CAPACITY >= 2048) {
        write_buf[len++] = (uint8_t)(mem_addr >> 8);
    }
    write_buf[len++] = (uint8_t)(mem_addr & 0xFF);
    write_buf[len++] = data;

    I2C_Status_t status = g_i2c_bus_ops.Write(bus, dev_addr, write_buf, len);
    if (status == I2C_OK) {
        // 写操作后必须延时等待
        g_rtos_ops->Delay(EEPROM_WRITE_DELAY_MS);
    }
    return status;
}

I2C_Status_t EEPROM_ReadByte(I2C_Bus_t* bus, uint8_t dev_addr, uint16_t mem_addr, uint8_t* p_data) {
    uint8_t addr_buf[2];
    uint8_t len = 0;

    // 构造地址序列
    if (AT24C02_CAPACITY >= 2048) {
        addr_buf[len++] = (uint8_t)(mem_addr >> 8);
    }
    addr_buf[len++] = (uint8_t)(mem_addr & 0xFF);

    // 使用I2C的复合读写操作：先写入要读取的地址，然后不释放总线，再读取数据
    return g_i2c_bus_ops.WriteAndRead(bus, dev_addr, addr_buf, len, p_data, 1);
}

I2C_Status_t EEPROM_WritePage(I2C_Bus_t* bus, uint8_t dev_addr, uint16_t mem_addr, const uint8_t* data, uint16_t len) {
    uint8_t write_buf[256 + 2]; // 最大页大小 + 2字节地址
    uint8_t addr_len = 0;

    // 构造地址
    if (AT24C02_CAPACITY >= 2048) {
        write_buf[addr_len++] = (uint8_t)(mem_addr >> 8);
    }
    write_buf[addr_len++] = (uint8_t)(mem_addr & 0xFF);
    
    // 拷贝数据
    memcpy(write_buf + addr_len, data, len);

    I2C_Status_t status = g_i2c_bus_ops.Write(bus, dev_addr, write_buf, len + addr_len);
    if (status == I2C_OK) {
        g_rtos_ops->Delay(EEPROM_WRITE_DELAY_MS);
    }
    return status;
}

I2C_Status_t EEPROM_ReadBytes(I2C_Bus_t* bus, uint8_t dev_addr, uint16_t mem_addr, uint8_t* p_data, uint16_t len) {
     uint8_t addr_buf[2];
    uint8_t addr_len = 0;

    // 构造地址
    if (AT24C02_CAPACITY >= 2048) {
        addr_buf[addr_len++] = (uint8_t)(mem_addr >> 8);
    }
    addr_buf[addr_len++] = (uint8_t)(mem_addr & 0xFF);
    
    return g_i2c_bus_ops.WriteAndRead(bus, dev_addr, addr_buf, addr_len, p_data, len);
}
