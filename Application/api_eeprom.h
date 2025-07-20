/**
 * =====================================================================================
 * @file        api_eeprom.h
 * @brief       通用 AT24Cxx EEPROM 驱动头文件 (Refactored)
 * @author      23Elapse & Gemini
 * @version     2.1 (Refactored)
 * @date        2025-06-14
 * @note        本驱动通过高层I2C总线接口操作，可用于任意I2C总线上的EEPROM。
 * =====================================================================================
 */
#ifndef __API_EEPROM_H
#define __API_EEPROM_H

#include "i2c_driver.h"     // 依赖于重构后的I2C总线驱动
#include "rtos_abstraction.h" // 依赖RTOS抽象层用于延时

// 常用EEPROM型号的容量定义 (字节)
#define AT24C01_CAPACITY    128U
#define AT24C02_CAPACITY    256U
#define AT24C04_CAPACITY    512U
#define AT24C08_CAPACITY    1024U
#define AT24C16_CAPACITY    2048U
#define AT24C32_CAPACITY    4096U
#define AT24C64_CAPACITY    8192U
#define AT24C128_CAPACITY  16384U
#define AT24C256_CAPACITY  32768U
#define AT24C512_CAPACITY  65536U

// EEPROM写操作后需要延时等待内部处理完成 (典型值5ms)
#define EEPROM_WRITE_DELAY_MS 5U

/**
 * @brief EEPROM设备句柄结构体
 * @note  封装了EEPROM所连接的I2C总线、设备地址和其容量（用于地址处理）。
 */
typedef struct {
    I2C_Bus_t* bus;       // 指向EEPROM所连接的I2C总线
    uint8_t    dev_addr;  // 设备的7位I2C地址 (例如 0x50 或根据硬件连接)
    uint16_t   capacity;  // EEPROM的总容量 (字节，例如 AT24C02_CAPACITY)
    uint8_t    page_size; // EEPROM的页大小 (字节，例如 AT24C02是8字节/页，AT24C256是64字节/页)
} EEPROM_Device_t;

/**
 * @brief EEPROM操作状态枚举 (与I2C_Status_t相同，但为EEPROM特定操作命名)
 */
typedef I2C_Status_t EEPROM_Status_t;

/**
 * @brief 初始化EEPROM设备句柄。
 * @param dev 指向要初始化的EEPROM设备句柄。
 * @param bus EEPROM连接的I2C总线句柄。
 * @param dev_addr EEPROM的7位I2C设备地址。
 * @param capacity EEPROM的容量 (例如 AT24C02_CAPACITY)。
 * @param page_size EEPROM的页大小 (例如 8)。
 * @return EEPROM_Status_t 操作状态。
 */
EEPROM_Status_t EEPROM_Device_Init(EEPROM_Device_t* dev, I2C_Bus_t* bus, uint8_t dev_addr, uint16_t capacity, uint8_t page_size);

/**
 * @brief 向EEPROM指定地址写入一个字节。
 * @param dev 指向EEPROM设备句柄。
 * @param mem_addr EEPROM内部的存储地址。
 * @param data 要写入的字节。
 * @return EEPROM_Status_t 通信状态。
 */
EEPROM_Status_t EEPROM_WriteByte(EEPROM_Device_t* dev, uint16_t mem_addr, uint8_t data);

/**
 * @brief 从EEPROM指定地址读取一个字节。
 * @param dev 指向EEPROM设备句柄。
 * @param mem_addr EEPROM内部的存储地址。
 * @param p_data 指向存储读取数据的变量。
 * @return EEPROM_Status_t 通信状态。
 */
EEPROM_Status_t EEPROM_ReadByte(EEPROM_Device_t* dev, uint16_t mem_addr, uint8_t* p_data);

/**
 * @brief 向EEPROM写入一页数据 (优化写入速度)。
 * @param dev 指向EEPROM设备句柄。
 * @param mem_addr EEPROM内部的起始存储地址 (必须是页边界，或页内起始)。
 * @param data 指向要写入的数据缓冲区。
 * @param len 要写入的字节数 (不能超过一页大小)。
 * @return EEPROM_Status_t 通信状态。
 */
EEPROM_Status_t EEPROM_WritePage(EEPROM_Device_t* dev, uint16_t mem_addr, const uint8_t* data, uint16_t len);

/**
 * @brief 从EEPROM连续读取多个字节。
 * @param dev 指向EEPROM设备句柄。
 * @param mem_addr EEPROM内部的起始存储地址。
 * @param p_data 指向存储读取数据的缓冲区。
 * @param len 要读取的字节数。
 * @return EEPROM_Status_t 通信状态。
 */
EEPROM_Status_t EEPROM_ReadBytes(EEPROM_Device_t* dev, uint16_t mem_addr, uint8_t* p_data, uint16_t len);

#endif // __API_EEPROM_H
