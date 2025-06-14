/**
 * =====================================================================================
 * @file        api_eeprom.h
 * @brief       通用 AT24Cxx EEPROM 驱动头文件 (已解耦)
 * @author      23Elapse & Gemini
 * @version     2.0 (Refactored)
 * @date        2025-06-08
 * @note        本驱动通过高层I2C总线接口操作，可用于任意I2C总线上的EEPROM。
 * =====================================================================================
 */
#ifndef __API_EEPROM_H
#define __API_EEPROM_H

#include "i2c_driver.h" // 依赖于重构后的I2C总线驱动

// 常用EEPROM型号定义 (用于地址计算)
#define AT24C01_CAPACITY    128
#define AT24C02_CAPACITY    256
#define AT24C04_CAPACITY    512
#define AT24C08_CAPACITY    1024
#define AT24C16_CAPACITY    2048
#define AT24C32_CAPACITY    4096
#define AT24C64_CAPACITY    8192

/**
 * @brief  向EEPROM指定地址写入一个字节
 * @param  bus 指向EEPROM所连接的I2C总线
 * @param  dev_addr 设备的7位I2C地址
 * @param  mem_addr EEPROM内部的存储地址
 * @param  data 要写入的字节
 * @return I2C_Status_t 通信状态
 */
I2C_Status_t EEPROM_WriteByte(I2C_Bus_t* bus, uint8_t dev_addr, uint16_t mem_addr, uint8_t data);

/**
 * @brief  从EEPROM指定地址读取一个字节
 * @param  bus 指向EEPROM所连接的I2C总线
 * @param  dev_addr 设备的7位I2C地址
 * @param  mem_addr EEPROM内部的存储地址
 * @param  p_data 指向存储读取数据的变量
 * @return I2C_Status_t 通信状态
 */
I2C_Status_t EEPROM_ReadByte(I2C_Bus_t* bus, uint8_t dev_addr, uint16_t mem_addr, uint8_t* p_data);

/**
 * @brief  向EEPROM写入一页数据 (优化写入速度)
 * @param  bus 指向EEPROM所连接的I2C总线
 * @param  dev_addr 设备的7位I2C地址
 * @param  mem_addr EEPROM内部的起始存储地址 (必须是页边界)
 * @param  data 指向要写入的数据缓冲区
 * @param  len 要写入的字节数 (不能超过一页大小)
 * @return I2C_Status_t 通信状态
 */
I2C_Status_t EEPROM_WritePage(I2C_Bus_t* bus, uint8_t dev_addr, uint16_t mem_addr, const uint8_t* data, uint16_t len);

/**
 * @brief  从EEPROM连续读取多个字节
 * @param  bus 指向EEPROM所连接的I2C总线
 * @param  dev_addr 设备的7位I2C地址
 * @param  mem_addr EEPROM内部的起始存储地址
 * @param  p_data 指向存储读取数据的缓冲区
 * @param  len 要读取的字节数
 * @return I2C_Status_t 通信状态
 */
I2C_Status_t EEPROM_ReadBytes(I2C_Bus_t* bus, uint8_t dev_addr, uint16_t mem_addr, uint8_t* p_data, uint16_t len);

#endif // __API_EEPROM_H
