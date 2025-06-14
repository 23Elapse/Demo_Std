/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-05 21:24:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-08 17:19:24
 * @FilePath: \Demo_backup\Drivers\BSP\pcf8574.c
 * @Description: PCF8574 IIC 扩展 IO 驱动实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "pcf8574.h"
// /* PCF8574 的 I2C 操作接口 */
// const IIC_Ops_t IIC1_PCF8574 = {
//     .dev_addr = PCF8574_ADDR,
//     .ReadByte = PCF8574_ReadBit,
//     .WriteByte = PCF8574_WriteBit,
// };

I2C_Status_t PCF8574_Init(I2C_Bus_t* bus, uint8_t dev_addr) {
    // 初始化时可以尝试读取一次，确保设备在线
    uint8_t temp_data;
    return PCF8574_ReadByte(bus, dev_addr, &temp_data);
}

I2C_Status_t PCF8574_WriteByte(I2C_Bus_t* bus, uint8_t dev_addr, uint8_t data) {
    // 直接调用高层总线API，不再关心Start/Stop等细节
    return g_i2c_bus_ops.Write(bus, dev_addr, &data, 1);
}

I2C_Status_t PCF8574_ReadByte(I2C_Bus_t* bus, uint8_t dev_addr, uint8_t* p_data) {
    return g_i2c_bus_ops.Read(bus, dev_addr, p_data, 1);
}