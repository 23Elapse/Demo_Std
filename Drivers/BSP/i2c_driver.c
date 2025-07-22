/**
 * =====================================================================================
 * @file        i2c_driver.c
 * @brief       通用的软件I2C总线驱动实现（支持多总线、带RTOS锁）
 * @author      23Elapse & Gemini
 * @version     2.1 (Refactored)
 * @date        2025-06-14
 * =====================================================================================
 */
#include "i2c_driver.h"
#include "common_driver.h" // For Common_Delay_us
#include "log_system.h"
#include "rtos_abstraction.h" // For g_rtos_ops
#include "stm32f4xx_gpio.h" // For GPIO functions

/*
 * =====================================================================================
 * 内部底层函数 (不对外暴露的I2C时序操作)
 * =====================================================================================
 */

// 微秒级延时，决定I2C速度。可根据需要调整。
#define I2C_DELAY_US 5

static void _IIC_Delay(void) {
    delay_us(I2C_DELAY_US);
}

// 设置SCL线电平
static void _IIC_SCL_Set(I2C_Bus_t* bus, uint8_t state) {
    if (state) {
        GPIO_SetBits(bus->scl_port, bus->scl_pin);
    } else {
        GPIO_ResetBits(bus->scl_port, bus->scl_pin);
    }
}

// 设置SDA线电平
static void _IIC_SDA_Set(I2C_Bus_t* bus, uint8_t state) {
    if (state) {
        GPIO_SetBits(bus->sda_port, bus->sda_pin);
    } else {
        GPIO_ResetBits(bus->sda_port, bus->sda_pin);
    }
}

// 读取SDA线电平
static uint8_t _IIC_SDA_Read(I2C_Bus_t* bus) {
    return GPIO_ReadInputDataBit(bus->sda_port, bus->sda_pin);
}

// 配置SDA为输出模式 (推挽输出)
static void _IIC_SDA_Dir_Out(I2C_Bus_t* bus) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = bus->sda_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(bus->sda_port, &GPIO_InitStructure);
}

// 配置SDA为输入模式 (带上拉)
static void _IIC_SDA_Dir_In(I2C_Bus_t* bus) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = bus->sda_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉
    GPIO_Init(bus->sda_port, &GPIO_InitStructure);
}

// I2C起始信号
static void _IIC_Start(I2C_Bus_t* bus) {
    _IIC_SDA_Dir_Out(bus); // SDA设为输出
    _IIC_SDA_Set(bus, 1);  // SDA拉高
    _IIC_SCL_Set(bus, 1);  // SCL拉高
    _IIC_Delay();
    _IIC_SDA_Set(bus, 0);  // SDA拉低
    _IIC_Delay();
    _IIC_SCL_Set(bus, 0);  // SCL拉低
    _IIC_Delay();
}

// I2C停止信号
static void _IIC_Stop(I2C_Bus_t* bus) {
    _IIC_SDA_Dir_Out(bus); // SDA设为输出
    _IIC_SCL_Set(bus, 0);  // SCL拉低
    _IIC_SDA_Set(bus, 0);  // SDA拉低
    _IIC_Delay();
    _IIC_SCL_Set(bus, 1);  // SCL拉高
    _IIC_Delay();
    _IIC_SDA_Set(bus, 1);  // SDA拉高
    _IIC_Delay();
}

// 等待ACK信号
static I2C_Status_t _IIC_WaitAck(I2C_Bus_t* bus) {
    uint16_t timeout_cnt = 0; // 短暂超时计数
    _IIC_SDA_Dir_In(bus);     // SDA设为输入
    _IIC_SDA_Set(bus, 1);     // 主机释放SDA (理论上输入模式下无需设置，但保险起见)
    _IIC_Delay();
    _IIC_SCL_Set(bus, 1);     // SCL拉高，等待从机响应
    _IIC_Delay();
    while (_IIC_SDA_Read(bus)) { // 等待SDA被拉低 (ACK)
        timeout_cnt++;
        if (timeout_cnt > 250) { // 简单超时机制
            _IIC_Stop(bus);      // 超时则发送停止信号
            Log_Message(LOG_LEVEL_WARNING, "[I2C] WaitAck: No ACK received from device.");
            return I2C_ERR_NO_ACK;
        }
    }
    _IIC_SCL_Set(bus, 0);     // SCL拉低
    _IIC_Delay();
    return I2C_OK;
}

// 发送ACK信号
static void _IIC_Ack(I2C_Bus_t* bus) {
    _IIC_SCL_Set(bus, 0);      // SCL拉低
    _IIC_SDA_Dir_Out(bus);     // SDA设为输出
    _IIC_SDA_Set(bus, 0);      // SDA拉低 (ACK)
    _IIC_Delay();
    _IIC_SCL_Set(bus, 1);      // SCL拉高
    _IIC_Delay();
    _IIC_SCL_Set(bus, 0);      // SCL拉低
}

// 发送NACK信号
static void _IIC_NAck(I2C_Bus_t* bus) {
    _IIC_SCL_Set(bus, 0);      // SCL拉低
    _IIC_SDA_Dir_Out(bus);     // SDA设为输出
    _IIC_SDA_Set(bus, 1);      // SDA拉高 (NACK)
    _IIC_Delay();
    _IIC_SCL_Set(bus, 1);      // SCL拉高
    _IIC_Delay();
    _IIC_SCL_Set(bus, 0);      // SCL拉低
}

// I2C发送一个字节
static void _IIC_WriteByte(I2C_Bus_t* bus, uint8_t data) {
    uint8_t i;
    _IIC_SDA_Dir_Out(bus); // SDA设为输出
    _IIC_SCL_Set(bus, 0);  // SCL拉低
    for (i = 0; i < 8; i++) {
        _IIC_SDA_Set(bus, (data & 0x80) >> 7); // 从最高位开始发送
        data <<= 1;
        _IIC_Delay();
        _IIC_SCL_Set(bus, 1); // SCL拉高，数据有效
        _IIC_Delay();
        _IIC_SCL_Set(bus, 0); // SCL拉低，准备下一个位
        _IIC_Delay();
    }
}

// I2C读取一个字节
static uint8_t _IIC_ReadByte(I2C_Bus_t* bus, uint8_t ack) {
    uint8_t i, receive_byte = 0;
    _IIC_SDA_Dir_In(bus); // SDA设为输入
    for (i = 0; i < 8; i++) {
        _IIC_SCL_Set(bus, 0); // SCL拉低
        _IIC_Delay();
        _IIC_SCL_Set(bus, 1); // SCL拉高，从机发送数据
        receive_byte <<= 1;
        if (_IIC_SDA_Read(bus)) {
            receive_byte++; // 读取SDA电平
        }
        _IIC_Delay();
    }
    if (!ack) { // 根据ack参数发送ACK或NACK
        _IIC_NAck(bus);
    } else {
        _IIC_Ack(bus);
    }
    return receive_byte;
}

/*
 * =====================================================================================
 * 高层API函数实现
 * =====================================================================================
 */

/**
 * @brief 初始化I2C总线
 * @param bus 指向I2C_Bus_t结构体的指针
 * @return I2C_Status_t 操作状态
 */
static I2C_Status_t I2C_Bus_Driver_Init(I2C_Bus_t* bus) {
    if (!bus || !bus->scl_port || !bus->sda_port) {
        Log_Message(LOG_LEVEL_ERROR, "[I2C] I2C_Bus_Driver_Init: Invalid bus parameters.");
        return I2C_ERR_PARAM;
    }

    // 初始化SCL和SDA GPIO为推挽输出模式，初始为高电平
    // I2C协议中，总线空闲时SCL和SDA都为高
    if (Common_GPIO_Init(bus->scl_port, bus->scl_pin, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz, 0) != COMMON_OK ||
        Common_GPIO_Init(bus->sda_port, bus->sda_pin, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz, 0) != COMMON_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[I2C] I2C_Bus_Driver_Init: Failed to init GPIOs.");
        return I2C_ERR_INIT_FAILED;
    }

    // 初始化时确保总线空闲状态
    _IIC_SCL_Set(bus, 1);
    _IIC_SDA_Set(bus, 1);
    _IIC_Delay();

    // 如果互斥锁未创建，则创建它
    if (bus->mutex == NULL) {
        bus->mutex = g_rtos_ops->SemaphoreCreate();
        if (bus->mutex == NULL) {
            Log_Message(LOG_LEVEL_ERROR, "[I2C] I2C_Bus_Driver_Init: Failed to create mutex.");
            return I2C_ERR_INIT_FAILED;
        }
    }
    Log_Message(LOG_LEVEL_INFO, "[I2C] I2C bus initialized successfully.");
    return I2C_OK;
}

/**
 * @brief 写入数据到I2C设备
 * @param bus 指向I2C_Bus_t结构体的指针
 * @param dev_addr I2C从设备地址 (7位)
 * @param data 要写入的数据缓冲区
 * @param len 要写入的字节数
 * @return I2C_Status_t 操作状态
 */
static I2C_Status_t I2C_Bus_Driver_Write(I2C_Bus_t* bus, uint8_t dev_addr, const uint8_t* data, uint16_t len) {
    if (!bus || !data || len == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[I2C] I2C_Bus_Driver_Write: Invalid parameters.");
        return I2C_ERR_PARAM;
    }
    // 尝试获取互斥锁以保护总线访问
    if (!g_rtos_ops->SemaphoreTake(bus->mutex, 100)) { // 100ms 超时
        Log_Message(LOG_LEVEL_WARNING, "[I2C] I2C_Bus_Driver_Write: Failed to get mutex (timeout).");
        return I2C_ERR_TIMEOUT;
    }

    _IIC_Start(bus);
    // 发送设备地址和写方向位 (0)
    _IIC_WriteByte(bus, (dev_addr << 1) | 0);
    if (_IIC_WaitAck(bus) != I2C_OK) {
        _IIC_Stop(bus);
        g_rtos_ops->SemaphoreGive(bus->mutex);
        Log_Message(LOG_LEVEL_WARNING, "[I2C] I2C_Bus_Driver_Write: No ACK after sending device address.");
        return I2C_ERR_NO_ACK;
    }

    for (uint16_t i = 0; i < len; i++) {
        _IIC_WriteByte(bus, data[i]);
        if (_IIC_WaitAck(bus) != I2C_OK) {
            _IIC_Stop(bus);
            g_rtos_ops->SemaphoreGive(bus->mutex);
            Log_Message(LOG_LEVEL_WARNING, "[I2C] I2C_Bus_Driver_Write: No ACK after sending data byte %u.", i);
            return I2C_ERR_NO_ACK;
        }
    }

    _IIC_Stop(bus);
    g_rtos_ops->SemaphoreGive(bus->mutex);
    Log_Message(LOG_LEVEL_DEBUG, "[I2C] Write to 0x%02X, len %u OK.", dev_addr, len);
    return I2C_OK;
}

/**
 * @brief 从I2C设备读取数据
 * @param bus 指向I2C_Bus_t结构体的指针
 * @param dev_addr I2C从设备地址 (7位)
 * @param data 接收数据缓冲区
 * @param len 要读取的字节数
 * @return I2C_Status_t 操作状态
 */
static I2C_Status_t I2C_Bus_Driver_Read(I2C_Bus_t* bus, uint8_t dev_addr, uint8_t* data, uint16_t len) {
    if (!bus || !data || len == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[I2C] I2C_Bus_Driver_Read: Invalid parameters.");
        return I2C_ERR_PARAM;
    }
    // 尝试获取互斥锁以保护总线访问
    if (!g_rtos_ops->SemaphoreTake(bus->mutex, 100)) {
        Log_Message(LOG_LEVEL_WARNING, "[I2C] I2C_Bus_Driver_Read: Failed to get mutex (timeout).");
        return I2C_ERR_TIMEOUT;
    }

    _IIC_Start(bus);
    // 发送设备地址和读方向位 (1)
    _IIC_WriteByte(bus, (dev_addr << 1) | 1);
    if (_IIC_WaitAck(bus) != I2C_OK) {
        _IIC_Stop(bus);
        g_rtos_ops->SemaphoreGive(bus->mutex);
        Log_Message(LOG_LEVEL_WARNING, "[I2C] I2C_Bus_Driver_Read: No ACK after sending device address for read.");
        return I2C_ERR_NO_ACK;
    }

    for (uint16_t i = 0; i < len; i++) {
        // 读取最后一个字节时发送NACK，其他字节发送ACK
        data[i] = _IIC_ReadByte(bus, (i == (len - 1)) ? 0 : 1);
    }

    _IIC_Stop(bus);
    g_rtos_ops->SemaphoreGive(bus->mutex);
    Log_Message(LOG_LEVEL_DEBUG, "[I2C] Read from 0x%02X, len %u OK.", dev_addr, len);
    return I2C_OK;
}

/**
 * @brief I2C复合操作：先写后读
 * @param bus 指向I2C_Bus_t结构体的指针
 * @param dev_addr I2C从设备地址 (7位)
 * @param w_data 要写入的数据缓冲区
 * @param w_len 要写入的字节数
 * @param r_data 接收数据缓冲区
 * @param r_len 要读取的字节数
 * @return I2C_Status_t 操作状态
 */
static I2C_Status_t I2C_Bus_Driver_WriteThenRead(I2C_Bus_t* bus, uint8_t dev_addr, const uint8_t* w_data, uint16_t w_len, uint8_t* r_data, uint16_t r_len) {
    I2C_Status_t status;
    if (!bus || !w_data || !r_data || w_len == 0 || r_len == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[I2C] I2C_Bus_Driver_WriteThenRead: Invalid parameters.");
        return I2C_ERR_PARAM;
    }
    // 尝试获取互斥锁以保护总线访问
    if (!g_rtos_ops->SemaphoreTake(bus->mutex, 100)) {
        Log_Message(LOG_LEVEL_WARNING, "[I2C] I2C_Bus_Driver_WriteThenRead: Failed to get mutex (timeout).");
        return I2C_ERR_TIMEOUT;
    }

    // 1. 写操作
    _IIC_Start(bus);
    _IIC_WriteByte(bus, (dev_addr << 1) | 0); // 发送写命令
    status = _IIC_WaitAck(bus);
    if (status != I2C_OK) {
        _IIC_Stop(bus);
        g_rtos_ops->SemaphoreGive(bus->mutex);
        Log_Message(LOG_LEVEL_WARNING, "[I2C] WriteThenRead: No ACK after sending device address for write.");
        return status;
    }
    for (uint16_t i = 0; i < w_len; i++) {
        _IIC_WriteByte(bus, w_data[i]);
        status = _IIC_WaitAck(bus);
        if (status != I2C_OK) {
            _IIC_Stop(bus);
            g_rtos_ops->SemaphoreGive(bus->mutex);
            Log_Message(LOG_LEVEL_WARNING, "[I2C] WriteThenRead: No ACK after sending write data byte %u.", i);
            return status;
        }
    }

    // 2. 读操作 (重复起始信号)
    _IIC_Start(bus); // 发送重复起始信号
    _IIC_WriteByte(bus, (dev_addr << 1) | 1); // 发送读命令
    status = _IIC_WaitAck(bus);
    if (status != I2C_OK) {
        _IIC_Stop(bus);
        g_rtos_ops->SemaphoreGive(bus->mutex);
        Log_Message(LOG_LEVEL_WARNING, "[I2C] WriteThenRead: No ACK after sending device address for read (repeated start).");
        return status;
    }
    for (uint16_t i = 0; i < r_len; i++) {
        r_data[i] = _IIC_ReadByte(bus, (i == (r_len - 1)) ? 0 : 1); // 最后字节发NACK，其他发ACK
    }

    _IIC_Stop(bus);
    g_rtos_ops->SemaphoreGive(bus->mutex);
    Log_Message(LOG_LEVEL_DEBUG, "[I2C] WriteThenRead to/from 0x%02X, W_len %u, R_len %u OK.", dev_addr, w_len, r_len);
    return I2C_OK;
}

/*
 * =====================================================================================
 * 全局操作接口实例定义
 * =====================================================================================
 */
const I2C_Bus_Ops_t g_i2c_bus_ops = {
    .Init = I2C_Bus_Driver_Init,
    .Write = I2C_Bus_Driver_Write,
    .Read = I2C_Bus_Driver_Read,
    .WriteThenRead = I2C_Bus_Driver_WriteThenRead,
};
