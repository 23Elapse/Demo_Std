/**
 * =====================================================================================
 * @file        i2c_driver.c
 * @brief       通用的软件I2C总线驱动实现（支持多总线、带RTOS锁）
 * @author      23Elapse & Gemini
 * @version     2.0 (Refactored)
 * @date        2025-06-08
 * =====================================================================================
 */
#include "i2c_driver.h"
#include "common_driver.h" // For Common_Delay_us
#include "log_system.h"
#include "rtos_abstraction.h" // For g_rtos_ops
#include "stm32f4xx_gpio.h" // For GPIO functions

/*
 * =====================================================================================
 * 内部底层函数 (不对外暴露)
 * =====================================================================================
 */

// 微秒级延时，决定I2C速度。可根据需要调整。
#define I2C_DELAY_US 5

static void IIC_Delay(void) {
    delay_us(I2C_DELAY_US);
}

// 设置SCL线电平
static void IIC_SCL_Set(I2C_Bus_t* bus, uint8_t state) {
    if (state) {
        GPIO_SetBits(bus->scl_port, bus->scl_pin);
    } else {
        GPIO_ResetBits(bus->scl_port, bus->scl_pin);
    }
}

// 设置SDA线电平
static void IIC_SDA_Set(I2C_Bus_t* bus, uint8_t state) {
    if (state) {
        GPIO_SetBits(bus->sda_port, bus->sda_pin);
    } else {
        GPIO_ResetBits(bus->sda_port, bus->sda_pin);
    }
}

// 读取SDA线电平
static uint8_t IIC_SDA_Read(I2C_Bus_t* bus) {
    return GPIO_ReadInputDataBit(bus->sda_port, bus->sda_pin);
}

// 配置SDA为输出模式
static void IIC_SDA_Dir_Out(I2C_Bus_t* bus) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = bus->sda_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(bus->sda_port, &GPIO_InitStructure);
}

// 配置SDA为输入模式
static void IIC_SDA_Dir_In(I2C_Bus_t* bus) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = bus->sda_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(bus->sda_port, &GPIO_InitStructure);
}

// I2C起始信号
static void IIC_Start(I2C_Bus_t* bus) {
    IIC_SDA_Dir_Out(bus);
    IIC_SDA_Set(bus, 1);
    IIC_SCL_Set(bus, 1);
    IIC_Delay();
    IIC_SDA_Set(bus, 0);
    IIC_Delay();
    IIC_SCL_Set(bus, 0);
    IIC_Delay();
}

// I2C停止信号
static void IIC_Stop(I2C_Bus_t* bus) {
    IIC_SDA_Dir_Out(bus);
    IIC_SCL_Set(bus, 0);
    IIC_SDA_Set(bus, 0);
    IIC_Delay();
    IIC_SCL_Set(bus, 1);
    IIC_Delay();
    IIC_SDA_Set(bus, 1);
    IIC_Delay();
}

// 等待ACK信号
static I2C_Status_t IIC_WaitAck(I2C_Bus_t* bus) {
    uint8_t timeout = 0;
    IIC_SDA_Dir_In(bus);
    IIC_SDA_Set(bus, 1); // 主机释放SDA
    IIC_Delay();
    IIC_SCL_Set(bus, 1);
    IIC_Delay();
    while (IIC_SDA_Read(bus)) {
        timeout++;
        if (timeout > 250) {
            IIC_Stop(bus);
            return I2C_ERR_NO_ACK;
        }
    }
    IIC_SCL_Set(bus, 0);
    IIC_Delay();
    return I2C_OK;
}

// 发送ACK信号
static void IIC_Ack(I2C_Bus_t* bus) {
    IIC_SCL_Set(bus, 0);
    IIC_SDA_Dir_Out(bus);
    IIC_SDA_Set(bus, 0);
    IIC_Delay();
    IIC_SCL_Set(bus, 1);
    IIC_Delay();
    IIC_SCL_Set(bus, 0);
}

// 发送NACK信号
static void IIC_NAck(I2C_Bus_t* bus) {
    IIC_SCL_Set(bus, 0);
    IIC_SDA_Dir_Out(bus);
    IIC_SDA_Set(bus, 1);
    IIC_Delay();
    IIC_SCL_Set(bus, 1);
    IIC_Delay();
    IIC_SCL_Set(bus, 0);
}

// I2C发送一个字节
static void IIC_WriteByte(I2C_Bus_t* bus, uint8_t data) {
    uint8_t i;
    IIC_SDA_Dir_Out(bus);
    IIC_SCL_Set(bus, 0);
    for (i = 0; i < 8; i++) {
        IIC_SDA_Set(bus, (data & 0x80) >> 7);
        data <<= 1;
        IIC_Delay();
        IIC_SCL_Set(bus, 1);
        IIC_Delay();
        IIC_SCL_Set(bus, 0);
        IIC_Delay();
    }
}

// I2C读取一个字节
static uint8_t IIC_ReadByte(I2C_Bus_t* bus, uint8_t ack) {
    uint8_t i, receive = 0;
    IIC_SDA_Dir_In(bus);
    for (i = 0; i < 8; i++) {
        IIC_SCL_Set(bus, 0);
        IIC_Delay();
        IIC_SCL_Set(bus, 1);
        receive <<= 1;
        if (IIC_SDA_Read(bus)) {
            receive++;
        }
        IIC_Delay();
    }
    if (!ack) {
        IIC_NAck(bus);
    } else {
        IIC_Ack(bus);
    }
    return receive;
}

/*
 * =====================================================================================
 * 高层API函数实现
 * =====================================================================================
 */

static I2C_Status_t I2C_Bus_Init(I2C_Bus_t* bus) {
    if (!bus || !bus->scl_port || !bus->sda_port) {
        return I2C_ERR_PARAM;
    }

    // 初始化GPIO
    Common_GPIO_Init(bus->scl_port, bus->scl_pin, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz, 0);
    Common_GPIO_Init(bus->sda_port, bus->sda_pin, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz, 0);

    // 初始化时确保总线空闲
    IIC_SCL_Set(bus, 1);
    IIC_SDA_Set(bus, 1);

    // 如果未创建Mutex，则创建
    if (bus->mutex == NULL) {
        bus->mutex = g_rtos_ops->SemaphoreCreate();
        if (bus->mutex == NULL) {
            Log_Message(LOG_LEVEL_ERROR, "Error: I2C bus mutex creation failed.");
            return I2C_ERR_UNKNOWN;
        }
    }
    return I2C_OK;
}

static I2C_Status_t I2C_Bus_Write(I2C_Bus_t* bus, uint8_t dev_addr, const uint8_t* data, uint16_t len) {
    if (!bus || !data) return I2C_ERR_PARAM;
    if (!g_rtos_ops->SemaphoreTake(bus->mutex, 100)) return I2C_ERR_TIMEOUT;

    IIC_Start(bus);
    IIC_WriteByte(bus, (dev_addr << 1) | 0); // 发送写命令
    if (IIC_WaitAck(bus) != I2C_OK) {
        g_rtos_ops->SemaphoreGive(bus->mutex);
        return I2C_ERR_NO_ACK;
    }

    for (uint16_t i = 0; i < len; i++) {
        IIC_WriteByte(bus, data[i]);
        if (IIC_WaitAck(bus) != I2C_OK) {
            g_rtos_ops->SemaphoreGive(bus->mutex);
            return I2C_ERR_NO_ACK;
        }
    }

    IIC_Stop(bus);
    g_rtos_ops->SemaphoreGive(bus->mutex);
    return I2C_OK;
}

static I2C_Status_t I2C_Bus_Read(I2C_Bus_t* bus, uint8_t dev_addr, uint8_t* data, uint16_t len) {
    if (!bus || !data) return I2C_ERR_PARAM;
    if (!g_rtos_ops->SemaphoreTake(bus->mutex, 100)) return I2C_ERR_TIMEOUT;
    
    IIC_Start(bus);
    IIC_WriteByte(bus, (dev_addr << 1) | 1); // 发送读命令
    if (IIC_WaitAck(bus) != I2C_OK) {
        g_rtos_ops->SemaphoreGive(bus->mutex);
        return I2C_ERR_NO_ACK;
    }

    for (uint16_t i = 0; i < len; i++) {
        data[i] = IIC_ReadByte(bus, (i == (len - 1)) ? 0 : 1); // 最后字节发NACK
    }

    IIC_Stop(bus);
    g_rtos_ops->SemaphoreGive(bus->mutex);
    return I2C_OK;
}

static I2C_Status_t I2C_Bus_WriteAndRead(I2C_Bus_t* bus, uint8_t dev_addr, const uint8_t* w_data, uint16_t w_len, uint8_t* r_data, uint16_t r_len) {
    I2C_Status_t status;
    if (!bus || !w_data || !r_data) return I2C_ERR_PARAM;
    if (!g_rtos_ops->SemaphoreTake(bus->mutex, 100)) return I2C_ERR_TIMEOUT;

    // 先执行写操作
    IIC_Start(bus);
    IIC_WriteByte(bus, (dev_addr << 1) | 0);
    if (IIC_WaitAck(bus) != I2C_OK) {
        g_rtos_ops->SemaphoreGive(bus->mutex);
        return I2C_ERR_NO_ACK;
    }
    for (uint16_t i = 0; i < w_len; i++) {
        IIC_WriteByte(bus, w_data[i]);
        if (IIC_WaitAck(bus) != I2C_OK) {
            g_rtos_ops->SemaphoreGive(bus->mutex);
            return I2C_ERR_NO_ACK;
        }
    }

    // 再执行读操作 (重复起始信号)
    IIC_Start(bus);
    IIC_WriteByte(bus, (dev_addr << 1) | 1);
    if (IIC_WaitAck(bus) != I2C_OK) {
        g_rtos_ops->SemaphoreGive(bus->mutex);
        return I2C_ERR_NO_ACK;
    }
    for (uint16_t i = 0; i < r_len; i++) {
        r_data[i] = IIC_ReadByte(bus, (i == (r_len - 1)) ? 0 : 1);
    }
    
    IIC_Stop(bus);
    g_rtos_ops->SemaphoreGive(bus->mutex);
    return I2C_OK;
}

/*
 * =====================================================================================
 * 全局操作接口实例定义
 * =====================================================================================
 */
const I2C_Bus_Ops_t g_i2c_bus_ops = {
    .Init = I2C_Bus_Init,
    .Write = I2C_Bus_Write,
    .Read = I2C_Bus_Read,
    .WriteAndRead = I2C_Bus_WriteAndRead,
};
