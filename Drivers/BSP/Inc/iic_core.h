#ifndef __IIC_CORE_H
#define __IIC_CORE_H

#include "pch.h"

#ifdef IIC_DEBUG
#define LOG_IIC_EVENT(dev, event) printf("[IIC%d] %s\n", dev->instance_id, event)
#else
#define LOG_IIC_EVENT(dev, event)
#endif
#define EEPROM_TYPE AT24C02  //通过宏定义选择具体型号
#define EEPROM_ADDR 0xA0     //基础设备地址（A2/A1/A0接地时为0xA0）

#define AT24C01     127
#define AT24C02     255
#define AT24C04     511
#define AT24C08     1023
#define AT24C16     2047
#define AT24C32     4095
#define AT24C64     8191
#define AT24C128    16383
#define AT24C256    32767

/* 开发板使用的是24c02，所以定义EE_TYPE为AT24C02 */
#define EE_TYPE     AT24C02

#define IIC1                  1
#define IIC2                  2
#define IIC1_SCL_GPIO_PORT    GPIOH
#define IIC1_SCL_PIN          GPIO_Pin_4
#define IIC1_SDA_GPIO_PORT    GPIOH
#define IIC1_SDA_PIN          GPIO_Pin_5

#define IIC2_SCL_GPIO_PORT    GPIOB
#define IIC2_SCL_PIN          GPIO_Pin_10
#define IIC2_SDA_GPIO_PORT    GPIOB
#define IIC2_SDA_PIN          GPIO_Pin_11

/* IO 操作 */ 
#define IIC_SCL_1(IIC_x)        do{ if(IIC_x == IIC1) GPIO_SetBits(IIC1_SCL_GPIO_PORT, IIC1_SCL_PIN); else GPIO_SetBits(IIC2_SCL_GPIO_PORT, IIC2_SCL_PIN);; }while(0)    /* SCL */
#define IIC_SCL_0(IIC_x)        do{ if(IIC_x == IIC1) GPIO_ResetBits(IIC1_SCL_GPIO_PORT, IIC1_SCL_PIN); else GPIO_ResetBits(IIC2_SCL_GPIO_PORT, IIC2_SCL_PIN); }while(0)    /* SCL */
#define IIC_SDA_1(IIC_x)        do{ if(IIC_x == IIC1) GPIO_SetBits(IIC1_SDA_GPIO_PORT, IIC1_SDA_PIN); else GPIO_SetBits(IIC2_SDA_GPIO_PORT, IIC2_SDA_PIN); }while(0)    /* SDA */
#define IIC_SDA_0(IIC_x)        do{ if(IIC_x == IIC1) GPIO_ResetBits(IIC1_SDA_GPIO_PORT, IIC1_SDA_PIN); else GPIO_ResetBits(IIC2_SDA_GPIO_PORT, IIC2_SDA_PIN); }while(0)    /* SDA */
#define IIC_SDA_READ(IIC_x)     ((IIC_x == IIC1) ? GPIO_ReadInputDataBit(IIC1_SDA_GPIO_PORT, IIC1_SDA_PIN) : \
                                    GPIO_ReadInputDataBit(IIC2_SDA_GPIO_PORT, IIC2_SDA_PIN))    /* 读SDA */ 
#define IIC_SDA_READ_BIT(IIC_x) ((IIC_x == IIC1) ? GPIO_ReadInputDataBit(IIC1_SDA_GPIO_PORT, IIC1_SDA_PIN) : \
                                    GPIO_ReadInputDataBit(IIC2_SDA_GPIO_PORT, IIC2_SDA_PIN))    /* 读SDA */
#define I2C_TIMEOUT 1000   // I2C超时时间
#define I2C_ERROR_TIMEOUT 0xFF
// 轮询带超时的I2C写入
// #define I2C_WAIT_WRITE(cmd) do { \
//     uint16_t timeout = I2C_TIMEOUT; \
//     while (cmd) if (--timeout == 0) return I2C_ERROR_TIMEOUT; \
// } while (0)

#define I2C_WAIT_WRITE(cmd) do { \
    uint16_t timeout = I2C_TIMEOUT; \
    IIC_Status status; \
    do { \
        status = (cmd); \
        if (--timeout == 0) { \
            return IIC_ERR_TIMEOUT; \
        } \
    } while (status != IIC_OK); \
} while (0)


typedef enum {
    IIC_OK = 0,
    IIC_ERR_INIT,
    IIC_ERR_TIMEOUT,
    IIC_ERR_NACK,
    IIC_ERR_BUS_BUSY
} IIC_Status;

typedef struct IIC_Device_t IIC_Device_t; // 前向声明
struct IIC_Device_t{
    uint8_t instance_id;       // 设备实例标识(IIC1/IIC2)
    GPIO_TypeDef* scl_port;    // SCL端口
    uint16_t scl_pin;          // SCL引脚
    GPIO_TypeDef* sda_port;    // SDA端口
    uint16_t sda_pin;          // SDA引脚
    uint8_t dev_addr;          // 设备地址（7位）
    uint32_t timeout;          // 超时时间
    void* user_data;           // 用户扩展数据
    
};


extern IIC_Device_t IIC1_EEPROM ;  // IIC1设备实例

/* 操作接口函数指针类型 */
typedef IIC_Status (*IIC_InitFunc)(struct IIC_Device_t*);
typedef IIC_Status (*IIC_ReadFunc)(struct IIC_Device_t*, uint8_t reg, uint8_t* val);
typedef IIC_Status (*IIC_WriteFunc)(struct IIC_Device_t*, uint8_t reg, uint8_t val);

/* 统一操作接口结构体 */
typedef struct {
    IIC_InitFunc    Init;
    IIC_ReadFunc    ReadByte;
    IIC_WriteFunc   WriteByte;
} IIC_Ops_t;

IIC_Status IIC_Read(IIC_Device_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);
IIC_Status IIC_Write(IIC_Device_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);
void IIC_Start(IIC_Device_t *dev);
void IIC_Stop(IIC_Device_t *dev);
IIC_Status IIC_ReadByte(IIC_Device_t *dev, uint8_t ack, uint8_t *data);
IIC_Status IIC_WriteByte(IIC_Device_t *dev, uint8_t data);
IIC_Status IIC_WaitWriteComplete(IIC_Device_t *dev);
uint8_t IIC_Check(IIC_Device_t *dev);
void IIC_INIT(void);
void IIC_ResetBus(IIC_Device_t *dev);   

#endif
