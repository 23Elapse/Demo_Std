
#ifndef __IIC_CORE_H
#define __IIC_CORE_H

#include "pch.h"

typedef enum {
    IIC_OK = 0,
    IIC_ERR_INIT,
    IIC_ERR_TIMEOUT,
    IIC_ERR_NACK,
    IIC_ERR_BUS_BUSY
} IIC_Status;


typedef struct {
    uint8_t instance_id;       // 设备实例标识(IIC1/IIC2)
    GPIO_TypeDef* scl_port;    // SCL端口
    uint16_t scl_pin;          // SCL引脚
    GPIO_TypeDef* sda_port;    // SDA端口
    uint16_t sda_pin;          // SDA引脚
    uint8_t dev_addr;          // 设备地址（7位）
    uint32_t timeout;          // 超时时间
    void* user_data;           // 用户扩展数据
} IIC_Device_t;

extern IIC_Device_t IIC_1 ;  // IIC1设备实例

/* 操作接口函数指针类型 */
typedef IIC_Status (*IIC_InitFunc)(struct IIC_Device*);
typedef IIC_Status (*IIC_ReadFunc)(struct IIC_Device*, uint8_t reg, uint8_t* val);
typedef IIC_Status (*IIC_WriteFunc)(struct IIC_Device*, uint8_t reg, uint8_t val);

/* 统一操作接口结构体 */
typedef struct {
    IIC_InitFunc    Init;
    IIC_ReadFunc    ReadByte;
    IIC_WriteFunc   WriteByte;
} IIC_Ops_t;

IIC_Status IIC_Device1_Read(IIC_Device_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);
IIC_Status IIC_Device1_Write(IIC_Device_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);
void IIC_Device1_Start(IIC_Device_t *dev);
void IIC_Device1_Stop(IIC_Device_t *dev);
IIC_Status IIC_Device1_ReadByte(IIC_Device_t *dev, uint8_t ack);
IIC_Status IIC_Device1_WriteByte(IIC_Device_t *dev, uint8_t data);
IIC_Status IIC_Device1_WaitWriteComplete(IIC_Device_t *dev);
IIC_Status IIC_Device1_Write(IIC_Device_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);
IIC_Status IIC_Device1_Read(IIC_Device_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t IIC_Device1_Check(IIC_Device_t *dev);
void IIC_Device1_INIT(void);


#endif
