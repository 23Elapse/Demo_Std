/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-03-26 20:19:40
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-03-30 14:53:15
 * @FilePath: \Demo\Drivers\BSP\Src\iic1_driver.c
 * @Description:
 *
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved.
 */
#include "iic_core.h"

IIC_Device_t IIC1_EEPROM = {
    .instance_id = IIC1,
    .scl_port = IIC1_SCL_GPIO_PORT,
    .scl_pin = IIC1_SCL_PIN,
    .sda_port = IIC1_SDA_GPIO_PORT,
    .sda_pin = IIC1_SDA_PIN,
    .dev_addr = 0xA0,
    .timeout = 1000
};


/* 配置SCL和SDA引脚为开漏输出 */
static void _scl_config(IIC_Device_t *dev)
{
    GPIO_InitTypeDef gpio = {
        .GPIO_Pin = dev->scl_pin,
        .GPIO_Mode = GPIO_Mode_OUT,
        .GPIO_OType = GPIO_OType_OD,
        .GPIO_Speed = GPIO_Speed_50MHz};
    GPIO_Init(dev->scl_port, &gpio);
}
static void _sda_config(IIC_Device_t *dev)
{
    GPIO_InitTypeDef gpio = {
        .GPIO_Pin = dev->sda_pin,
        .GPIO_Mode = GPIO_Mode_OUT,
        .GPIO_OType = GPIO_OType_OD,
        .GPIO_Speed = GPIO_Speed_50MHz};
    GPIO_Init(dev->scl_port, &gpio);
}
/**
 * @brief  初始化IIC设备
 * @param  dev: IIC设备实例指针
 * @retval IIC_Status_t 状态码
 * @note   需在系统时钟初始化后调用，配置GPIO为开漏模式
 */
IIC_Status IICx_Init(IIC_Device_t *dev)
{
    if (!dev->scl_port || !dev->sda_port)
        return IIC_ERR_INIT;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    _scl_config(dev);
    _sda_config(dev);
    IIC_SCL_1(dev->instance_id); // 初始拉高
    IIC_SDA_1(dev->instance_id);
    return IIC_OK;
}
/**
 * @brief  IIC复位总线
 * @param  dev: IIC设备实例指针
 * @retval 无
 */
void IIC_ResetBus(IIC_Device_t *dev) {
    GPIO_InitTypeDef GPIO_InitStruct;
    // 临时将SCL配置为推挽输出
    GPIO_InitStruct.GPIO_Pin = dev->scl_pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(dev->scl_port, &GPIO_InitStruct);
    
    for (uint8_t i = 0; i < 9; i++) {
        IIC_SCL_1(dev->instance_id);
        delay_us(5);
        IIC_SCL_0(dev->instance_id);
        delay_us(5);
    }
    // 恢复开漏配置
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(dev->scl_port, &GPIO_InitStruct);
}

/**
 * @brief  IIC发送起始信号
 * @param  dev: IIC设备实例指针
 * @retval 无
 */
void IIC_Start(IIC_Device_t *dev) {
    IIC_SDA_1(dev->instance_id);  // 释放SDA
    IIC_SCL_1(dev->instance_id);
    delay_us(5);
    IIC_SDA_0(dev->instance_id);   // SDA拉低
    delay_us(5);
    IIC_SCL_0(dev->instance_id);  // SCL拉低
}

/**
 * @brief  IIC发送停止信号
 * @param  dev: IIC设备实例指针
 * @retval 无
 */
void IIC_Stop(IIC_Device_t *dev) {
    IIC_SDA_0(dev->instance_id);
    IIC_SCL_0(dev->instance_id);
    delay_us(5);
    IIC_SCL_1(dev->instance_id);
    delay_us(5);
    IIC_SDA_1(dev->instance_id);
}

/**
 * @brief  IIC读取一个字节
 * @param  dev: IIC设备实例指针
 * @param  ack: 1-发送ACK，0-发送NACK
 * @retval 读取的数据
 */
IIC_Status IIC_ReadByte(IIC_Device_t *dev, uint8_t ack, uint8_t *data) {
    uint8_t i = 0;
    IIC_SDA_1(dev->instance_id);  // 释放SDA
    for (i = 0; i < 8; i++) 
    {
        IIC_SCL_1(dev->instance_id);
        *data <<= 1;
        delay_us(5);
        if (IIC_SDA_READ(dev->instance_id)) 
        {
            *data |= 0x01;
        }
        IIC_SCL_0(dev->instance_id);
        delay_us(5);
    }
    if (!ack) 
    {
        IIC_SDA_0(dev->instance_id);  // 发送ACK
    } 
    else 
    {
        IIC_SDA_1(dev->instance_id);  // 发送NACK
    }
    delay_us(5);
    IIC_SCL_1(dev->instance_id);
    delay_us(5);
    IIC_SCL_0(dev->instance_id);
    return IIC_OK;
}

/**
 * @brief  IIC发送一个字节
 * @param  dev: IIC设备实例指针
 * @param  data: 要发送的数据
 * @retval ACK状态
 */
IIC_Status IIC_WriteByte(IIC_Device_t *dev, uint8_t data) {
    uint8_t i;
    for (i = 0; i < 8; i++) 
    {
        if (data & 0x80) {
            IIC_SDA_1(dev->instance_id);
        } else {
            IIC_SDA_0(dev->instance_id);
        }
        data <<= 1;
        IIC_SCL_1(dev->instance_id);
        delay_us(5);
        IIC_SCL_0(dev->instance_id);
        delay_us(5);
    }
    IIC_SDA_1(dev->instance_id);  // 释放SDA线
    delay_us(5);
    IIC_SCL_1(dev->instance_id);
    uint8_t ack = IIC_SDA_READ(dev->instance_id);  // 读取ACK
    delay_us(5);
    IIC_SCL_0(dev->instance_id);
    if (ack == 0) return IIC_OK;
    else return IIC_ERR_NACK;  // 返回枚举值而非整数
}

/**
 * @brief  IIC读一个字节到指定寄存器
 * @param  dev: IIC设备实例指针
 * @param  reg: 寄存器地址
 * @param  val: 数据
 * @retval 无
 */
IIC_Status IIC1_ReadByteFromReg(IIC_Device_t *dev, uint8_t reg, uint8_t *val)
{
    IIC_Start(dev);
    //地址处理逻辑与写入一致
    #if (EEPROM_TYPE > AT24C16)
        IIC_WriteByte(dev, dev->dev_addr);
        IIC_WriteByte(dev, reg >> 8);
    #else
        uint8_t devAddr = dev->dev_addr | ((reg >> 7) & 0x0E);
        I2C_WAIT_WRITE(IIC_WriteByte(dev, devAddr));
    #endif
    I2C_WAIT_WRITE(IIC_WriteByte(dev, reg & 0xFF));
    IIC_Start(dev);                          //重复起始条件
    I2C_WAIT_WRITE(IIC_WriteByte(dev, dev->dev_addr | 0x01));      //切换到读模式  
    IIC_ReadByte(dev, 0, val);               //读取数据（发送NACK结束）
    IIC_Stop(dev);

    return IIC_OK;
}

/**
 * @brief  IIC写一个字节到指定寄存器
 * @param  dev: IIC设备实例指针
 * @param  reg: 寄存器地址
 * @param  val: 数据
 * @retval 无
 */
IIC_Status IIC1_WriteByteToReg(IIC_Device_t *dev, uint8_t reg, uint8_t val)
{
    IIC_Start(dev);
    //动态生成设备地址（处理容量>2K的情况）
    #if (EEPROM_TYPE > AT24C16)
        IIC_WriteByte(dev, dev->dev_addr);         //基础设备地址
        IIC_WriteByte(dev, reg >> 8);           //发送高8位地址[4,7](@ref)
    #else
        uint8_t devAddr = dev->dev_addr | ((reg >> 7) & 0x0E); //24C04/08/16地址处理
        I2C_WAIT_WRITE(IIC_WriteByte(dev, devAddr));            //含地址高位的设备地址[1,9](@ref)
    #endif
    I2C_WAIT_WRITE(IIC_WriteByte(dev, reg & 0xFF));             //低8位地址  
    I2C_WAIT_WRITE(IIC_WriteByte(dev, val));                    //发送数据
    IIC_Stop(dev);
    IIC_WaitWriteComplete(dev);                          //等待写入完成
    return IIC_OK;
}

/**
 * @brief  等待IIC写入完成
 * @param  dev: IIC设备实例指针
 * @retval 无
 */
IIC_Status IIC_WaitWriteComplete(IIC_Device_t *dev) {
    uint16_t timeout = 1000;  // 超时时间
    while (timeout--) {
        IIC_Start(dev);
        if (IIC_WriteByte(dev, dev->dev_addr) == 0) {
            IIC_Stop(dev);
            return IIC_OK;  // 写入完成
        }
        IIC_Stop(dev);
        delay_ms(1);
    }
    return IIC_ERR_TIMEOUT;  // 超时
}
/* IIC1操作接口实例 */
const IIC_Ops_t IIC1_Operations = {
    .Init = IICx_Init,
    .ReadByte = IIC1_ReadByteFromReg,
    .WriteByte = IIC1_WriteByteToReg
};

/**
 * @brief  初始化IIC设备
 * @param  无
 * @retval 无
 */
void IIC_INIT(void)
{
    IIC1_Operations.Init(&IIC1_EEPROM);  //初始化IIC1设备
    while (IIC_Check(&IIC1_EEPROM))  /* 检测不到 24c02 */ 
    {
        printf("AT24CXX Check Failed!\r\n");
        delay_ms(1000);
    }
    printf("AT24CXX Check OK!\r\n");
}
/**
 * @brief  检测IIC设备是否存在
 * @param  dev: IIC设备实例指针
 * @param  dev_type: 设备类型
 * @retval 0: 设备存在，1: 设备不存在
 */
uint8_t IIC_Check(IIC_Device_t *dev)
{
    if(!dev->scl_port || !dev->sda_port)
        return 1;  //设备不存在
    if(EEPROM_ADDR == dev->dev_addr)
    {
        uint8_t temp;
        IIC1_Operations.ReadByte(dev, EEPROM_TYPE, &temp);   //读取末地址数据
        if (temp == 0x55) return 0;                     //已初始化
        
        IIC1_Operations.WriteByte(dev, EEPROM_TYPE, 0x55);   //写入测试值
        IIC1_Operations.ReadByte(dev, EEPROM_TYPE, &temp);
        return (temp == 0x55) ? 0 : 1;                  //返回检测结果[6,11](@ref)
    }
    else if(PCF8574_ADDR == dev->dev_addr)
    {
        IIC_Start(dev);
        IIC_WriteByte(dev, dev->dev_addr);
        IIC_Stop(dev);     
        IIC_WriteByte(dev, 0XFF);     
    }
    else
    {
        return 1;  //设备不存在
    }
    return 0;

}
/**
 * @brief  写入多个字节到指定寄存器
 * @param  dev: IIC设备实例指针
 * @param  reg: 寄存器地址
 * @param  buf: 数据缓冲区
 * @param  len: 数据长度
 * @retval 无
 */
IIC_Status IIC_Write(IIC_Device_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    while (len--) {
        IIC1_Operations.WriteByte(dev, reg++, *buf++);
    }
    return IIC_OK;
}
/**
 * @brief  读取多个字节从指定寄存器
 * @param  dev: IIC设备实例指针
 * @param  reg: 寄存器地址
 * @param  buf: 数据缓冲区
 * @param  len: 数据长度
 * @retval 无
 */
IIC_Status IIC_Read(IIC_Device_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    while (len--) {
        IIC1_Operations.ReadByte(dev, reg++, buf++);
    }
    return IIC_OK;
}


