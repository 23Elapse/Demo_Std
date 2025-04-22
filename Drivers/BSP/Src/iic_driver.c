/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-03-26 20:19:40
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-18 20:52:06
 * @FilePath: \Demo\Drivers\BSP\Src\iic_driver.c
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
#include "iic_core.h"

IIC_Config_t IIC1_config = {
    .instance_id = IIC1,
    .scl_port = IIC1_SCL_GPIO_PORT,
    .scl_pin = IIC1_SCL_PIN,
    .sda_port = IIC1_SDA_GPIO_PORT,
    .sda_pin = IIC1_SDA_PIN,
};

/* 配置SCL和SDA引脚为开漏输出 */
static void _scl_config(IIC_Config_t *IICx)
{
    GPIO_InitTypeDef gpio = {
        .GPIO_Pin = IICx->scl_pin,
        .GPIO_Mode = GPIO_Mode_OUT,
        .GPIO_OType = GPIO_OType_OD,
        .GPIO_Speed = GPIO_Speed_50MHz};
    GPIO_Init(IICx->scl_port, &gpio);
}
static void _sda_config(IIC_Config_t *IICx)
{
    GPIO_InitTypeDef gpio = {
        .GPIO_Pin = IICx->sda_pin,
        .GPIO_Mode = GPIO_Mode_OUT,
        .GPIO_OType = GPIO_OType_OD,
        .GPIO_Speed = GPIO_Speed_50MHz};
    GPIO_Init(IICx->scl_port, &gpio);
}
/**
 * @brief  初始化IIC设备
 * @param  IICx: IIC设备实例指针
 * @retval IIC_Status_t 状态码
 * @note   需在系统时钟初始化后调用，配置GPIO为开漏模式
 */
IIC_Status IICx_Init(IIC_Config_t *IICx)
{
    if (!IICx->scl_port || !IICx->sda_port)
        return IIC_ERR_INIT;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    _scl_config(IICx);
    _sda_config(IICx);
    IIC_SCL_1(IICx->instance_id); // 初始拉高
    IIC_SDA_1(IICx->instance_id);
    return IIC_OK;
}
/**
 * @brief  IIC复位总线
 * @param  IICx: IIC设备实例指针
 * @retval 无
 */
void IIC_ResetBus(IIC_Config_t *IICx) {
    GPIO_InitTypeDef GPIO_InitStruct;
    // 临时将SCL配置为推挽输出
    GPIO_InitStruct.GPIO_Pin = IICx->scl_pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(IICx->scl_port, &GPIO_InitStruct);
    
    for (uint8_t i = 0; i < 9; i++) {
        IIC_SCL_1(IICx->instance_id);
        delay_us(5);
        IIC_SCL_0(IICx->instance_id);
        delay_us(5);
    }
    // 恢复开漏配置
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(IICx->scl_port, &GPIO_InitStruct);
}

/**
 * @brief  IIC发送起始信号
 * @param  IICx: IIC设备实例指针
 * @retval 无
 */
void IIC_Start(IIC_Config_t *IICx) {
    IIC_SDA_1(IICx->instance_id);  // 释放SDA
    IIC_SCL_1(IICx->instance_id);
    delay_us(5);
    IIC_SDA_0(IICx->instance_id);   // SDA拉低
    delay_us(5);
    IIC_SCL_0(IICx->instance_id);  // SCL拉低
}

/**
 * @brief  IIC发送停止信号
 * @param  IICx: IIC设备实例指针
 * @retval 无
 */
void IIC_Stop(IIC_Config_t *IICx) {
    IIC_SDA_0(IICx->instance_id);
    IIC_SCL_0(IICx->instance_id);
    delay_us(5);
    IIC_SCL_1(IICx->instance_id);
    delay_us(5);
    IIC_SDA_1(IICx->instance_id);
}

/**
 * @brief  IIC读取一个字节
 * @param  IICx: IIC设备实例指针
 * @param  ack: 1-发送ACK，0-发送NACK
 * @retval 读取的数据
 */
IIC_Status IIC_ReadByte(IIC_Config_t *IICx, uint8_t ack, uint8_t *data) {
    uint8_t i = 0;
    IIC_SDA_1(IICx->instance_id);  // 释放SDA
    for (i = 0; i < 8; i++) 
    {
        IIC_SCL_1(IICx->instance_id);
        *data <<= 1;
        delay_us(5);
        if (IIC_SDA_READ(IICx->instance_id)) 
        {
            *data |= 0x01;
        }
        IIC_SCL_0(IICx->instance_id);
        delay_us(5);
    }
    if (!ack) 
    {
        IIC_SDA_0(IICx->instance_id);  // 发送ACK
    } 
    else 
    {
        IIC_SDA_1(IICx->instance_id);  // 发送NACK
    }
    delay_us(5);
    IIC_SCL_1(IICx->instance_id);
    delay_us(5);
    IIC_SCL_0(IICx->instance_id);
    return IIC_OK;
}

/**
 * @brief  IIC发送一个字节
 * @param  IICx: IIC设备实例指针
 * @param  data: 要发送的数据
 * @retval ACK状态
 */
IIC_Status IIC_WriteByte(IIC_Config_t *IICx, uint8_t data) {
    uint8_t i;
    for (i = 0; i < 8; i++) 
    {
        if (data & 0x80) {
            IIC_SDA_1(IICx->instance_id);
        } else {
            IIC_SDA_0(IICx->instance_id);
        }
        data <<= 1;
        IIC_SCL_1(IICx->instance_id);
        delay_us(5);
        IIC_SCL_0(IICx->instance_id);
        delay_us(5);
    }
    IIC_SDA_1(IICx->instance_id);  // 释放SDA线
    delay_us(5);
    IIC_SCL_1(IICx->instance_id);
    uint8_t ack = IIC_SDA_READ(IICx->instance_id);  // 读取ACK
    delay_us(5);
    IIC_SCL_0(IICx->instance_id);
    if (ack == 0) return IIC_OK;
    else return IIC_ERR_NACK;  // 返回枚举值而非整数
}


/**
 * @brief  等待IIC写入完成
 * @param  IICx: IIC设备实例指针
 * @retval 无
 */
IIC_Status IIC_WaitWriteComplete(IIC_Config_t *IICx, uint8_t dev_addr) {
    uint16_t timeout = 1000;  // 超时时间
    while (timeout--) {
        IIC_Start(IICx);
        if (IIC_WriteByte(IICx, dev_addr) == 0) {
            IIC_Stop(IICx);
            return IIC_OK;  // 写入完成
        }
        IIC_Stop(IICx);
        delay_ms(1);
    }
    return IIC_ERR_TIMEOUT;  // 超时
}
/**
 * @brief  初始化IIC设备
 * @param  无
 * @retval 无
 */
void IIC_INIT(void)
{
    IICx_Init(&IIC1_config);  //初始化IIC1设备
    // IICx_Init(&IIC2_config);  //初始化IIC2设备
    // IICx_Init(&IIC3_config);  //初始化IIC3设备
    // IICx_Init(&IIC4_config);  //初始化IIC4设备
    // IICx_Init(&IIC5_config);  //初始化IIC5设备
    
//     while (IIC_Check(&IIC1_config ,&IIC1_EEPROM))  /* 检测不到 24c02 */ 
//     {
//         printf("AT24CXX Check Failed!\r\n");
//         delay_ms(1000);
//     }
//     printf("AT24CXX Check OK!\r\n");
}
/**
 * @brief  检测IIC设备是否存在
 * @param  IICx: IIC设备实例指针
 * @param  i2c_dev: 设备类型
 * @retval 0: 设备存在，1: 设备不存在
 */
uint8_t IIC_Check(IIC_Config_t *IICx, IIC_Ops_t *i2c_dev)
{
    if(!IICx->scl_port || !IICx->sda_port)
        return 1;  //设备不存在
    if(EEPROM_ADDR == i2c_dev->dev_addr)
    {
        uint8_t temp;
        i2c_dev->ReadByte(IICx, EEPROM_TYPE, &temp);   //读取末地址数据
        if (temp == 0x55) return 0;                     //已初始化
        
        i2c_dev->WriteByte(IICx, EEPROM_TYPE, 0x55);   //写入测试值
        i2c_dev->ReadByte(IICx, EEPROM_TYPE, &temp);
        return (temp == 0x55) ? 0 : 1;                  //返回检测结果[6,11](@ref)
    }
    else if(PCF8574_ADDR == i2c_dev->dev_addr)
    {
        IIC_Start(IICx);
        IIC_WriteByte(IICx, i2c_dev->dev_addr);
        IIC_Stop(IICx);     
        IIC_WriteByte(IICx, 0XFF);     
        return 0;
    }
    else
    {
        return 1;  //设备不存在
    }

}
/**
 * @brief  写入多个字节到指定寄存器
 * @param  IICx: IIC设备实例指针
 * @param  reg: 寄存器地址
 * @param  buf: 数据缓冲区
 * @param  len: 数据长度
 * @retval 无
 */
IIC_Status IICx_DevWrite(IIC_Config_t *IICx, IIC_Ops_t *i2c_dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    while (len--) {
        i2c_dev->WriteByte(IICx, reg++, *buf++);
    }
    return IIC_OK;
}
/**
 * @brief  读取多个字节从指定寄存器
 * @param  IICx: IIC设备实例指针
 * @param  reg: 寄存器地址
 * @param  buf: 数据缓冲区
 * @param  len: 数据长度
 * @retval 无
 */
IIC_Status IICx_DevRead(IIC_Config_t *IICx, IIC_Ops_t *i2c_dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    while (len--) {
        i2c_dev->ReadByte(IICx, reg++, buf++);
    }
    return IIC_OK;
}


