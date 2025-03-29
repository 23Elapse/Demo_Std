/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-11-16 17:04:35
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-03-25 00:36:04
 * @FilePath: \Demo\Drivers\BSP\Src\24cxx.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "pch.h"

/**
 * @brief       初始化IIC接口
 * @param       无
 * @retval      无
 */
void at24cxx_init(void)
{
    IIC_Init(&IIC1_Struct);
}
void AT24CXX_WaitWriteComplete(IIC_TypeDef *IIC_Struct) {
    uint16_t timeout = 1000;  // 超时时间
    while (timeout--) {
        IIC_Start(IIC_Struct);
        if (IIC_WriteByte(IIC_Struct, EEPROM_ADDR) == 0) {
            IIC_Stop(IIC_Struct);
            return;
        }
        IIC_Stop(IIC_Struct);
        delay_ms(1);
    }
}
/**
 * @brief       在AT24CXX指定地址读出一个数据
 * @param       addr: 开始读数的地址
 * @retval      读到的数据
 */
    /* 根据不同的24CXX型号, 发送高位地址
     * 1, 24C16以上的型号, 分2个字节发送地址
     * 2, 24C16及以下的型号, 分1个低字节地址 + 占用器件地址的bit1~bit3位 用于表示高位地址, 最多11位地址
     *    对于24C01/02, 其器件地址格式(8bit)为: 1  0  1  0  A2  A1  A0  R/W
     *    对于24C04,    其器件地址格式(8bit)为: 1  0  1  0  A2  A1  a8  R/W
     *    对于24C08,    其器件地址格式(8bit)为: 1  0  1  0  A2  a9  a8  R/W
     *    对于24C16,    其器件地址格式(8bit)为: 1  0  1  0  a10 a9  a8  R/W
     *    R/W      : 读/写控制位 0,表示写; 1,表示读;
     *    A0/A1/A2 : 对应器件的1,2,3引脚(只有24C01/02/04/8有这些脚)
     *    a8/a9/a10: 对应存储整列的高位地址, 11bit地址最多可以表示2048个位置, 可以寻址24C16及以内的型号
     */    
uint8_t AT24CXX_ReadOneByte(IIC_TypeDef *IIC_Struct, uint16_t addr) {
    uint8_t temp = 0;
    IIC_Start(IIC_Struct);
    //地址处理逻辑与写入一致
    #if (EEPROM_TYPE > AT24C16)
    IIC_WriteByte(IIC_Struct, EEPROM_ADDR);
        IIC_WaitAck(IIC_Struct);
        IIC_WriteByte(IIC_Struct, addr >> 8);
    #else
        uint8_t devAddr = EEPROM_ADDR | ((addr >> 7) & 0x0E);
        I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, devAddr));
    #endif
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, addr & 0xFF));
    IIC_Start(IIC_Struct);                          //重复起始条件
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, EEPROM_ADDR | 0x01));      //切换到读模式  
    temp = IIC_ReadByte(IIC_Struct, 0);               //读取数据（发送NACK结束）
    IIC_Stop(IIC_Struct);
    
    return temp;
}

/**
 * @brief       在AT24CXX指定地址写入一个数据
 * @param       addr: 写入的地址
 * @param       data: 写入的数据
 * @retval      无
 */
void AT24CXX_WriteOneByte(IIC_TypeDef *IIC_Struct, uint16_t addr, uint8_t data) {
    IIC_Start(IIC_Struct);
    
    //动态生成设备地址（处理容量>2K的情况）
    #if (EEPROM_TYPE > AT24C16)
        IIC_WriteByte(IIC_Struct, EEPROM_ADDR);         //基础设备地址
        IIC_WaitAck(IIC_Struct);
        IIC_WriteByte(IIC_Struct, addr >> 8);           //发送高8位地址[4,7](@ref)
    #else
        uint8_t devAddr = EEPROM_ADDR | ((addr >> 7) & 0x0E); //24C04/08/16地址处理
        I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, devAddr));            //含地址高位的设备地址[1,9](@ref)
    #endif
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, addr & 0xFF));             //低8位地址  
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, data));                    //发送数据
    IIC_Stop(IIC_Struct);
    AT24CXX_WaitWriteComplete(IIC_Struct);                          //等待写入完成
    // delay_ms(10);                          //统一延时等待写入完成[3,5](@ref)
}

/**
 * @brief       从AT24CXX指定地址开始写入指定长度的数据
 * @param       addr: 开始写入的地址
 * @param       pbuf: 数据数组首地址
 * @param       datalen: 要写入数据的个数
 * @retval      无
 */
void AT24CXX_Write(IIC_TypeDef *IIC_Struct, uint16_t addr, uint8_t *buf, uint16_t len) {
    while (len--) {
        AT24CXX_WriteOneByte(IIC_Struct, addr++, *buf++);
    }
}

/**
 * @brief       从AT24CXX指定地址开始读出指定长度的数据
 * @param       addr: 开始读出的地址
 * @param       pbuf: 数据数组首地址
 * @param       datalen: 要读出数据的个数
 * @retval      无
 */
void AT24CXX_Read(IIC_TypeDef *IIC_Struct, uint16_t addr, uint8_t *buf, uint16_t len) {
    while (len--) {
        *buf++ = AT24CXX_ReadOneByte(IIC_Struct, addr++);
    }
}

/**
 * @brief       检查AT24CXX是否正常
 * @note        检测原理: 在器件的末地址写如0X55, 然后再读取, 如果读取值为0X55
 *              则表示检测正常. 否则,则表示检测失败.
 * @param       无
 * @retval      检测结果
 *              0: 检测成功
 *              1: 检测失败
 */
uint8_t AT24CXX_Check(IIC_TypeDef *IIC_Struct)
{
    uint8_t temp;
    temp = AT24CXX_ReadOneByte(IIC_Struct, EEPROM_TYPE);   //读取末地址数据
    if (temp == 0x55) return 0;                     //已初始化
    
    AT24CXX_WriteOneByte(IIC_Struct, EEPROM_TYPE, 0x55);   //写入测试值
    temp = AT24CXX_ReadOneByte(IIC_Struct, EEPROM_TYPE);
    return (temp == 0x55) ? 0 : 1;                  //返回检测结果[6,11](@ref)
}

//
// Created by Elapse on 2024-11-16.
//



