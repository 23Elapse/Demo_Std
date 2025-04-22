#include "api_eeprom.h"


const IIC_Ops_t IIC1_EEPROM = {
    .dev_addr = EEPROM_ADDR,  // 设备地址（7位）
    .ReadByte = EEPROMReadByteFromReg,
    .WriteByte = EEPROMWriteByteToReg
};

/**
 * @brief  IIC读一个字节到指定寄存器
 * @param  IICx: IIC设备实例指针
 * @param  reg: 寄存器地址
 * @param  val: 数据
 * @retval 无
 */
IIC_Status EEPROMReadByteFromReg(IIC_Config_t *IICx, uint8_t reg, uint8_t *val)
{
    IIC_Start(IICx);
    //地址处理逻辑与写入一致
    #if (EEPROM_TYPE > AT24C16)
        IIC_WriteByte(IICx, EEPROM_ADDR);
        IIC_WriteByte(IICx, reg >> 8);
    #else
        uint8_t devAddr = EEPROM_ADDR | ((reg >> 7) & 0x0E);
        I2C_WAIT_WRITE(IIC_WriteByte(IICx, devAddr));
    #endif
    I2C_WAIT_WRITE(IIC_WriteByte(IICx, reg & 0xFF));
    IIC_Start(IICx);                          //重复起始条件
    I2C_WAIT_WRITE(IIC_WriteByte(IICx, EEPROM_ADDR | 0x01));      //切换到读模式  
    IIC_ReadByte(IICx, 0, val);               //读取数据（发送NACK结束）
    IIC_Stop(IICx);

    return IIC_OK;
}

/**
 * @brief  IIC写一个字节到指定寄存器
 * @param  IICx: IIC设备实例指针
 * @param  reg: 寄存器地址
 * @param  val: 数据
 * @retval 无
 */
IIC_Status EEPROMWriteByteToReg(IIC_Config_t *IICx, int8_t reg, uint8_t val)
{
    IIC_Start(IICx);
    //动态生成设备地址（处理容量>2K的情况）
    #if (EEPROM_TYPE > AT24C16)
        IIC_WriteByte(IICx, EEPROM_ADDR);         //基础设备地址
        IIC_WriteByte(IICx, reg >> 8);           //发送高8位地址[4,7](@ref)
    #else
        uint8_t devAddr = EEPROM_ADDR | ((reg >> 7) & 0x0E); //24C04/08/16地址处理
        I2C_WAIT_WRITE(IIC_WriteByte(IICx, devAddr));            //含地址高位的设备地址[1,9](@ref)
    #endif
    I2C_WAIT_WRITE(IIC_WriteByte(IICx, reg & 0xFF));             //低8位地址  
    I2C_WAIT_WRITE(IIC_WriteByte(IICx, val));                    //发送数据
    IIC_Stop(IICx);
    IIC_WaitWriteComplete(IICx, devAddr);                          //等待写入完成
    return IIC_OK;
}
