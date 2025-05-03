/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-03-26 20:19:40
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 14:14:15
 * @FilePath: \Demo\Drivers\BSP\Src\iic_driver.c
 * @Description: IIC 驱动实现，支持 RTOS 抽象
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "iic_core.h"
#include "common_driver.h"
#include "pch.h"

/**
 * @brief IIC1 设备配置
 */
IIC_Config_t IIC1_config = {
    .instance_id = IIC1,
    .scl_port = IIC1_SCL_GPIO_PORT,
    .scl_pin = IIC1_SCL_PIN,
    .sda_port = IIC1_SDA_GPIO_PORT,
    .sda_pin = IIC1_SDA_PIN,
    .mutex = NULL,
    .device_count = 0};

/**
 * @brief 微秒级忙等待延时
 * @param us 延时时间（微秒）
 */
static void delay_us(uint32_t us)
{
    uint32_t cycles = us * 168 / 6;
    while (cycles--)
        __NOP();
}

/**
 * @brief 配置 SCL 引脚为开漏输出
 * @param IICx IIC 设备实例指针
 */
static void _scl_config(IIC_Config_t *IICx)
{
    Common_GPIO_Init(IICx->scl_port, IICx->scl_pin, GPIO_Mode_OUT,
                     GPIO_OType_OD, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0);
}

/**
 * @brief 配置 SDA 引脚为开漏输出
 * @param IICx IIC 设备实例指针
 */
static void _sda_config(IIC_Config_t *IICx)
{
    Common_GPIO_Init(IICx->sda_port, IICx->sda_pin, GPIO_Mode_OUT,
                     GPIO_OType_OD, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0);
}

/**
 * @brief 初始化 IIC 设备
 * @param IICx IIC 设备实例指针
 * @return IIC 操作状态
 */
IIC_Status IICx_Init(IIC_Config_t *IICx)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !IICx || !IICx->scl_port || !IICx->sda_port)
    {
        printf("[IIC%d] Invalid RTOS or config\n", IICx->instance_id);
        return IIC_ERR_INIT;
    }

    if (IICx->mutex == NULL)
    {
        IICx->mutex = rtos_ops->SemaphoreCreate();
        if (IICx->mutex == NULL)
        {
            printf("[IIC%d] Failed to create mutex\n", IICx->instance_id);
            return IIC_ERR_INIT;
        }
    }

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    _scl_config(IICx);
    _sda_config(IICx);
    IIC_SCL_1(IICx->instance_id);
    IIC_SDA_1(IICx->instance_id);

    printf("[IIC%d] Initialized successfully\n", IICx->instance_id);
    return IIC_OK;
}

/**
 * @brief IIC 复位总线
 * @param IICx IIC 设备实例指针
 * @return IIC 操作状态
 */
IIC_Status IIC_ResetBus(IIC_Config_t *IICx)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !IICx)
    {
        printf("[IIC%d] Invalid RTOS or config\n", IICx ? IICx->instance_id : 0);
        return IIC_ERR_INIT;
    }

    if (!rtos_ops->SemaphoreTake(IICx->mutex, 100))
    {
        printf("[IIC%d] Failed to take mutex, timeout after 100ms\n", IICx->instance_id);
        return IIC_ERR_TIMEOUT;
    }

    Common_GPIO_Init(IICx->scl_port, IICx->scl_pin, GPIO_Mode_OUT,
                     GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0);

    for (uint8_t i = 0; i < 9; i++)
    {
        IIC_SCL_1(IICx->instance_id);
        delay_us(5);
        IIC_SCL_0(IICx->instance_id);
        delay_us(5);
    }

    Common_GPIO_Init(IICx->scl_port, IICx->scl_pin, GPIO_Mode_OUT,
                     GPIO_OType_OD, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0);

    rtos_ops->SemaphoreGive(IICx->mutex);
    return IIC_OK;
}

/**
 * @brief IIC 发送起始信号
 * @param instance_id IIC 设备实例 ID
 * @return IIC 操作状态
 */
IIC_Status IIC_Start(uint8_t instance_id)
{
    IIC_SDA_1(instance_id);
    IIC_SCL_1(instance_id);
    delay_us(5);
    IIC_SDA_0(instance_id);
    delay_us(5);
    IIC_SCL_0(instance_id);

    LOG_IIC_EVENT(&IIC1_config, "Start signal sent");
    return IIC_OK;
}

/**
 * @brief IIC 发送停止信号
 * @param instance_id IIC 设备实例 ID
 * @return IIC 操作状态
 */
IIC_Status IIC_Stop(uint8_t instance_id)
{
    IIC_SDA_0(instance_id);
    IIC_SCL_0(instance_id);
    delay_us(5);
    IIC_SCL_1(instance_id);
    delay_us(5);
    IIC_SDA_1(instance_id);

    LOG_IIC_EVENT(&IIC1_config, "Stop signal sent");
    return IIC_OK;
}

/**
 * @brief IIC 读取一个字节
 * @param instance_id IIC 设备实例 ID
 * @param ack 1-发送 ACK，0-发送 NACK
 * @param data 读取的数据存储指针
 * @return IIC 操作状态
 */
IIC_Status IIC_ReadByte(uint8_t instance_id, uint8_t ack, uint8_t *data)
{
    if (!data)
    {
        printf("[IIC%d] Invalid data pointer\n", instance_id);
        return IIC_ERR_INIT;
    }

    *data = 0;
    IIC_SDA_1(instance_id);
    for (uint8_t i = 0; i < 8; i++)
    {
        IIC_SCL_1(instance_id);
        *data <<= 1;
        delay_us(5);
        if (IIC_SDA_READ(instance_id))
        {
            *data |= 0x01;
        }
        IIC_SCL_0(instance_id);
        delay_us(5);
    }

    if (!ack)
    {
        IIC_SDA_0(instance_id);
    }
    else
    {
        IIC_SDA_1(instance_id);
    }
    delay_us(5);
    IIC_SCL_1(instance_id);
    delay_us(5);
    IIC_SCL_0(instance_id);

    return IIC_OK;
}

/**
 * @brief IIC 发送一个字节
 * @param instance_id IIC 设备实例 ID
 * @param data 要发送的数据
 * @return IIC 操作状态
 */
IIC_Status IIC_WriteByte(uint8_t instance_id, uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (data & 0x80)
        {
            IIC_SDA_1(instance_id);
        }
        else
        {
            IIC_SDA_0(instance_id);
        }
        data <<= 1;
        IIC_SCL_1(instance_id);
        delay_us(5);
        IIC_SCL_0(instance_id);
        delay_us(5);
    }

    IIC_SDA_1(instance_id);
    delay_us(5);
    IIC_SCL_1(instance_id);
    uint8_t ack = IIC_SDA_READ(instance_id);
    delay_us(5);
    IIC_SCL_0(instance_id);

    return ack == 0 ? IIC_OK : IIC_ERR_NACK;
}

/**
 * @brief 等待 IIC 写入完成
 * @param instance_id IIC 设备实例 ID
 * @param dev_addr 设备地址
 * @return IIC 操作状态
 */
IIC_Status IIC_WaitWriteComplete(uint8_t instance_id, uint8_t dev_addr)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops)
    {
        printf("[IIC%d] RTOS ops not initialized\n", instance_id);
        return IIC_ERR_INIT;
    }

    rtos_ops->Delay(3);
    uint16_t timeout = I2C_TIMEOUT;
    while (timeout--)
    {
        IIC_Start(instance_id);
        if (IIC_WriteByte(instance_id, dev_addr) == IIC_OK)
        {
            IIC_Stop(instance_id);
            return IIC_OK;
        }
        IIC_Stop(instance_id);
        rtos_ops->Delay(1);
    }

    printf("[IIC%d] Write timeout\n", instance_id);
    return IIC_ERR_TIMEOUT;
}

/**
 * @brief 初始化所有 IIC 设备
 */
void IIC_INIT(void)
{
    IIC_Status status = IICx_Init(&IIC1_config);
    if (status != IIC_OK)
    {
        printf("[IIC] Initialization failed: %d\n", status);
        return;
    }

    // 挂载 IIC 设备
    IIC_AttachDevice(&IIC1_config, (IIC_Ops_t *)&IIC1_EEPROM);
    IIC_AttachDevice(&IIC1_config, (IIC_Ops_t *)&IIC1_PCF8574);
}

/**
 * @brief 挂载 IIC 设备
 * @param IICx IIC 设备实例指针
 * @param device IIC 设备操作接口
 * @return IIC 操作状态
 */
IIC_Status IIC_AttachDevice(IIC_Config_t *IICx, IIC_Ops_t *device)
{
    if (!IICx || !device || IICx->device_count >= MAX_IIC_DEVICES)
    {
        printf("[IIC%d] Invalid config or max devices reached\n", IICx->instance_id);
        return IIC_ERR_INIT;
    }

    IICx->devices[IICx->device_count++] = device;
    printf("[IIC%d] Attached device with addr 0x%02X\n", IICx->instance_id, device->dev_addr);
    return IIC_OK;
}

/**
 * @brief 检测 IIC 设备是否存在
 * @param IICx IIC 设备实例指针
 * @param i2c_dev IIC 设备操作接口
 * @return 0: 设备存在，1: 设备不存在
 */
uint8_t IIC_Check(IIC_Config_t *IICx, const IIC_Ops_t *i2c_dev)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !IICx || !IICx->scl_port || !IICx->sda_port || !i2c_dev)
    {
        printf("[IIC%d] Invalid config, device, or RTOS ops\n", IICx ? IICx->instance_id : 0);
        return 1;
    }

    if (!rtos_ops->SemaphoreTake(IICx->mutex, 100))
    {
        printf("[IIC%d] Failed to take mutex, timeout after 100ms\n", IICx->instance_id);
        return 1;
    }

    if (EEPROM_ADDR == i2c_dev->dev_addr)
    {
        uint8_t temp;
        IIC_Status status = i2c_dev->ReadByte(EEPROM_TYPE, &temp);
        if (status != IIC_OK)
        {
            printf("[IIC%d] Failed to read EEPROM (addr 0x%02X): %d\n", IICx->instance_id, i2c_dev->dev_addr, status);
            rtos_ops->SemaphoreGive(IICx->mutex);
            return 1;
        }

        if (temp == 0x55)
        {
            printf("[IIC%d] EEPROM already initialized (addr 0x%02X)\n", IICx->instance_id, i2c_dev->dev_addr);
            rtos_ops->SemaphoreGive(IICx->mutex);
            return 0;
        }

        status = i2c_dev->WriteByte(EEPROM_TYPE, 0x55);
        if (status != IIC_OK)
        {
            printf("[IIC%d] Failed to write EEPROM (addr 0x%02X): %d\n", IICx->instance_id, i2c_dev->dev_addr, status);
            rtos_ops->SemaphoreGive(IICx->mutex);
            return 1;
        }

        status = i2c_dev->ReadByte(EEPROM_TYPE, &temp);
        rtos_ops->SemaphoreGive(IICx->mutex);
        return (temp == 0x55) ? 0 : 1;
    }
    else if (PCF8574_ADDR == i2c_dev->dev_addr)
    {
        IIC_Start(IICx->instance_id);
        IIC_Status status = IIC_WriteByte(IICx->instance_id, i2c_dev->dev_addr);
        IIC_Stop(IICx->instance_id);
        if (status != IIC_OK)
        {
            printf("[IIC%d] Failed to write PCF8574 (addr 0x%02X): %d\n", IICx->instance_id, i2c_dev->dev_addr, status);
            rtos_ops->SemaphoreGive(IICx->mutex);
            return 1;
        }

        status = IIC_WriteByte(IICx->instance_id, 0xFF);
        rtos_ops->SemaphoreGive(IICx->mutex);
        return (status == IIC_OK) ? 0 : 1;
    }

    printf("[IIC%d] Unsupported device address: 0x%02X\n", IICx->instance_id, i2c_dev->dev_addr);
    rtos_ops->SemaphoreGive(IICx->mutex);
    return 1;
}

/**
 * @brief 写入多个字节到指定寄存器
 * @param i2c_dev IIC 设备操作接口
 * @param reg 寄存器地址
 * @param buf 数据缓冲区
 * @param len 数据长度
 * @return IIC 操作状态
 */
IIC_Status IICx_DevWrite(IIC_Ops_t *i2c_dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (!i2c_dev || !buf || len == 0)
    {
        printf("[IIC] Invalid parameters for write\n");
        return IIC_ERR_INIT;
    }

    while (len--)
    {
        IIC_Status status = i2c_dev->WriteByte(reg++, *buf++);
        if (status != IIC_OK)
        {
            printf("[IIC] Write failed at reg 0x%02X: %d\n", reg - 1, status);
            return status;
        }
    }

    return IIC_OK;
}

/**
 * @brief 从指定寄存器读取多个字节
 * @param i2c_dev IIC 设备操作接口
 * @param reg 寄存器地址
 * @param buf 数据缓冲区
 * @param len 数据长度
 * @return IIC 操作状态
 */
IIC_Status IICx_DevRead(IIC_Ops_t *i2c_dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (!i2c_dev || !buf || len == 0)
    {
        printf("[IIC] Invalid parameters for read\n");
        return IIC_ERR_INIT;
    }

    while (len--)
    {
        IIC_Status status = i2c_dev->ReadByte(reg++, buf++);
        if (status != IIC_OK)
        {
            printf("[IIC] Read failed at reg 0x%02X: %d\n", reg - 1, status);
            return status;
        }
    }

    return IIC_OK;
}
