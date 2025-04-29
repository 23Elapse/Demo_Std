#include "pch.h"
#include "pcf8574.h"
const IIC_Ops_t IIC1_PCF8574 = {
    .dev_addr = PCF8574_ADDR,  // 设备地址（7位）
    .ReadByte = PCF8574_ReadBit,
    .WriteByte = PCF8574_WriteBit,
};
/**
 * @brief       初始化PCF8574
 * @param       无
 * @retval      0, 成功;
                1, 失败;
 */
uint8_t pcf8574_init(void)
{    
    GPIO_InitTypeDef gpio_init_struct;
    PCF8574_GPIO_CLK_ENABLE();                               /* 使能GPIOB时钟 */

    gpio_init_struct.GPIO_Pin = PCF8574_GPIO_PIN;                 /* PB12 */
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;            /* 输入 */
    gpio_init_struct.GPIO_OType = GPIO_OType_PP;         /* 推挽输出 */
    gpio_init_struct.GPIO_PuPd = GPIO_PuPd_UP;       /* 无上下拉 */
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;       /* 50MHz */
    
    GPIO_Init(PCF8574_GPIO_PORT, &gpio_init_struct);       /* 初始化PB12引脚 */

    IIC_INIT();                                              /* IIC初始化 */

    /* 检查PCF8574是否在位 */ 
    if(!IIC_Check(&IIC1_config, &IIC1_PCF8574))                                         /* 检测PCF8574是否在位 */   
        printf("[IIC%d] dev_addr:%d check success\n", IIC1_config.instance_id,IIC1_PCF8574.dev_addr); 
    else
        printf("[IIC%d] dev_addr:%d check failed\n", IIC1_config.instance_id,IIC1_PCF8574.dev_addr);
    return 0;
}
void rs485_tx_set(uint8_t en)
{
    PCF8574_WriteBit(RS485_RE_IO, en);
}

/**
 * @brief       设置PCF8574某个IO的高低电平
 * @param       IICx    : IIC设备实例指针
 * @param       bit    : 要设置的IO编号,0~7
 * @param       sta    : IO的状态;0或1
 * @retval      无
 */
IIC_Status PCF8574_WriteBit(uint8_t bit, uint8_t sta)
{
    uint8_t data = 0;
    PCF8574_ReadByte(&data);          /* 先读出原来的设置 */

    if (sta == 0)
    {
        data &= ~(1 << bit);
    }
    else
    {
        data |= 1 << bit;
    }

    PCF8574_WriteByte(data);            /* 写入新的数据 */
    return IIC_OK;
}

/**
 * @brief       读取PCF8574的某个IO的值
 * @param       IICx     : IIC设备实例指针
 * @param       bit     : 要读取的IO编号, 0~7
 * @retval      此IO口的值(状态, 0/1)
 */
IIC_Status PCF8574_ReadBit(uint8_t reg, uint8_t* bit)
{
    uint8_t data;
    PCF8574_ReadByte(&data);          /* 先读取这个8位IO的值  */

    if (data & (1 << *bit))
    {
        return 1;
    }
    else 
    {
        return 0; 
    }
}

/**
 * @brief       读取PCF8574的某个IO的值
 * @param       IICx     : IIC设备实例指针
 * @param       val     : 读取到的值
 * @retval      0, 成功;
                1, 失败;    
 */
uint8_t PCF8574_ReadByte(uint8_t *val)
{ 
    uint8_t instance_id = IIC1;
    IIC_Start(instance_id);
    IIC_WriteByte(instance_id, PCF8574_ADDR | 0X01);
    IIC_ReadByte(instance_id, 0, val); // 读取数据
    IIC_Stop(instance_id);               /* 产生一个停止条件 */
    return 0;
}

/**
 * @brief       向PCF8574写入一个字节
 * @param       IICx     : IIC设备实例指针
 * @param       data    : 要写入的数据
 * @retval      无
 */
void PCF8574_WriteByte(uint8_t data)
{
    uint8_t instance_id = IIC1;
    IIC_Start(instance_id);  
    IIC_WriteByte(instance_id, PCF8574_ADDR | 0X00);   /* 发送器件地址0X40,写数据 */
    IIC_WriteByte(instance_id, data);                  /* 发送字节 */
    IIC_Stop(instance_id);                           /* 产生一个停止条件  */
    delay_ms(10); 
}

