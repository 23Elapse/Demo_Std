// #include "pcf8574.h"
#include "pch.h"
// #include "iic_core.h"
IIC_Device_t IIC1_PCF8574 = {
    .instance_id = IIC1,
    .scl_port = IIC1_SCL_GPIO_PORT,
    .scl_pin = IIC1_SCL_PIN,
    .sda_port = IIC1_SDA_GPIO_PORT,
    .sda_pin = IIC1_SDA_PIN,
    .dev_addr = PCF8574_ADDR,
    .timeout = 1000
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
    if(!IIC_Check(&IIC1_PCF8574))                                         /* 检测PCF8574是否在位 */   
        printf("[IIC%d] dev_addr:%d check success\n", IIC1_PCF8574.instance_id,IIC1_PCF8574.dev_addr); 
    else
        printf("[IIC%d] dev_addr:%d check failed\n", IIC1_PCF8574.instance_id,IIC1_PCF8574.dev_addr);
    return 0;
}

void rs485_tx_set(uint8_t en)
{
    IIC_DeviceWriteBit(&IIC1_PCF8574,RS485_RE_IO, en);
}

/**
 * @brief       设置PCF8574某个IO的高低电平
 * @param       dev    : IIC设备实例指针
 * @param       bit    : 要设置的IO编号,0~7
 * @param       sta    : IO的状态;0或1
 * @retval      无
 */
void IIC_DeviceWriteBit(IIC_Device_t *dev, uint8_t bit, uint8_t sta)
{
    uint8_t data = 0;

    IIC_DeviceReadByte(dev, &data);          /* 先读出原来的设置 */

    if (sta == 0)
    {
        data &= ~(1 << bit);
    }
    else
    {
        data |= 1 << bit;
    }

    IIC_DeviceWriteByte(dev, data);            /* 写入新的数据 */
}

/**
 * @brief       读取PCF8574的某个IO的值
 * @param       dev     : IIC设备实例指针
 * @param       bit     : 要读取的IO编号, 0~7
 * @retval      此IO口的值(状态, 0/1)
 */
uint8_t IIC_DeviceReadBit(IIC_Device_t *dev, uint8_t bit)
{
    uint8_t data;

    IIC_DeviceReadByte(dev, &data);          /* 先读取这个8位IO的值  */

    if (data & (1 << bit))
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
 * @param       dev     : IIC设备实例指针
 * @param       val     : 读取到的值
 * @retval      0, 成功;
                1, 失败;    
 */
uint8_t IIC_DeviceReadByte(IIC_Device_t *dev, uint8_t *val)
{ 
    IIC_Start(dev);
    IIC_WriteByte(dev, dev->dev_addr | 0X01);
    IIC_ReadByte(dev, 0, val); // 读取数据
    IIC_Stop(dev);               /* 产生一个停止条件 */
    return 0;
}

/**
 * @brief       向PCF8574写入一个字节
 * @param       dev     : IIC设备实例指针
 * @param       data    : 要写入的数据
 * @retval      无
 */
void IIC_DeviceWriteByte(IIC_Device_t *dev, uint8_t data)
{
    IIC_Start(dev);  
    IIC_WriteByte(dev, dev->dev_addr | 0X00);   /* 发送器件地址0X40,写数据 */
    IIC_WriteByte(dev, data);                  /* 发送字节 */
    IIC_Stop(dev);                           /* 产生一个停止条件  */
    delay_ms(10); 
}

