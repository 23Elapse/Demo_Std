#ifndef __IIC_CORE_H
#define __IIC_CORE_H

#include "pch.h"

#ifdef IIC_DEBUG
#define LOG_IIC_EVENT(IICx, event) printf("[IIC%d] %s\n", IICx->instance_id, event)
#else
#define LOG_IIC_EVENT(IICx, event)
#endif


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
#define SPI_FLASH_WAIT_UNTIL_TIMEOUT(condition)	WAIT_UNTIL_TIMEOUT(condition, 30000);

// /**
//  * @brief   等待condition等条件表达式成立，或超时，才退出
//  * @param   condition 等待条件
//  * @param   timeout Timeout duration
//  * @retval  none
//  */
// #define WAIT_UNTIL_TIMEOUT(condition, timeout) \
// ({ \
//     int result = 0; \
//     uint32_t tickstart = GetTick(); \
//     /* Wait until flag is set */ \
//     while (!(condition)) \
//     { \
//         if (timeout != SYSTICK_MAX_DELAY) \
//         { \
//             if ((timeout == 0) || ((GetTick() - tickstart) > timeout)) \
//             { \
//                 if (timeout != 0) \
//                 { \
//                     TBB_WARN("WAIT_UNTIL_TIMEOUT is timeout"); \
//                 } \
//                 result = -1; \
//                 break; \
//             } \
//         } \
//     } \
//     result; \
// })

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

//SPI_WAIT_UNTIL_TIMEOUT(RESET != spi_i2s_flag_get(spi_periph,SPI_FLAG_TBE));
typedef enum {
    IIC_OK = 0,
    IIC_ERR_INIT,
    IIC_ERR_TIMEOUT,
    IIC_ERR_NACK,
    IIC_ERR_BUS_BUSY
} IIC_Status;

// typedef struct IIC_Config_t IIC_Config_t; // 前向声明
typedef struct IIC_Config_t{
    GPIO_TypeDef* scl_port;   // SCL引脚端口
    GPIO_TypeDef* sda_port;   // SDA引脚端口
    uint16_t scl_pin;         // SCL引脚
    uint16_t sda_pin;         // SDA引脚
    uint8_t instance_id;      // IIC实例ID
}IIC_Config_t;


extern IIC_Config_t IIC1_config ;  // IIC1设备实例

/* 操作接口函数指针类型 */
typedef IIC_Status (*IIC_ReadFunc)(struct IIC_Config_t*, uint8_t reg, uint8_t* val);
typedef IIC_Status (*IIC_WriteFunc)(struct IIC_Config_t*, uint8_t reg, uint8_t val);

/* 统一操作接口结构体 */
typedef struct {
    IIC_ReadFunc    ReadByte;
    IIC_WriteFunc   WriteByte;
    uint8_t dev_addr;          // 设备地址（7位）
} IIC_Ops_t;

void IIC_Start(IIC_Config_t *IICx);
void IIC_Stop(IIC_Config_t *IICx);
IIC_Status IICx_Init(IIC_Config_t *IICx);
IIC_Status IIC_ReadByte(IIC_Config_t *IICx, uint8_t ack, uint8_t *data);
IIC_Status IIC_WriteByte(IIC_Config_t *IICx, uint8_t data);
void IIC_INIT(void);
void IIC_ResetBus(IIC_Config_t *IICx);   

#endif
