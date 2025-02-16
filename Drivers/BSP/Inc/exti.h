/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-27 21:17:23
 * @LastEditors: Elapse userszy@163.com
 * @LastEditTime: 2024-10-27 21:18:25
 * @FilePath: \Demo\Drivers\BSP\Inc\exti.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __EXTI_H
#define __EXTI_H
#include "pch.h"

typedef struct {
    uint32_t RCC_AHB1Periph_GPIOx;  // GPIO 时钟使能
    GPIO_TypeDef *port;           // GPIO 端口，例如 GPIOA, GPIOB, GPIOC 等
    uint16_t pin;                 // 引脚编号
    GPIOMode_TypeDef mode;       // 引脚模式
    GPIOOType_TypeDef otype;    // 输出类型：推挽输出或开漏输出
    GPIOPuPd_TypeDef pull;       // 上拉/下拉配置
    GPIOSpeed_TypeDef speed;     // 引脚速度
} GPIO_Config_TypeDef;
typedef struct {
    uint32_t EXTI_Line;                 // EXTI 线路
    EXTIMode_TypeDef EXTI_Mode;        // 中断模式
    EXTITrigger_TypeDef EXTI_Trigger;   // 触发方式
    FunctionalState EXTI_LineCmd;       // 线路使能
    uint8_t EXTI_PortSourceGPIOx;       // GPIO 端口源
    uint8_t EXTI_PinSourcex;            // 引脚源
} EXTI_Config_TypeDef;

typedef struct {
    uint8_t NVIC_IRQChannel;                        // 中断编号
    uint8_t NVIC_IRQChannelPreemptionPriority;      // 抢占优先级
    uint8_t NVIC_IRQChannelSubPriority;             // 子优先级
    FunctionalState NVIC_IRQChannelCmd;             //中断使能       
} NVIC_Config_TypeDef;

/******************************************************************************************/
/* 引脚 和 中断编号 & 中断服务函数 定义 */ 

#define KEY0_INT_GPIO_PORT              GPIOH
#define KEY0_INT_GPIO_PIN               GPIO_Pin_3
#define KEY0_INT_GPIO_CLK_ENABLE()      do{ RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE) }while(0);   /* PH口时钟使能 */
#define KEY0_INT_IRQn                   EXTI3_IRQn
#define KEY0_INT_IRQHandler             EXTI3_IRQHandler
#define KEY0_INT_EXTI_LINE              EXTI_Line3
#define KEY0_INT_EXTI_PORTSOURCE        EXTI_PortSourceGPIOH
#define KEY0_INT_EXTI_PINSOURCE         EXTI_PinSource3

#define KEY1_INT_GPIO_PORT              GPIOH
#define KEY1_INT_GPIO_PIN               GPIO_Pin_2
#define KEY1_INT_GPIO_CLK_ENABLE()      do{ RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE) }while(0);   /* PH口时钟使能 */
#define KEY1_INT_IRQn                   EXTI2_IRQn
#define KEY1_INT_IRQHandler             EXTI2_IRQHandler
#define KEY1_INT_EXTI_LINE              EXTI_Line2
#define KEY1_INT_EXTI_PORTSOURCE        EXTI_PortSourceGPIOH
#define KEY1_INT_EXTI_PINSOURCE         EXTI_PinSource2

#define KEY2_INT_GPIO_PORT              GPIOC
#define KEY2_INT_GPIO_PIN               GPIO_Pin_13
#define KEY2_INT_GPIO_CLK_ENABLE()      do{ RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE) }while(0);   /* PC口时钟使能 */
#define KEY2_INT_IRQn                   EXTI15_10_IRQn
#define KEY2_INT_IRQHandler             EXTI15_10_IRQHandler
#define KEY2_INT_EXTI_LINE              EXTI_Line13
#define KEY2_INT_EXTI_PORTSOURCE        EXTI_PortSourceGPIOC
#define KEY2_INT_EXTI_PINSOURCE         EXTI_PinSource13

#define WKUP_INT_GPIO_PORT              GPIOA
#define WKUP_INT_GPIO_PIN               GPIO_Pin_0
#define WKUP_INT_GPIO_CLK_ENABLE()      do{ RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE) }while(0);   /* PA口时钟使能 */
#define WKUP_INT_IRQn                   EXTI0_IRQn
#define WKUP_INT_IRQHandler             EXTI0_IRQHandler
#define WKUP_INT_EXTI_LINE              EXTI_Line0
#define WKUP_INT_EXTI_PORTSOURCE        EXTI_PortSourceGPIOA
#define WKUP_INT_EXTI_PINSOURCE         EXTI_PinSource0



/******************************************************************************************/

void extix_init(void);           /* 外部中断初始化 */

#endif


