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

#include "./SYSTEM/Inc/sys.h"

/******************************************************************************************/
/* 引脚 和 中断编号 & 中断服务函数 定义 */ 

#define KEY0_INT_GPIO_PORT              GPIOH
#define KEY0_INT_GPIO_PIN               GPIO_PIN_3
#define KEY0_INT_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)   /* PH口时钟使能 */
#define KEY0_INT_IRQn                   EXTI3_IRQn
#define KEY0_INT_IRQHandler             EXTI3_IRQHandler

#define KEY1_INT_GPIO_PORT              GPIOH
#define KEY1_INT_GPIO_PIN               GPIO_PIN_2
#define KEY1_INT_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)   /* PH口时钟使能 */
#define KEY1_INT_IRQn                   EXTI2_IRQn
#define KEY1_INT_IRQHandler             EXTI2_IRQHandler

#define KEY2_INT_GPIO_PORT              GPIOC
#define KEY2_INT_GPIO_PIN               GPIO_PIN_13
#define KEY2_INT_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)   /* PC口时钟使能 */
#define KEY2_INT_IRQn                   EXTI15_10_IRQn
#define KEY2_INT_IRQHandler             EXTI15_10_IRQHandler

#define WKUP_INT_GPIO_PORT              GPIOA
#define WKUP_INT_GPIO_PIN               GPIO_PIN_0
#define WKUP_INT_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */
#define WKUP_INT_IRQn                   EXTI0_IRQn
#define WKUP_INT_IRQHandler             EXTI0_IRQHandler

/******************************************************************************************/

void extix_init(void);           /* 外部中断初始化 */

#endif


