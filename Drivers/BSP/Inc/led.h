#ifndef __LED_H
#define __LED_H

#include "sys.h"

/******************************************************************************************/
/* 引脚定义 */

#define LED0_GPIO_PORT                  GPIOB
#define LED0_GPIO_PIN                   GPIO_Pin_1
#define LED0_GPIO_CLK_ENABLE()         do{ RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); }while(0)             /* PB口时钟使能 */
            /* PB口时钟使能 */

#define LED1_GPIO_PORT                  GPIOB
#define LED1_GPIO_PIN                   GPIO_Pin_0
#define LED1_GPIO_CLK_ENABLE()          do{ RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); }while(0)             /* PB口时钟使能*/

/******************************************************************************************/

/* LED端口定义 */
#define LED0(x)   do{ x ? \
                    GPIO_SetBits(LED0_GPIO_PORT, LED0_GPIO_PIN) : \
                    GPIO_ResetBits(LED0_GPIO_PORT, LED0_GPIO_PIN); \
                  }while(0)       /* LED0 = RED */

#define LED1(x)   do{ x ? \
                    GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN) : \
                    GPIO_ResetBits(LED1_GPIO_PORT, LED1_GPIO_PIN); \
                  }while(0)       /* LED1 = GREEN */

/* LED取反定义*/
#define LED0_TOGGLE()    do{ GPIO_ToggleBits(LED0_GPIO_PORT, LED0_GPIO_PIN); }while(0)       /* LED0 = !LED0 */
#define LED1_TOGGLE()    do{ GPIO_ToggleBits(LED1_GPIO_PORT, LED1_GPIO_PIN); }while(0)       /* LED1 = !LED1 */

/******************************************************************************************/
/* 外部接口函数*/
void led_init(void);                                                                            /* 初始化 */

#endif
