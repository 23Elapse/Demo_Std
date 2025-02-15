#ifndef __KEY_H
#define __KEY_H

#include "sys.h"

/******************************************************************************************/
/* 引脚定义 */

#define KEY0_GPIO_PORT                  GPIOH
#define KEY0_GPIO_PIN                   GPIO_PIN_3
#define KEY0_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)             /* PH口时钟使能 */

#define KEY1_GPIO_PORT                  GPIOH
#define KEY1_GPIO_PIN                   GPIO_PIN_2
#define KEY1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)             /* PH口时钟使能*/

#define KEY2_GPIO_PORT                  GPIOC
#define KEY2_GPIO_PIN                   GPIO_PIN_13
#define KEY2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)             /* PC口时钟使能 */

#define WKUP_GPIO_PORT                  GPIOA
#define WKUP_GPIO_PIN                   GPIO_PIN_0
#define WKUP_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)             /* PA口时钟使能*/
/******************************************************************************************/

#define KEY0        HAL_GPIO_ReadPin(KEY0_GPIO_PORT, KEY0_GPIO_PIN)     /* 读取KEY0引脚 */
#define KEY1        HAL_GPIO_ReadPin(KEY1_GPIO_PORT, KEY1_GPIO_PIN)     /* 读取KEY1引脚 */
#define KEY2        HAL_GPIO_ReadPin(KEY2_GPIO_PORT, KEY2_GPIO_PIN)     /* 读取KEY2引脚 */
#define WK_UP       HAL_GPIO_ReadPin(WKUP_GPIO_PORT, WKUP_GPIO_PIN)     /* 读取WKUP引脚 */

#define KEY0_PRES    1              /* KEY0按下 */
#define KEY1_PRES    2              /* KEY1按下 */
#define KEY2_PRES    3              /* KEY2按下 */
#define WKUP_PRES    4              /* KEY_UP按下(即WK_UP) */

/******************************************************************************************/
/* 外部接口函数*/
void key_init(void);      
uint8_t key_scan(uint8_t mode);         //按键扫描函数
#endif
