#ifndef __LED_H
#define __LED_H

#include "./SYSTEM/Inc/sys.h"

/* 引脚定义 */
#define LED0_GPIO_PORT              GPIOB
#define LED0_GPIO_PIN               GPIO_PIN_1
#define LED0_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define LED1_GPIO_PORT              GPIOB
#define LED1_GPIO_PIN               GPIO_PIN_0
#define LED1_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

/* LED端口定义 */
#define LED0(x)     do{ x ? \
                        HAL_GPIO_WritePin(LED0_GPIO_PORT, LED0_GPIO_PIN, GPIO_PIN_SET) : \
                        HAL_GPIO_WritePin(LED0_GPIO_PORT, LED0_GPIO_PIN, GPIO_PIN_RESET); \    
                    }while(0)   /* LED0 = RED */
                    
#define LED1(x)     do{ x ? \
                        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET) : \
                        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET); \    
                    }while(0)   /* LED1 = GREEN */
/* LED取反定义*/
#define LED0_TOGGLE()    do{ HAL_GPIO_TogglePin(LED0_GPIO_PORT, LED0_GPIO_PIN); }while(0)       /* LED0 = !LED0 */
#define LED1_TOGGLE()    do{ HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN); }while(0)  

void led_init(void);

// key.h
/* 引脚定义*/
#define KEY0_GPIO_PORT          GPIOH
#define KEY0_GPIO_PIN           GPIO_PIN_3
#define KEY0_GPIO_CLK_ENABLE    do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0);
#define KEY0_IRQn               EXTI3_IRQn
#define KEY0_IRQHandler         EXTI3_IRQHandler

#define KEY1_GPIO_PORT          GPIOH
#define KEY1_GPIO_PIN           GPIO_PIN_2
#define KEY1_GPIO_CLK_ENABLE    do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while (0);
#define KEY1_IRQn               EXTI2_IRQn
#define KEY1_IRQHandler         EXTI2_IRQHandler

#define KEY2_GPIO_PORT          GPIOC
#define KEY2_GPIO_PIN           GPIO_PIN_13
#define KEY2_GPIO_CLK_ENABLE    do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0);
#define KEY2_IRQn               EXTI15_10_IRQn
#define KEY2_IRQHandler         EXTI15_10_IRQHandler

#define WKUP_GPIO_PORT          GPIOA
#define WKUP_GPIO_PIN           GPIO_PIN_0
#define WKUP_GPIO_CLK_ENABEL    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0);
#define WKUP_IRQn               EXTI0_IRQn
#define WKUP_IRQHandler         EXTI0_IRQHandler

/* 读引脚*/
#define KEY0        HAL_GPIO_ReadPin(KEY0_GPIO_PORT, KEY0_GPIO_PIN)
#define KEY1        HAL_GPIO_ReadPin(KEY1_GPIO_PORT, KEY1_GPIO_PIN)
#define KEY2        HAL_GPIO_ReadPin(KEY2_GPIO_PORT, KEY2_GPIO_PIN)
#define WK_UP       HAL_GPIO_ReadPin(WKUP_GPIO_PORT, WKUP_GPIO_PIN)


#define KEY0_PRES   1
#define KEY1_PRES   2
#define KEY2_PRES   3
#define WKUP_PRES   4

void key_init(void);

