#ifndef __BTIM_H
#define __BTIM_H


/******************************************************************************************/
/* 基本定时器 定义 */

/* TIMX 中断定义 
 * 默认是针对TIM6/TIM7
 */
 
#define BTIM_TIMX_INT                       TIM6
#define BTIM_TIMX_INT_IRQn                  TIM6_DAC_IRQn
#define BTIM_TIMX_INT_IRQHandler            TIM6_DAC_IRQHandler
#define BTIM_TIMX_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM6_CLK_ENABLE(); }while(0)  /* TIM6 时钟使能 */

/******************************************************************************************/

void TIM6_Init(uint16_t arr, uint16_t psc);                                    /* 基本定时器 定时中断初始化函数 */

#endif

















