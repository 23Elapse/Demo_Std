#include "pch.h"
#include "delay.h"
static uint16_t  fac_us=0;							//us延时倍乘数			   
//static uint16_t fac_ms=0;							//ms延时倍乘数,在os下,代表每个节拍的ms数
void delay_init(uint16_t SYSCLK)
{
 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); 
	fac_us=SYSCLK/8;						//不论是否使用OS,fac_us都需要使用
//	fac_ms=(uint16_t)fac_us*1000;				//非OS下,代表每个ms需要的systick时钟数   

}	

//void delay_us(uint32_t nus)
//{
//    uint32_t ticks;
//    uint32_t told, tnow, tcnt = 0;
//    uint32_t reload = SysTick->LOAD;        /* LOAD的值 */
//    ticks = nus * g_fac_us;                 /* 需要的节拍数 */

//    told = SysTick->VAL;                    /* 刚进入时的计数器值 */
//    while (1)
//    {
//        tnow = SysTick->VAL;
//        if (tnow != told)
//        {
//            if (tnow < told)
//            {
//                tcnt += told - tnow;        /* 这里注意一下SYSTICK是一个递减的计数器就可以了 */
//            }
//            else
//            {
//                tcnt += reload - tnow + told;
//            }
//            told = tnow;
//            if (tcnt >= ticks) 
//            {
//                break;                      /* 时间超过/等于要延迟的时间,则退出 */
//            }
//        }
//    }
//} 

void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; 				//时间加载	  		 
	SysTick->VAL=0x00;        				//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; //关闭计数器
	SysTick->VAL =0X00;       				//清空计数器 
}

/**
  * @brief  启动系统滴答定时器 SysTick
  * @param  无
  * @retval 无
  */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
	if (SysTick_Config(SystemCoreClock / 100000))
	{ 
		/* Capture error */ 
		while (1);
	}
}
/**
 * @brief     延时nms
 * @param     nms: 要延时的ms数 (0< nms <= (2^32 / fac_us / 1000))(fac_us一般等于系统主频, 自行套入计算)
 * @retval    无
 */
void delay_ms(uint16_t nms)
{
    delay_us((uint32_t)(nms * 1000));                   /* 普通方式延时 */
}












