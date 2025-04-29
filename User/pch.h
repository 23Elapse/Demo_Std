/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-15 20:36:56
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-29 09:58:14
 * @FilePath: \Demo\User\pch.h
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef PCH_H
#define PCH_H

#include "stm32f4xx.h"
//#include "stm32f429xx.h"
#include "stm32f4xx_gpio.h" 
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_fsmc.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_cryp.h"
#include "stm32f4xx_hash.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_ltdc.h"
#include "stm32f4xx_sai.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dbgmcu.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_wwdg.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_lptim.h"
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"					//FreeRTOS使用		  
#include "task.h" 
#include <stdbool.h>
#include <string.h> // 用于 memcpy 和 memset
#include <stdint.h>
#include <stdio.h>
#include "sys.h"
#include <stdbool.h>
#include <stdint.h>
#include "semphr.h"
#include "ring_buffer.h"   
#include "usart.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "exti.h"
#include "wdg.h"
#include "btim.h"
// #include "spi.h"
#include "spi_flash.h"
// #include "myiic.h"
//#include "24cxx.h"
#include "iic_core.h"
//#include "my_rs485.h"
#include "pcf8574.h"
#include "freertos_demo.h"
#include "queue.h"
#include "tsk_wifi.h"
#include "api_wifi.h"
#include "semphr.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "api_wifi.h"
#include "api_eeprom.h"
// #include "api_flash.h"
#include "serial_driver.h"
#include "common_driver.h"
#include "api_eeprom.h"




#endif


