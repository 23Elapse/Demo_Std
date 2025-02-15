/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-26 15:40:20
 * @LastEditors: Elapse userszy@163.com
 * @LastEditTime: 2024-12-14 16:11:41
 * @FilePath: \Demo\Drivers\SYSTEM\Inc\sys.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
/**
 ****************************************************************************************************
 * @file        sys.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-04-20
 * @brief       系统初始化代码(包括时钟配置/中断管理/GPIO设置等)
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 阿波罗 F429开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 第一次发布
 * V1.0 20220420

 ****************************************************************************************************
 */

#ifndef _SYS_H
#define _SYS_H

#include "stm32f4xx.h"
#include "core_cm4.h"



/**
 * SYS_SUPPORT_OS用于定义系统文件夹是否支持OS
 * 0,不支持OS
 * 1,支持OS
 */
#define SYS_SUPPORT_OS         1


/*函数声明*******************************************************************************************/

void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset);                         /* 设置中断偏移量 */
void sys_standby(void);                                                                     /* 进入待机模式 */
void sys_soft_reset(void);                                                                  /* 系统软复位 */
uint8_t SystemClock_Config(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq);   /* 配置系统时钟 */


/* 以下为汇编函数 */
void sys_wfi_set(void);             /* 执行WFI指令 */
void sys_intx_disable(void);        /* 关闭所有中断 */
void sys_intx_enable(void);         /* 开启所有中断 */
void sys_msr_msp(uint32_t addr);    /* 设置栈顶地址 */

#endif

