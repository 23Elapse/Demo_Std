/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-11-16 17:12:04
 * @LastEditors: Elapse userszy@163.com
 * @LastEditTime: 2024-11-16 17:12:20
 * @FilePath: \Demo\Drivers\BSP\Inc\sdram.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
/**
 ****************************************************************************************************
 * @file        sdram.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-4-20
 * @brief       SDRAM 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 */

#ifndef _SDRAM_H
#define _SDRAM_H

#include "./SYSTEM/Inc/sys.h"


/******************************************************************************************/

extern SDRAM_HandleTypeDef  g_sdram_handle;           /* SDRAM句柄 */
#define BANK5_SDRAM_ADDR    ((uint32_t)(0XC0000000))  /* SDRAM开始地址 */

/* SDRAM配置参数 */
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

/******************************************************************************************/

void sdram_init(void);
uint8_t sdram_send_cmd(uint8_t bankx, uint8_t cmd, uint8_t refresh, uint16_t regval);
void fmc_sdram_write_buffer(uint8_t *pbuf, uint32_t addr, uint32_t n);
void fmc_sdram_read_buffer(uint8_t *pbuf, uint32_t addr, uint32_t n);
void sdram_initialization_sequence(void);

#endif
