/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-27 21:32:31
 * @LastEditors: Elapse userszy@163.com
 * @LastEditTime: 2024-10-27 21:32:55
 * @FilePath: \Demo\Drivers\BSP\Src\wdg.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pch.h"


/**
 * @brief       初始化独立看门狗 
 * @param       prer: IWDG_PRESCALER_4~IWDG_PRESCALER_256,对应4~256分频
 *   @arg       分频因子 = 4 * 2^prer. 但最大值只能是256!
 * @param       rlr: 自动重装载值,0~0XFFF. 
 * @note        时间计算(大概):Tout=((4 * 2^prer) * rlr) / 32 (ms). 
 * @retval      无
 */
// 独立看门狗初始化函数
void IWDG_Init(uint8_t prescaler, uint16_t reload) {
    // 1. 使能 IWDG（启动独立看门狗）
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  // 允许写入 IWDG 寄存器
    IWDG_Enable();  // 启动独立看门狗

    // 2. 设置预分频器
    IWDG_SetPrescaler(prescaler);  // 设置预分频器

    // 3. 设置重装载值
    IWDG_SetReload(reload);  // 设置重装载值

    // 4. 重载计数器（喂狗）
    IWDG_ReloadCounter();  // 重置计数器

    // 5. 等待寄存器更新完成
    while (IWDG_GetFlagStatus(IWDG_FLAG_PVU)) {}  // 等待预分频器更新完成
    while (IWDG_GetFlagStatus(IWDG_FLAG_RVU)) {}  // 等待重装载值更新完成
}

// 喂狗函数
void IWDG_Feed(void) {
    IWDG_ReloadCounter();  // 重置计数器
}

