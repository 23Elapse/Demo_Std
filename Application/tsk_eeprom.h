/**
 * =====================================================================================
 * @file        tsk_eeprom.h
 * @brief       EEPROM 参数管理模块头文件 (Refactored)
 * @author      23Elapse & Gemini
 * @version     2.2 (Refactored & Encapsulated)
 * @date        2025-06-14
 * @note        本模块负责所有需要掉电保存的参数的管理。
 * 外部模块只需调用 Tsk_Eeprom_Init() 即可启动该服务。
 * =====================================================================================
 */
#ifndef __TSK_EEPROM_H
#define __TSK_EEPROM_H

#include "rtos_abstraction.h" // 间接包含FreeRTOS等
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief EEPROM 参数管理模块错误码
 */
typedef enum
{
    EEPROM_MGR_OK = 0,
    EEPROM_MGR_ERROR,
    EEPROM_MGR_INVALID_ID,
    EEPROM_MGR_INVALID_PARAM,
    EEPROM_MGR_LOCK_TIMEOUT,
    EEPROM_MGR_INIT_FAILED, // 新增初始化失败状态
} EepromMgr_Status_t;

/**
 * @brief 初始化EEPROM参数管理模块。
 * @note  此函数会完成所有参数的初始化（从EEPROM加载或使用默认值），
 * 并自动创建后台监控任务，用于定期保存“脏”数据。
 * 应在RTOS调度器启动前调用。
 * @return EepromMgr_Status_t 初始化状态。
 */
EepromMgr_Status_t Tsk_Eeprom_Init(void);

/**
 * @brief 恢复所有参数到出厂默认设置。
 * @note  这是一个阻塞操作，会立即将所有可恢复的参数写入EEPROM。
 * @return EepromMgr_Status_t 操作状态。
 */
EepromMgr_Status_t Tsk_Eeprom_FactoryReset(void);

/**
 * @brief 通过ID获取一个参数的当前值。
 * @param id  要获取的参数ID (定义在 g_eeprom_table 中)。
 * @param p_value 指向用于存储参数值的变量。
 * @return EepromMgr_Status_t 操作状态。
 */
EepromMgr_Status_t Tsk_Eeprom_GetParam(uint16_t id, uint16_t* p_value);

/**
 * @brief 通过ID设置一个参数的值。
 * @note  此函数只会更新RAM中的值并标记为“脏数据”。
 * 后台任务会自动将其回写到EEPROM。
 * @param id 要设置的参数ID。
 * @param value 要设置的新值。
 * @return EepromMgr_Status_t 操作状态。
 */
EepromMgr_Status_t Tsk_Eeprom_SetParam(uint16_t id, uint16_t value);

#endif /* __TSK_EEPROM_H */
