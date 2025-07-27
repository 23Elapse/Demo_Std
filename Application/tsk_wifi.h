/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-07-22 23:42:51
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-07-26 18:15:25
 * @FilePath: \Demo_Std_F407\Application\tsk_wifi.h
 * @Description:
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __TSK_WIFI_H
#define __TSK_WIFI_H
#include "pch.h"
// 发送周期（单位ms）
#define REALTIME_DATA_INTERVAL_MS 20000
#define OFFLINE_DATA_INTERVAL_MS 20000
static uint8_t wifi_send_buf[5000]; // 共享缓冲区
typedef enum
{
    COMM_TCP,
    COMM_BLE
} Comm_Channel_t;
typedef union
{
    struct
    {
        uint8_t wifi_app_init : 1;
        uint8_t ble_app_init : 1;
        uint8_t tcp_app_init : 1;
        uint8_t reserved : 5;
    } bits;
    uint8_t all;
} AppInitFlags_t;


typedef struct
{
    uint8_t ready;             // ESP32是否就绪
    uint8_t ready_retry_count; // 就绪检测重试次数
    uint8_t wifi_init_retry_count;
    uint8_t ble_init_retry_count;
    uint8_t tcp_init_retry_count;
} ESP32Status_t;

// 初始化阶段枚举
typedef enum {
    INIT_INDEX_DEVICE_TREE = 0,
    INIT_INDEX_PROP_TREE,
    INIT_INDEX_OFFLINE_TREE,
    INIT_INDEX_TIME_SYNC,
    INIT_INDEX_HEARTBEAT,
    INIT_INDEX_COUNT
} InitStageIndex_t;

// 位掩码定义
typedef enum {
    INIT_STAGE_MASK_DEVICE_TREE   = (1 << INIT_INDEX_DEVICE_TREE),
    INIT_STAGE_MASK_PROP_TREE     = (1 << INIT_INDEX_PROP_TREE),
    INIT_STAGE_MASK_OFFLINE_TREE  = (1 << INIT_INDEX_OFFLINE_TREE),
    INIT_STAGE_MASK_TIME_SYNC     = (1 << INIT_INDEX_TIME_SYNC),
    INIT_STAGE_MASK_HEARTBEAT     = (1 << INIT_INDEX_HEARTBEAT),
    INIT_STAGE_MASK_ALL           = (1 << INIT_INDEX_COUNT) - 1
} InitStageMask_t;

// 初始化控制结构
typedef struct
{
    InitStageMask_t mask;           // 启用的阶段掩码
    InitStageMask_t finished_mask;  // 已完成的阶段掩码
    InitStageIndex_t current_stage; // 当前阶段索引
    uint8_t completed;              // 当前阶段是否已完成（修改：只表示当前阶段完成）
    InitStageMask_t trigger_mask; // 触发重新发送的阶段掩码（可动态置位触发）
} InitPhaseControl_t;

// 定时发送控制结构体
typedef struct
{
    uint32_t last_realtime_tick;   // 上次发送实时数据的时间戳（单位ms）
    uint32_t last_history_tick;    // 上次发送历史数据的时间戳（单位ms）
    uint32_t realtime_interval_ms; // 实时数据发送间隔
    uint32_t history_interval_ms;  // 历史数据发送间隔
} PeriodicSendControl_t;

typedef void (*InitStageSendFunc)(void);

typedef struct {
    InitStageMask_t stage_mask;
    InitStageIndex_t stage_index;
    InitStageSendFunc send_func;
} InitStageEntry_t;

extern InitPhaseControl_t init_ctrl; // 声明全局控制结构体
#ifdef __cplusplus
}
#endif

#endif
