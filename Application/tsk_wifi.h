/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-07-22 23:42:51
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-07-23 20:56:52
 * @FilePath: \Demo_Std_F407\Application\tsk_wifi.h
 * @Description: 
 * 
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved. 
 */
#ifndef __TSK_WIFI_H
#define __TSK_WIFI_H
#include "pch.h"
typedef enum {
    COMM_TCP,
    COMM_BLE
} Comm_Channel_t;



typedef enum {
    INIT_STAGE_DEVICE_TREE = 0,   // 设备树
    INIT_STAGE_PROP_TREE   = 1,   // 属性树
    INIT_STAGE_TIME_SYNC   = 2,   // 时间同步
    INIT_STAGE_COUNT              // 阶段总数（可用于循环）
} InitStageIndex_t;

typedef struct {
    uint8_t  stage_index;      // 阶段索引，用于准备发送数据
    const char *ack_keyword;   // 接收时匹配的ACK关键字
} InitStageDynamic_t;

typedef struct {
    uint8_t ready;                // ESP32是否就绪
    uint8_t ready_retry_count;    // 就绪检测重试次数
    uint8_t wifi_init_retry_count;
    uint8_t ble_init_retry_count;
    uint8_t tcp_init_retry_count;
} ESP32Status_t;


/**
 * @brief 通信通道类型定义
 */
void Handle_Comm_Channel(Comm_Channel_t channel, uint32_t interval_ms);

/**
 * @brief 多阶段动态发送流程处理，定时调用发送当前阶段数据。
 * 
 * @param buf          用于存放发送数据的缓冲区，调用者保证大小足够。
 * @param buf_size     发送缓冲区大小。
 * @param stages       初始化阶段数组，以 {stage_index, ack_keyword} 形式。
 * @param current_stage 当前阶段索引指针，函数内部更新。
 * @param is_done      初始化完成标志指针，完成后置1。
 * @param comm_type    发送使用的通信通道（WiFi或BLE）。
 * @param tag          日志标签字符串，方便调试。
 */
void ServerCommProcess(uint8_t *buf, uint16_t buf_size,
                       const InitStageDynamic_t *stages,
                       uint8_t *current_stage,
                       uint8_t *is_done,
                       ESP32_Comm_Type_t comm_type,
                       const char *tag);

/**
 * @brief 处理接收到的字符串中是否包含当前阶段的ACK关键词，匹配成功推进阶段。
 * 
 * @param stages        初始化阶段数组。
 * @param current_stage 当前阶段索引指针。
 * @param is_done      初始化完成标志指针。
 * @param recv_str     接收到的字符串数据。
 * @param tag          日志标签。
 */
void ServerCommProcess_HandleAck(const InitStageDynamic_t *stages,
                                uint8_t *current_stage,
                                uint8_t *is_done,
                                const char *recv_str,
                                const char *tag);

/**
 * @brief 根据阶段索引准备对应的发送数据，写入buf，返回数据长度。
 * 
 * @param stage_index 阶段索引
 * @param out_buf    输出缓冲区指针
 * @param max_len    缓冲区大小
 * @return uint16_t  返回写入数据长度
 */
uint16_t Prepare_Init_Stage_Data(uint8_t stage_index, uint8_t *out_buf, uint16_t max_len);

#ifdef __cplusplus
}
#endif
                                
#endif
        
                                