/**
 * =====================================================================================
 * @file        protocol_handler.h
 * @brief       通用协议解析器头文件，提供RS485帧处理功能。
 * @author      23Elapse & Gemini
 * @version     2.0 (Refactored)
 * @date        2025-06-14
 * @note        本模块处理协议帧的打包和解包，提供可重入的字节流解析器。
 * =====================================================================================
 */
#ifndef __PROTOCOL_HANDLER_H
#define __PROTOCOL_HANDLER_H

#include "stdint.h"
#include "stdbool.h" // For bool type

// 定义协议帧的固定长度和信息域最大长度
#define PROTOCOL_MIN_FRAME_LEN  (8U)  // 最小帧长度 (SOF + ADDR1 + ADDR2 + CMD + CMD_SUB + LEN + CRC_L + CRC_H)
#define PROTOCOL_MAX_INFO_LEN   (200U) // 信息域最大长度
#define PROTOCOL_MAX_FRAME_LEN  (PROTOCOL_MIN_FRAME_LEN + PROTOCOL_MAX_INFO_LEN) // 最大帧长度
#define PROTOCOL_CRC_LEN        (2U)  // CRC长度 (2字节)

/**
 * @brief 协议帧头定义 (示例)
 * @note  根据实际协议修改
 */
typedef enum
{
    FRAME_HEADER_7E = 0x7E, // 帧头示例1
    FRAME_HEADER_F6 = 0xF6, // 帧头示例2
    FRAME_HEADER_F7 = 0xF7,
    FRAME_HEADER_52 = 0x52
} Protocol_FrameHeader_t;

/**
 * @brief 协议处理状态码
 */
typedef enum {
    PROTOCOL_OK = 0,            // 操作成功 / 完整帧已解析
    PROTOCOL_IN_PROGRESS,       // 协议解析进行中，等待更多数据
    PROTOCOL_ERROR_CRC,         // CRC校验失败
    PROTOCOL_ERROR_FRAME,       // 帧格式错误 (如长度异常, 非法字符等)
    PROTOCOL_ERROR_PARAM,       // 参数错误
    PROTOCOL_ERROR_BUFFER_FULL  // 内部缓冲区不足
} Protocol_Status_t;

/**
 * @brief RS485协议帧结构体
 * @note  根据您的协议定义，这是标准RS485帧的结构。
 */
typedef struct
{
    uint8_t sof;        // 帧起始符
    uint8_t addr1;      // 地址1
    uint8_t addr2;      // 地址2
    uint8_t cmd;        // 命令码
    uint8_t cmd_sub;    // 子命令码
    uint8_t length;     // 信息域长度 (在发送时计算并填充，在接收时表示整个帧的长度)
    uint8_t *info;      // 信息域数据指针
    uint16_t info_len;  // 信息域实际长度
    uint16_t crc;       // CRC校验码
} RS485_Frame_t;

/**
 * @brief 通用协议数据结构体
 * @note  用于统一存储不同协议解析后的数据。
 * rs485_frame.info 指向 uart_data.data 缓冲区，减少内存占用。
 */
typedef struct
{
    union
    {
        RS485_Frame_t rs485_frame; // RS485协议帧数据
        struct
        {
            uint8_t data[PROTOCOL_MAX_FRAME_LEN]; // 通用数据缓冲区，也用于RS485信息域
            uint32_t length;                      // 通用数据长度
        } uart_data; // 非RS485通用串口数据
    };
    bool is_rs485; // 标志位：true表示当前数据为RS485帧，false表示为通用UART数据
} Protocol_Data_t;

// 协议解析器内部状态枚举
typedef enum {
    PROTOCOL_PARSE_STATE_WAIT_SOF = 0, // 等待帧头
    PROTOCOL_PARSE_STATE_READ_FIXED_HEADER, // 读取固定头部分 (地址, 命令, 长度)
    PROTOCOL_PARSE_STATE_READ_INFO,         // 读取信息域
    PROTOCOL_PARSE_STATE_READ_CRC           // 读取CRC
} Protocol_ParseState_t;

/**
 * @brief 协议解析器的内部状态和缓冲区上下文
 * @note  为了使Protocol_ProcessByte可重入，其所有状态都应通过此上下文传入。
 */
typedef struct {
    uint8_t             temp_buffer[PROTOCOL_MAX_FRAME_LEN]; // 临时接收缓冲区，用于构建完整帧
    uint32_t            current_idx;       // 当前解析到的索引
    Protocol_ParseState_t parse_state;     // 当前解析状态
    uint8_t             expected_len;      // 期望的帧总长度
    Protocol_Data_t     parsed_data;       // 存储解析出的数据 (RS485或UART)
} Protocol_ParserContext_t;


/**
 * @brief Modbus CRC16 计算函数
 * @param data 输入数据缓冲区
 * @param length 数据长度
 * @return 计算得到的CRC16值
 */
uint16_t Modbus_CRC16(const uint8_t *data, uint32_t length);

/**
 * @brief 协议模块初始化
 * @note  执行任何全局或一次性初始化。
 */
void Protocol_Init(void);

/**
 * @brief 将RS485帧结构体打包成字节流，并计算CRC。
 * @param frame_in 指向要打包的RS485帧结构体
 * @param buffer_out 存储打包后字节流的缓冲区
 * @param max_buffer_len 缓冲区最大长度
 * @param packed_len_out 打包后实际字节长度输出
 * @return Protocol_Status_t 操作状态 (PROTOCOL_OK 或错误码)
 */
Protocol_Status_t Protocol_PackRS485Frame(const RS485_Frame_t *frame_in, uint8_t *buffer_out, uint32_t max_buffer_len, uint32_t *packed_len_out);

/**
 * @brief 逐字节解析接收到的数据，构建协议帧。
 * @param context 指向协议解析上下文，用于维护状态和存储中间数据。
 * @param byte_in 接收到的单个字节
 * @param data_out 指向存储解析结果的 Protocol_Data_t 结构体。如果解析成功，此结构体将被填充。
 * @return Protocol_Status_t 解析状态 (PROTOCOL_OK表示一帧完成，PROTOCOL_IN_PROGRESS表示正在解析中，其他为错误)
 * @note  此函数是可重入的，通过传入 context 来管理状态。
 */
Protocol_Status_t Protocol_ProcessByte(Protocol_ParserContext_t *context, uint8_t byte_in, Protocol_Data_t *data_out);

/**
 * @brief 重置协议解析器的状态。
 * @param context 指向要重置的协议解析上下文。
 */
void Protocol_ResetParser(Protocol_ParserContext_t *context);

#endif /* __PROTOCOL_HANDLER_H */
