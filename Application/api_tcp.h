#ifndef __API_TCP_H
#define __API_TCP_H

#include "log_system.h"
#include "rtos_abstraction.h"
#include "serial_driver.h" // 依赖底层串口驱动

typedef enum {
    TCP_PACKET = 0,
    BLE_PACKET = 1
} PacketType_t;

typedef struct {
  PacketType_t source;    // 来源 (TCP/BLE)
  uint8_t type;      // 命令类型 (查询/设置)
  uint16_t cmd_id;   // 命令ID
  uint8_t data_len;  // 数据长度
  uint8_t *data;         // 数据指针
} CommPacket;


/* -------------------------------------------------------------------------- */
/*                                   DEFINES                                  */
/* -------------------------------------------------------------------------- */
#define NET_DECRYPTION_KEY_LEN                  4
#define NETWORK_PROTOCOL_VER                    0x2009
#define NETWORK_SYSTEM_TYPE                     0X03000000

#define NETWORK_MAX_PACK_LEN                    2000
#define NETWORK_MAX_SEND_LEN                    1000
#define NETWORK_MAX_ACK_LEN                     2000

#define NET_INQR_ADDR_LEN                       5
#define NET_CTRL_ADDR_LEN                       5
#define NET_CTRL_DATA_LEN                       255
/* -------------------------------------------------------------------------- */
/*                                TYPE DEFINES                                */
/* -------------------------------------------------------------------------- */
/**
 * @brief 解码器状态结构体，用于跨函数调用保持解码上下文。
 */
typedef struct {
    bool escape;  ///< 上一次是否遇到转义起始符（0xFD），用于判断当前是否处于转义状态
} Decoder;

/**
 * @brief 输入数据结构体，表示解码器当前要读取的原始字节流。
 */
typedef struct {
    const uint8_t *data;  ///< 指向输入数据缓冲区（原始数据）
    uint16_t length;      ///< 输入数据总长度（字节数）
    uint16_t offset;      ///< 当前已读取到的位置（偏移量）
} Input;

/**
 * @brief 输出数据结构体，表示解码器解码后的数据写入区域。
 */
typedef struct {
    uint8_t *data;        ///< 指向输出缓冲区（用于存放解码后数据）
    uint16_t capacity;    ///< 输出缓冲区总容量（可选，仅用于扩展，当前不检查溢出）
    uint16_t offset;      ///< 当前已写入的位置（偏移量）
} Output;

/* ----------------------------- net header ---------------------------- */
typedef enum net_header_type_t{
    NET_HEADER_TYPE_LONG = 0,
    NET_HEADER_TYPE_SHORT,
}net_header_type_t;
typedef enum net_header_len_t{
    NET_HEADER_LONG_LEN = 18,
    NET_HEADER_SHORT_LEN = 9,
}net_header_len_t;

typedef union net_header_t{
    struct st_header_long_t{
        uint8_t net_header_len[2];      // 网络头长度 (2字节)
        uint8_t seq[4];                 // 通信识别码 (4字节)
        uint8_t protocol_ver[2];        // 协议版本 (2字节)
        uint8_t sys_type[4];            // 系统类型 (4字节)
        uint8_t timestamp[4];           // 时间戳 (4字节)   
        uint8_t cmd_type[2];            // 命令类型 (2字节)   
    }st_header_long;
    struct st_header_short_t
    {
        uint8_t net_header_len[2];
        uint8_t seq;
        uint8_t timestamp[4];
        uint8_t cmd_type[2];
    }st_header_short;
}net_header_t;
/* ------------------------------ network pack ------------------------------ */
typedef enum net_unpack_result{
    net_data_recv_continue = 0,
    net_unpack_continue,
    net_unpack_complete,
    net_unpack_fail,
}net_unpack_result_t;
/* ------------------------------ network data ------------------------------ */
typedef struct net_data_info_t{
    net_header_t *pst_net_header_part;
    uint8_t *pnet_body_part;
}net_data_info_t;

typedef struct network_data_t{
    uint16_t data_len;
    uint8_t *pdata;
}network_data_t;

typedef bool (*network_send_func)(uint16_t ,uint8_t *);

/* ---------------------------- net protocol data --------------------------- */
typedef struct net_inquire_info{
    uint8_t addr_len;
    uint8_t device_id_s[3];
    uint8_t start_addr[2];
    uint8_t device_id_e[3];
    uint8_t end_addr[2];
}net_inquire_info_t;
typedef struct net_ctrl_info{
    uint8_t ctrl_addr_len;
    uint8_t ctrl_addr[NET_CTRL_ADDR_LEN];
    uint8_t ctrl_data_len;
    uint8_t ctrl_data[NET_CTRL_DATA_LEN];
}net_ctrl_info_t;
typedef union net_ctrl_data{
    uint8_t *pencryption_data;
    net_ctrl_info_t *pst_decryption_data;
}net_ctrl_data_t;
typedef net_ctrl_data_t net_set_data_t;

typedef struct net_list_operate{
    uint8_t device_id[2];
    uint8_t device_num;
    uint8_t rw_type;
    uint8_t start_addr[2];
    uint8_t end_addr[2];
    uint8_t set_data[2];//结构体仅可作为指针转换，不可声明为局部变量，否则此参数需修改
}net_list_oper_t;


#endif /* __API_TCP_H */

