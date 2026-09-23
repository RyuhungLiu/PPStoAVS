/*
 * USB PD 报文定义与 PDO/RDO 编解码
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PD_MAX_DATA_OBJS        7
#define PD_MAX_MSG_LEN          (2 + 4 * PD_MAX_DATA_OBJS)  /* header + 7 个数据对象 */
#define PD_REV_30               0b10

typedef enum
{
    MSG_TYPE_GoodCRC                 = 0b00001,
    MSG_TYPE_GotoMin                 = 0b00010,
    MSG_TYPE_Accept                  = 0b00011,
    MSG_TYPE_Reject                  = 0b00100,
    MSG_TYPE_Ping                    = 0b00101,
    MSG_TYPE_PS_RDY                  = 0b00110,
    MSG_TYPE_Get_Source_Cap          = 0b00111,
    MSG_TYPE_Get_Sink_Cap            = 0b01000,
    MSG_TYPE_DR_Swap                 = 0b01001,
    MSG_TYPE_PR_Swap                 = 0b01010,
    MSG_TYPE_VCONN_Swap              = 0b01011,
    MSG_TYPE_Wait                    = 0b01100,
    MSG_TYPE_Soft_Reset              = 0b01101,
    MSG_TYPE_Data_Reset              = 0b01110,
    MSG_TYPE_Data_Reset_Complete     = 0b01111,
    MSG_TYPE_Not_Supported           = 0b10000,
    MSG_TYPE_Get_Source_Cap_Extended = 0b10001,
    MSG_TYPE_Get_Status              = 0b10010,
    MSG_TYPE_FR_Swap                 = 0b10011,
    MSG_TYPE_Get_PPS_Status          = 0b10100,
    MSG_TYPE_Get_Country_Codes       = 0b10101,
    MSG_TYPE_Get_Sink_Cap_Extended   = 0b10110,
    MSG_TYPE_Get_Source_Info         = 0b10111,
    MSG_TYPE_Get_Revision            = 0b11000,
} pd_ctrl_msg_t;

typedef enum
{
    MSG_TYPE_Source_Capabilities = 0b00001,
    MSG_TYPE_Request             = 0b00010,
    MSG_TYPE_BIST                = 0b00011,
    MSG_TYPE_Sink_Capabilities   = 0b00100,
    MSG_TYPE_Battery_Status      = 0b00101,
    MSG_TYPE_Alert               = 0b00110,
    MSG_TYPE_Get_Country_Info    = 0b00111,
    MSG_TYPE_Enter_USB           = 0b01000,
    MSG_TYPE_EPR_Request         = 0b01001,
    MSG_TYPE_EPR_Mode            = 0b01010,
    MSG_TYPE_Source_Info         = 0b01011,
    MSG_TYPE_Revision            = 0b01100,
    MSG_TYPE_Vendor_Defined      = 0b01111,
} pd_data_msg_t;

typedef enum
{
    FPDO        = 0b0000,
    BPDO        = 0b0100,
    VPDO        = 0b1000,
    PPS_PDO     = 0b1100,
    EPR_AVS_PDO = 0b1101,
    SPR_AVS_PDO = 0b1110,
} pdo_type_t;

typedef struct
{
    bool     extended;
    uint8_t  num_objs;
    uint8_t  msg_id;
    uint8_t  power_role;
    uint8_t  revision;
    uint8_t  data_role;
    uint8_t  msg_type;
} pd_header_t;

/* 解析后的 PDO（只保留本项目用到的类型字段） */
typedef struct
{
    pdo_type_t type;
    uint32_t   raw;
    uint16_t   min_mv;      /* FPDO: = max_mv */
    uint16_t   max_mv;
    uint16_t   max_ma;      /* SPR_AVS: 9~15V 档电流 */
    uint16_t   max_ma_20v;  /* 仅 SPR_AVS: 15~20V 档电流 */
} pdo_t;

static inline pd_header_t pd_parse_header(uint16_t h)
{
    pd_header_t r;
    r.extended   = (h >> 15) & 0x1;
    r.num_objs   = (h >> 12) & 0x7;
    r.msg_id     = (h >> 9) & 0x7;
    r.power_role = (h >> 8) & 0x1;
    r.revision   = (h >> 6) & 0x3;
    r.data_role  = (h >> 5) & 0x1;
    r.msg_type   = h & 0x1F;
    return r;
}

static inline uint16_t pd_build_header(uint8_t msg_type, uint8_t num_objs, uint8_t msg_id,
                                       uint8_t power_role, uint8_t data_role, uint8_t revision)
{
    return (uint16_t)(((num_objs & 0x7) << 12) | ((msg_id & 0x7) << 9) | ((power_role & 1) << 8) |
                      ((revision & 0x3) << 6) | ((data_role & 1) << 5) | (msg_type & 0x1F));
}

static inline pdo_t pd_parse_pdo(uint32_t v)
{
    pdo_t p = {0};
    uint8_t raw_type = (v >> 28) & 0b1100;
    p.type = (raw_type != 0b1100) ? (pdo_type_t)raw_type : (pdo_type_t)(v >> 28);
    p.raw = v;
    switch (p.type)
    {
    case FPDO:
        p.min_mv = p.max_mv = ((v >> 10) & 0x3FF) * 50;
        p.max_ma = (v & 0x3FF) * 10;
        break;
    case VPDO:
        p.max_mv = ((v >> 20) & 0x3FF) * 50;
        p.min_mv = ((v >> 10) & 0x3FF) * 50;
        p.max_ma = (v & 0x3FF) * 10;
        break;
    case PPS_PDO:
        p.max_mv = ((v >> 17) & 0xFF) * 100;
        p.min_mv = ((v >> 8) & 0xFF) * 100;
        p.max_ma = (v & 0x7F) * 50;
        break;
    case SPR_AVS_PDO:
        p.min_mv = 9000;
        p.max_mv = (v & 0x3FF) ? 20000 : 15000;
        p.max_ma = ((v >> 10) & 0x3FF) * 10;
        p.max_ma_20v = (v & 0x3FF) * 10;
        break;
    default:
        break;
    }
    return p;
}

/* Fixed PDO（除 PDO1 外标志位全为 0） */
static inline uint32_t pd_build_fixed_pdo(uint16_t mv, uint16_t ma, uint32_t flags)
{
    return flags | ((uint32_t)(mv / 50) << 10) | (ma / 10);
}

/* SPR AVS APDO：[31:28]=1110，[19:10] 9~15V 最大电流，[9:0] 15~20V 最大电流（10mA） */
static inline uint32_t pd_build_spr_avs_apdo(uint16_t ma_15v, uint16_t ma_20v)
{
    return (0b1110u << 28) | ((uint32_t)(ma_15v / 10) << 10) | (ma_20v / 10);
}

/* PPS APDO（用于 Sink_Capabilities） */
static inline uint32_t pd_build_pps_apdo(uint16_t min_mv, uint16_t max_mv, uint16_t ma)
{
    return (0b1100u << 28) | ((uint32_t)(max_mv / 100) << 17) | ((uint32_t)(min_mv / 100) << 8) | (ma / 50);
}

/* Fixed/Variable RDO */
static inline uint32_t pd_build_fixed_rdo(uint8_t pos, uint16_t op_ma, uint16_t max_ma)
{
    return ((uint32_t)pos << 28) | (1u << 24) /* No USB Suspend */ |
           ((uint32_t)(op_ma / 10) << 10) | (max_ma / 10);
}

/* PPS RDO：[20:9] 电压 20mV，[6:0] 电流 50mA */
static inline uint32_t pd_build_pps_rdo(uint8_t pos, uint16_t mv, uint16_t ma)
{
    return ((uint32_t)pos << 28) | (1u << 24) | ((uint32_t)(mv / 20) << 9) | (ma / 50);
}

static inline uint8_t  pd_rdo_pos(uint32_t rdo)          { return (rdo >> 28) & 0xF; }
static inline uint16_t pd_rdo_fixed_op_ma(uint32_t rdo)  { return ((rdo >> 10) & 0x3FF) * 10; }
static inline uint32_t pd_rdo_avs_mv(uint32_t rdo)       { return ((rdo >> 9) & 0xFFF) * 25; }
static inline uint16_t pd_rdo_avs_ma(uint32_t rdo)       { return (rdo & 0x7F) * 50; }

static inline uint32_t pd_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void pd_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = v;
    p[1] = v >> 8;
    p[2] = v >> 16;
    p[3] = v >> 24;
}
