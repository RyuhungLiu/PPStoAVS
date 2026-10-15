/* SPDX-License-Identifier: MPL-2.0 */
/*
 * USB PD 报文定义与 PDO/RDO 编解码
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PD_MAX_DATA_OBJS        7
#define PD_MAX_MSG_LEN          (2 + 4 * PD_MAX_DATA_OBJS)  /* header + 7 个数据对象 */
#define PD_MAX_EPR_OBJS         11      /* EPR_Source_Capabilities：1~7 SPR（不足补 0），8~11 EPR */
#define PD_REV_30               0b10

/* 报文类型（Message Header 的 Message Type 字段，USB PD R3.2 表 6-5 / 6-6 / 6-48；只列本项目用到的） */
typedef enum
{
    PD_CTRL_GOODCRC         = 0x01,
    PD_CTRL_GOTOMIN         = 0x02,
    PD_CTRL_ACCEPT          = 0x03,
    PD_CTRL_REJECT          = 0x04,
    PD_CTRL_PING            = 0x05,
    PD_CTRL_PS_RDY          = 0x06,
    PD_CTRL_GET_SRC_CAP     = 0x07,
    PD_CTRL_GET_SNK_CAP     = 0x08,
    PD_CTRL_DR_SWAP         = 0x09,
    PD_CTRL_WAIT            = 0x0C,
    PD_CTRL_SOFT_RESET      = 0x0D,
    PD_CTRL_NOT_SUPPORTED   = 0x10,
    PD_CTRL_GET_SRC_CAP_EXT = 0x11,
    PD_CTRL_GET_STATUS      = 0x12,
    PD_CTRL_GET_SNK_CAP_EXT = 0x16,
    PD_CTRL_GET_SRC_INFO    = 0x17,
} pd_ctrl_msg_t;

typedef enum
{
    PD_DATA_SRC_CAP         = 0x01,
    PD_DATA_REQUEST         = 0x02,
    PD_DATA_BIST            = 0x03,
    PD_DATA_SNK_CAP         = 0x04,
    PD_DATA_BATTERY_STATUS  = 0x05,
    PD_DATA_ALERT           = 0x06,
    PD_DATA_EPR_REQUEST     = 0x09,
    PD_DATA_EPR_MODE        = 0x0A,
    PD_DATA_SRC_INFO        = 0x0B,
    PD_DATA_VDM             = 0x0F,
} pd_data_msg_t;

typedef enum
{
    PD_EXT_SRC_CAP_EXT        = 0x01,
    PD_EXT_STATUS             = 0x02,
    PD_EXT_GET_BATTERY_CAP    = 0x03,
    PD_EXT_GET_BATTERY_STATUS = 0x04,
    PD_EXT_BATTERY_CAP        = 0x05,
    PD_EXT_GET_MFR_INFO       = 0x06,
    PD_EXT_MFR_INFO           = 0x07,
    PD_EXT_SNK_CAP_EXT        = 0x0F,
    PD_EXT_CONTROL            = 0x10,
    PD_EXT_EPR_SRC_CAP        = 0x11,
} pd_ext_msg_t;

/* 结构化 VDM：[31:16] SVID，[15] 结构化，[14:13] 版本，[7:6] 命令类型，[4:0] 命令 */
#define VDM_DISC_IDENT_REQ      ((0xFF00u << 16) | (1u << 15) | (1u << 13) | 1u)    /* PD SID、VDM 2.0、Discover Identity */
#define VDM_STRUCTURED(v)       (((v) >> 15) & 1)
#define VDM_CMD_TYPE(v)         (((v) >> 6) & 3)    /* 0 REQ，1 ACK，2 NAK，3 BUSY */
#define VDM_IS_DISC_IDENT(v)    (((v) >> 16) == 0xFF00 && ((v) & 0x1F) == 1)
#define VDM_REPLY(v, type)      (((v) & ~0xC0u) | ((uint32_t)(type) << 6))
#define VDM_ACK                 1
#define VDM_NAK                 2
#define VDM_BUSY                3

/* 扩展报文头（Extended Message Header） */
#define EXT_CHUNKED             (1u << 15)
#define EXT_CHUNK_NUM(h)        (((h) >> 11) & 0xF)
#define EXT_REQUEST_CHUNK       (1u << 10)
#define EXT_DATA_SIZE(h)        ((h) & 0x1FF)
#define EXT_CHUNK_BYTES         26

/* Extended_Control（ECDB）类型 */
#define ECDB_EPR_GET_SOURCE_CAP 1
#define ECDB_EPR_GET_SINK_CAP   2
#define ECDB_EPR_KEEPALIVE      3
#define ECDB_EPR_KEEPALIVE_ACK  4

/* EPR_Mode（EPRMDO）：[31:24] Action，[23:16] Data */
#define EPR_MODE_ENTER          1
#define EPR_MODE_ENTER_ACK      2
#define EPR_MODE_ENTER_OK       3
#define EPR_MODE_ENTER_FAILED   4
#define EPR_MODE_EXIT           5

#define PDO_EPR_CAPABLE         (1u << 23)  /* Fixed 5V PDO：供电端支持 EPR */
#define RDO_EPR_CAPABLE         (1u << 22)  /* RDO：受电端支持 EPR Mode */

/* Sink_Capabilities_Extended（SKEDB，24 字节）中的 Sink Modes（byte 17） */
#define SKEDB_LEN               24
#define SKEDB_SINK_MODES        17
#define SINK_MODE_PPS           (1u << 0)
#define SINK_MODE_VBUS          (1u << 1)
#define SINK_MODE_AVS           (1u << 5)

/* PDO 类型：取 PDO 的 [31:28]。Fixed / Battery / Variable 只看 [31:30]（低两位清零），APDO（[31:30] = 11）再看 [29:28] */
typedef enum
{
    PDO_FIXED    = 0x0,
    PDO_BATTERY  = 0x4,
    PDO_VARIABLE = 0x8,
    PDO_PPS      = 0xC,
    PDO_EPR_AVS  = 0xD,
    PDO_SPR_AVS  = 0xE,
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
    uint16_t   min_mv;      /* PDO_FIXED: = max_mv */
    uint16_t   max_mv;
    uint16_t   max_ma;      /* SPR_AVS: 9~15V 档电流 */
    uint16_t   max_ma_20v;  /* 仅 SPR_AVS: 15~20V 档电流 */
    uint8_t    pdp;         /* 仅 EPR_AVS: PDP（W） */
} pdo_t;

/* Message Header：[15] Extended，[14:12] 数据对象数，[11:9] MessageID，[8] 电源角色，[7:6] 规范版本，[5] 数据角色，[4:0] 报文类型 */
static inline pd_header_t pd_parse_header(uint16_t h)
{
    return (pd_header_t){
        .msg_type   = h & 0x1F,
        .data_role  = (h >> 5) & 1,
        .revision   = (h >> 6) & 3,
        .power_role = (h >> 8) & 1,
        .msg_id     = (h >> 9) & 7,
        .num_objs   = (h >> 12) & 7,
        .extended   = h >> 15,
    };
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
    uint8_t top = v >> 28;
    p.type = (pdo_type_t)((top & 0xC) == 0xC ? top : (top & 0xC));    /* APDO 保留子类型，其余只留 [31:30] */
    p.raw = v;
    switch (p.type)
    {
    case PDO_FIXED:
        p.min_mv = p.max_mv = ((v >> 10) & 0x3FF) * 50;
        p.max_ma = (v & 0x3FF) * 10;
        break;
    case PDO_VARIABLE:
        p.max_mv = ((v >> 20) & 0x3FF) * 50;
        p.min_mv = ((v >> 10) & 0x3FF) * 50;
        p.max_ma = (v & 0x3FF) * 10;
        break;
    case PDO_PPS:
        p.max_mv = ((v >> 17) & 0xFF) * 100;
        p.min_mv = ((v >> 8) & 0xFF) * 100;
        p.max_ma = (v & 0x7F) * 50;
        break;
    case PDO_SPR_AVS:
        p.min_mv = 9000;
        p.max_mv = (v & 0x3FF) ? 20000 : 15000;
        p.max_ma = ((v >> 10) & 0x3FF) * 10;
        p.max_ma_20v = (v & 0x3FF) * 10;
        break;
    case PDO_EPR_AVS:
        p.min_mv = ((v >> 8) & 0xFF) * 100;
        p.max_mv = ((v >> 17) & 0x1FF) * 100;
        p.pdp = v & 0xFF;
        p.max_ma = 5000;
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

/* SPR AVS RDO：[20:9] 电压 25mV（100mV 步进），[6:0] 电流 50mA */
static inline uint32_t pd_build_avs_rdo(uint8_t pos, uint16_t mv, uint16_t ma)
{
    return ((uint32_t)pos << 28) | (1u << 24) | ((uint32_t)(mv / 100 * 4) << 9) | (ma / 50);
}

/* EPR AVS 在某电压下的最大电流：min(PDP ÷ V, 5A) */
static inline uint16_t pd_epr_avs_ma(const pdo_t *p, uint16_t mv)
{
    uint32_t ma = (uint32_t)p->pdp * 1000000u / mv;
    return ma > 5000 ? 5000 : (uint16_t)ma;
}

static inline uint8_t  pd_rdo_pos(uint32_t rdo)          { return (rdo >> 28) & 0xF; }
static inline uint16_t pd_rdo_fixed_op_ma(uint32_t rdo)  { return ((rdo >> 10) & 0x3FF) * 10; }
static inline uint32_t pd_rdo_avs_mv(uint32_t rdo)       { return ((rdo >> 9) & 0xFFF) * 25; }
static inline uint16_t pd_rdo_pps_mv(uint32_t rdo)       { return ((rdo >> 9) & 0xFFF) * 20; }
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
