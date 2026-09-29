/*
 * 透传记录：每次上电为一段（session），事件按时间顺序写入 Flash 环形区（128 页 × 128 字节）
 *
 * 页格式：magic u16 | session u16 | seq u32 | data[116] | crc32（前 124 字节）
 * 记录格式：type u8 | len u8 | t_ms u32（上电后毫秒）| payload[len]；type 0 表示页内结束
 *
 * 先写入 RAM 页缓冲，页满或空闲 2s 后封页，由 store 模块在 PD 空闲时写入 Flash。
 * 每段最多 64 条记录 / 16 页；超出后丢弃并计数。
 */
#pragma once

#include "cfg.h"
#include <stdbool.h>
#include <stdint.h>

#define EVLOG_PAGE_DATA         116u
#define EVLOG_RAM_CURRENT       0xFF    /* evlog_page() 索引：当前正在填写的 RAM 页 */
#define EVLOG_RAM_SEALED0       0xFE    /* 已封页、尚未写入 Flash */
#define EVLOG_RAM_SEALED1       0xFD

typedef enum
{
    EV_BOOT         = 1,    /* ev_boot_t */
    EV_FE_CAPS      = 2,    /* uint32_t pdo[n]：充电器 Source_Capabilities */
    EV_FE_LEGACY    = 3,    /* uint16_t ma：充电器不支持 PD */
    EV_FE_CONTRACT  = 4,    /* ev_fe_contract_t：前端自行发起的请求（能力响应、DPS 跟随、回 5V） */
    EV_BE_ATTACH    = 5,    /* uint8_t cc */
    EV_BE_DETACH    = 6,
    EV_BE_CAPS_V1   = 7,    /* uint32_t pdo[n]：v0.2.0 开发版格式，仅供上位机解码旧记录 */
    EV_REQUEST      = 8,    /* ev_request_t：设备请求及结果 */
    EV_PROTECT      = 9,    /* ev_protect_t */
    EV_RESET        = 10,   /* ev_reset_t */
    EV_CFG          = 11,   /* cfg_t：设置已修改 */
    EV_BE_CAPS      = 12,   /* uint32_t pdo[n] + uint8_t src[n]：后端能力（生成或变化时，不论设备是否插入）及对应充电器位置 */
    EV_AVS_2ND      = 13,   /* ev_avs_2nd_t：AVS 二次握手（Lab） */
    EV_NOTE         = 14,   /* ev_note_t：协议异常但已自动恢复 */
    EV_IDENT        = 15,   /* ev_ident_t：读到充电器 / 设备的身份（Discover Identity） */
    EV_UFCS_PKT     = 16,   /* ev_ufcs_pkt_t：UFCS 原始报文（探测固件） */
    EV_UFCS_STEP    = 17,   /* ev_ufcs_step_t：UFCS 探测流程节点 */
} ev_type_t;

typedef struct __attribute__((packed))
{
    uint16_t fw_version;
    uint8_t  reset_flags;   /* RCC_RSTSCKR[31:24] */
    cfg_t    cfg;
} ev_boot_t;

typedef struct __attribute__((packed))
{
    uint32_t rdo;
    uint16_t mv;
    uint8_t  ok;
} ev_fe_contract_t;

typedef enum
{
    REQ_OK          = 0,
    REQ_REJECT      = 1,    /* 超出后端能力 */
    REQ_FRONT_FAIL  = 2,    /* 前端请求失败 */
    REQ_VERIFY_FAIL = 3,    /* 电压未在窗口内稳定 */
} req_result_t;

typedef struct __attribute__((packed))
{
    uint32_t back_rdo;      /* 设备发来的 RDO */
    uint32_t front_rdo;     /* 对应的前端 RDO（完成时的前端合约） */
    uint16_t mv;            /* 目标电压 */
    uint16_t vbus_mv;       /* 完成时实测前端 VBUS */
    uint8_t  result;        /* req_result_t */
    uint8_t  repeat;        /* 同一 PPS/AVS 档位的连续微调合并次数 */
    uint16_t ms;            /* 从回复设备 Accept 到结束的时间 */
    uint8_t  waits;         /* 充电器回复 Wait 的次数 */
    uint8_t  txwait;        /* 前端等 SinkTxOK 的 ms（最大一次）；255 = 超时仍发送 */
} ev_request_t;

typedef enum
{
    PROT_OVP = 1,
    PROT_UVP = 2,
    PROT_OCP = 3,
} prot_kind_t;

typedef struct __attribute__((packed))
{
    uint8_t  kind;          /* prot_kind_t */
    uint16_t vbus_mv;
    uint16_t ibus_ma;
    uint16_t target_mv;
} ev_protect_t;

typedef enum
{
    RST_HARD_SENT   = 0,
    RST_HARD_RCVD   = 1,
    RST_SOFT_RCVD   = 2,
    RST_SOFT_SENT   = 3,
} rst_kind_t;

typedef struct __attribute__((packed))
{
    uint8_t  side;          /* 0 = 前端，1 = 后端 */
    uint8_t  kind;          /* rst_kind_t */
    uint8_t  state;         /* 当时的状态机状态（fe/be_state_code） */
    uint16_t rx_err;        /* 该端口累计 RX_RESET 次数 */
} ev_reset_t;

typedef enum
{
    NOTE_FE_ACCEPT_MISSED = 1,  /* 前端漏收 Accept，已由 PS_RDY 补回 */
    NOTE_FE_EMARKER       = 2,  /* 已以虚拟 E-Marker（5A 线材）应答充电器的 SOP' Discover Identity */
    NOTE_FE_EPR_ENTERED   = 3,  /* 前端进入 EPR 模式 */
    NOTE_FE_EPR_FAILED    = 4,  /* 充电器拒绝进入 EPR，data = 原因码 */
    NOTE_FE_EPR_EXIT      = 5,  /* 前端退出 EPR 模式 */
    NOTE_COMP             = 7,  /* 压降补偿：周期更新已生效，data = 补偿量（10mV 单位，与上次记录相差 ≥100mV 才记） */
    NOTE_FE_IDENT         = 6,  /* 身份透传：回复充电器的 Discover Identity，data = 1 ACK、2 NAK、3 BUSY（相同回复只记一次） */
} note_code_t;

typedef struct __attribute__((packed))
{
    uint8_t side;
    uint8_t code;           /* note_code_t */
    uint8_t data;
} ev_note_t;

typedef enum
{
    AVS2_FE_SKEDB_SENT  = 0,    /* 前端：回复充电器 Sink_Capabilities_Extended（声明支持 AVS） */
    AVS2_BE_SKEDB_RCVD  = 1,    /* 后端：收到设备 Sink_Capabilities_Extended，modes 为 Sink Modes */
    AVS2_BE_NO_SKEDB    = 2,    /* 后端：设备不支持或未回复，维持 PPS */
} avs_2nd_kind_t;

typedef struct __attribute__((packed))
{
    uint8_t side;           /* 0 = 前端，1 = 后端 */
    uint8_t kind;           /* avs_2nd_kind_t */
    uint8_t modes;          /* Sink Modes */
} ev_avs_2nd_t;

typedef struct __attribute__((packed))
{
    uint8_t  side;          /* 0 = 充电器（前端读到），1 = 设备（后端读到） */
    uint16_t vid;
    uint16_t pid;
    uint16_t bcd;           /* bcdDevice */
} ev_ident_t;

typedef struct __attribute__((packed))
{
    uint16_t session;
    uint8_t  first_idx;     /* 该段最早一页的环形索引 */
    uint8_t  pages;
} evlog_session_t;

void evlog_init(void);                              /* 上电：扫描环形区、预擦除（PD 初始化前调用） */
uint16_t evlog_session(void);
uint16_t evlog_dropped(void);
uint8_t evlog_used_pages(void);

void evlog_add(uint8_t type, const void *payload, uint8_t len);
void evlog_request(const ev_request_t *r, bool adjustable);    /* adjustable：PPS/AVS，可合并 */

void evlog_panic_flush(void);                       /* 掉电时的紧急写入：封页并同步写入已擦除的页，不做任何等待 */
bool evlog_flush_step(void);                        /* store 模块在 PD 空闲时调用 */
void evlog_clear(void);                             /* 清除全部记录（分多次空闲窗口擦除） */
bool evlog_clearing(void);

uint8_t evlog_sessions(evlog_session_t *out, uint8_t max);     /* 最新在前 */
const uint8_t *evlog_page(uint8_t idx);             /* 128 字节整页；无效索引返回 NULL */

/* UFCS 探测：原始报文（dir 0 = 收，1 = 发；raw 为线上字节，不含训练字节；后面跟 raw[len]） */
typedef struct __attribute__((packed))
{
    uint8_t  dir;
    uint8_t  baud;          /* 0 = 115200，1 = 57600，2 = 38400（收：训练字节判定；发：标称） */
    uint16_t bit_ticks;     /* 收：实测位宽，12MHz 计数（115200 ≈ 104）；发：0 */
    uint8_t  len;
} ev_ufcs_pkt_t;

typedef enum
{
    UFS_DCP         = 1,    /* arg = 判别标志（bit0 D+ 跟随 D−，bit1 撤去驱动后 D+ 回低） */
    UFS_HS_OK       = 2,    /* arg = 第几次握手尝试成功 */
    UFS_HS_FAIL     = 3,    /* 三次握手均无 D+ 拉高：不是 UFCS 充电器 */
    UFS_PING_OK     = 4,    /* arg = baud | (发送地址 << 4)；v_mv = 从握手到 ACK 的 ms */
    UFS_PING_FAIL   = 5,    /* arg = baud | (发送地址 << 4) */
    UFS_CAPS        = 6,    /* arg = 模式数（原始报文见 EV_UFCS_PKT） */
    UFS_REQUEST     = 7,    /* arg = 模式号，v_mv / i_ma = 请求值 */
    UFS_ACCEPT      = 8,    /* v_mv = 从发出 Request 到收到 Accept 的 ms */
    UFS_READY       = 9,    /* v_mv = 请求值，vbus_mv = 实测，i_ma = 从 Accept 到 Power_Ready 的 ms */
    UFS_REFUSE      = 10,   /* arg = 原因，v_mv = 被拒命令 */
    UFS_TIMEOUT     = 11,   /* arg = 等待对象（见 ufs_wait_t） */
    UFS_SRC_INFO    = 12,   /* v_mv / i_ma = 充电器报告的输出，arg = 温度 ℃（无数据 = 0x80），vbus_mv = 实测 */
    UFS_HARD_RESET  = 13,   /* arg = 0 收到充电器硬复位，1 我方发出 */
    UFS_END         = 14,   /* arg = 0 探测完成保持中，1 失败（已发硬复位），2 充电器结束（硬复位或 Exit_UFCS），v_mv = 累计报文错误数 */
    UFS_SCAN_A      = 16,   /* D+/D− 电平扫描（档位×51.6mV）：arg = D+ 悬空，v_mv = D− 悬空 */
    UFS_SCAN_B      = 17,   /* arg = D− 驱动 0.6V 时的 D+，v_mv = D+ 驱动 0.6V 时的 D− */
    UFS_VBUS_LOST   = 18,   /* 前端 VBUS 掉到 3.5V 以下（充电器断电）：arg = 当时的流程状态，v_mv = 读数；随后立即把记录写入 Flash */
    UFS_RESET_LEN   = 19,   /* 充电器硬复位的低电平持续时间：v_mv = µs（规范应 ≥ 2000） */
    UFS_PHY_STATS   = 15,   /* arg = 训练不合格数，v_mv = 训练合格数，i_ma = 超时+成帧错误 */
} ufs_step_t;

typedef enum
{
    UFW_ACK         = 1,    /* 三次重发仍无 ACK */
    UFW_CAPS        = 2,    /* Get_Output_Capabilities 之后没有 Output_Capabilities */
    UFW_ACCEPT      = 3,
    UFW_READY       = 4,
    UFW_INFO        = 5,
} ufs_wait_t;

typedef struct __attribute__((packed))
{
    uint8_t  code;          /* ufs_step_t */
    uint8_t  arg;
    uint16_t v_mv;
    uint16_t i_ma;
    uint16_t vbus_mv;
} ev_ufcs_step_t;
