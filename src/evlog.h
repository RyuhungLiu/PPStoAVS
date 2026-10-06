/*
 * 透传记录：每次上电为一段（session），事件按时间顺序写入 Flash 环形区（32 页 × 128 字节，Flash 布局 2）
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
    EV_HV_STEP      = 20,   /* ev_hv_step_t：AFC/FCP 探测固件（-DHV_PROBE）流程节点 */
    EV_HV_EDGES     = 21,   /* AFC/FCP 探测：tag u8（FCP：事务 | CRC 方案 << 4；0x0E/0x0F = AFC 9V/5V 第一轮的从机回应）, (起始序号 << 1 | 首段电平) u8, n u8, 电平持续 µs u16[n]（电平逐段交替） */
    EV_HV_BYTES     = 22,   /* AFC/FCP/SCP 探测：tag u8, 校验错位图 u8, n u8, 从机字节[n]（固件按波形解码） */
    EV_QC_STEP      = 19,   /* ev_qc_step_t：QC2.0/3.0 探测固件（-DQC_PROBE）流程节点，正式前端只记 QCS_CONNECT */
    EV_CABLE        = 18,   /* ev_cable_t：后端线材 E-Marker 读取结果（每次连接一条） */
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
    uint16_t defer;         /* 该端口发送前因 RX 未处理而推迟的累计次数 */
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
    uint8_t  status;        /* be_cable_status_t：2 无应答、3 NAK、4 已读到 */
    uint8_t  flags;         /* bit0 另一根 CC 脚出现 Ra，bit1 硬件可供 VCONN */
    uint8_t  n;             /* VDO 数 */
    uint32_t vdo[5];        /* ID Header、Cert Stat、Product、Cable VDO1、Cable VDO2；记录时只写 n 个 */
} ev_cable_t;

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
const uint8_t *evlog_page(uint8_t idx);             /* 128 字节整页；无效索引返回 NULL（索引 0~31） */

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

/* QC 探测（-DQC_PROBE） */
typedef enum
{
    QCS_DCP     = 1,    /* arg：1 = D− 跟随 D+ 0.6V（DCP 短接）；x = D− 电平档位（×51.6mV） */
    QCS_BC_DONE = 2,    /* arg：1 = 充电器已放开 D−；x = 从 D+ 0.6V 起的 ms */
    QCS_MODE    = 3,    /* arg：qc_mode_t；x = 等待 ms */
    QCS_PULSE   = 4,    /* arg：bit7 = 升，低 7 位 = 脉冲数；x = 脉宽 µs */
    QCS_DONE    = 5,
    QCS_AFC     = 8,    /* 正式前端（Lab AFC）：arg = 充电器回的 V/I 表字节数（0 = 不是 AFC），x = 表的第 1、2 字节，t_ms 字段 = 第 3、4 字节 */
    QCS_CONNECT = 7,    /* 正式前端（Lab QC）：arg = QC_* 探测结果，0 = 不是 QC 充电器；x = QC3 测试 3 个脉冲后 VBUS 升高 mV，t_ms 字段改放测试前基准 VBUS（mV） */
    QCS_IDLE    = 6,    /* 不驱动时：arg = D+ 电平档位，x = D− 电平档位（×51.6mV，64 = 高于 3.25V） */
} qc_step_t;

typedef enum
{
    QCM_5V   = 0,       /* D+ 0.6 / D− 0 */
    QCM_9V   = 1,       /* 3.3 / 0.6 */
    QCM_12V  = 2,       /* 0.6 / 0.6 */
    QCM_20V  = 3,       /* 3.3 / 3.3 */
    QCM_CONT = 4,       /* QC3 连续模式：0.6 / 3.3 */
} qc_mode_t;

typedef struct __attribute__((packed))
{
    uint8_t  code;      /* qc_step_t */
    uint8_t  arg;
    uint16_t x;
    uint16_t vbus_mv;
    uint16_t t_ms;      /* 从探测开始的 ms */
} ev_qc_step_t;

/* AFC / FCP 探测（-DHV_PROBE） */
typedef enum
{
    HVS_HANDSHAKE = 1,  /* arg：1 = 充电器已放开 D−；x = ms */
    HVS_AFC       = 2,  /* arg：低 4 位 0 = 成功、1~4 = 第几个从机 Ping 失败，高 4 位 = 第几轮；x = 第一个从机 Ping 的 µs */
    HVS_VBUS      = 3,  /* arg：1 AFC 9V 后，2 AFC 回 5V 后，3 SCP 5.5V 输出使能后，5 SCP 复位后，9 VBUS 掉到 4V 以下（充电器断电）；x = VBUS mV */
    HVS_FCP       = 4,  /* arg：低 4 位 = 事务（hv_probe.c xfers[]：1 读 0x80 … 9 读 0x21，10~13 写 0xA0 / 0xCA），
                           高 4 位 = CRC 方案（0 不带，见 hv_probe.c crcs[]）；x：低 8 位 = 回应跳变数（2 = 只有从机 Ping），
                           bit8 = 首个从机 Ping 失败，bit9 = 结尾从机 Ping 失败 */
    HVS_DONE      = 5,
    HVS_FCP_CRC   = 6,  /* arg：有回应的 CRC 方案，0xFF = 都没有 */
} hv_step_t;

typedef struct __attribute__((packed))
{
    uint8_t  code;      /* hv_step_t */
    uint8_t  arg;
    uint16_t x;
    uint16_t vbus_mv;
    uint16_t t_ms;
} ev_hv_step_t;
