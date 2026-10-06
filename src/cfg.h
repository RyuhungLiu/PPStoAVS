/*
 * 可由上位机修改的透传设置，保存在 Flash（A/B 两页交替），修改后立即生效
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CFG_MODE_AVS       = 0,     /* a：Fixed + AVS（由 PPS 平移），默认 */
    CFG_MODE_PPS       = 1,     /* b：Fixed + PPS 原样透传 */
    CFG_MODE_AVS_PPS   = 2,     /* c：Fixed + AVS + PPS */
    CFG_MODE_FIXED     = 3,     /* d：仅 Fixed */
} cfg_mode_t;

/* hide_fixed 位 */
#define CFG_HIDE_9V         (1u << 0)
#define CFG_HIDE_12V        (1u << 1)
#define CFG_HIDE_15V        (1u << 2)
#define CFG_HIDE_20V        (1u << 3)

/* flags 位 */
#define CFG_FIX12           (1u << 0)   /* 充电器没有 12V 时，用 PPS（优先）或 AVS 提供 12V Fixed，PDO 位置不够就不提供 */
#define CFG_LOG_PPS         (1u << 1)   /* 记录设备的 PPS 请求（默认不记录：PPS 会频繁调压） */
#define CFG_FE_AVS_2ND      (1u << 2)   /* Lab：充电器查询 Sink_Capabilities_Extended 时声明支持 AVS，触发二次握手 */
#define CFG_BE_AVS_2ND      (1u << 3)   /* Lab：后端先给 PPS，设备声明支持 AVS 后才给 AVS（模拟二次握手） */
#define CFG_AVS_TO_PPS      (1u << 4)   /* Lab：充电器原生 AVS 转为后端 PPS（5~9V 拒绝，电压四舍五入到 100mV） */
#define CFG_FE_EMARKER      (1u << 5)   /* Lab：前端应答充电器的 SOP' 查询，扮演 5A 线材；电流上限可设到 5A */
#define CFG_EPR_AVS         (1u << 6)   /* Lab（需 CFG_FE_EMARKER）：前端 EPR AVS 的 15~20V 作为后端 SPR AVS 15~20V；9V/15V 走 Fixed */
#define CFG_FIX_CUSTOM      (1u << 7)   /* Lab：自订非标准 Fixed（fix_dv/fix_ma50），由 PPS（优先）或 AVS 提供；与 CFG_FIX12 互斥 */
#define CFG_FLAGS_MASK      (CFG_FIX12 | CFG_LOG_PPS | CFG_FE_AVS_2ND | CFG_BE_AVS_2ND | CFG_AVS_TO_PPS | CFG_FE_EMARKER | \
                             CFG_EPR_AVS | CFG_FIX_CUSTOM)
/* flags2 位（0 为默认值：旧版设置迁移后行为不变） */
#define CFG2_NO_INFO        (1u << 0)   /* 关闭 PD 信息透传（电量、充电器/设备信息） */
#define CFG2_ID_PT          (1u << 1)   /* 身份透传：Discover Identity、VID/PID、厂商信息用对方的 */
#define CFG2_FORCE_PPS      (1u << 2)   /* Lab：强制 PPS——后端所有档位（Fixed、AVS）尽量由充电器 PPS 提供，可做压降补偿、切换不经 Fixed */
#define CFG2_PPS_CUSTOM     (1u << 3)   /* Lab：自订 PPS（pps_min_dv ~ pps_max_dv、pps_ma50），由覆盖该范围的充电器 PPS（或 9V 起的 AVS）提供 */
#define CFG2_UFCS           (1u << 4)   /* 前端优先尝试 UFCS（上电检测 D+/D−，握手成功则前端走 UFCS，此时 D+/D− 不能用于上位机） */
#define CFG2_ID_FE          (1u << 5)   /* 自订前端 Sink 身份（fe_vid/fe_pid 给充电器看），与身份透传互斥 */
#define CFG2_ID_BE          (1u << 6)   /* 自订后端 Source 身份（be_vid/be_pid 给设备看），与身份透传互斥；默认开启 */
#define CFG2_AVS_PPS5       (1u << 7)   /* Lab AVS 转 PPS：从 5V 开始（默认 0 = 从 9V 开始，即 AVS 的实际范围） */
#define CFG2_MASK           (CFG2_AVS_PPS5 | CFG2_NO_INFO | CFG2_ID_PT | CFG2_FORCE_PPS | CFG2_PPS_CUSTOM | CFG2_UFCS | CFG2_ID_FE | CFG2_ID_BE)
/* flags3 位 */
#define CFG3_LOG            (1u << 0)   /* 事件记录（默认关闭，需在网页开启；关闭时不写 Flash） */
#define CFG3_OLD_NO5A       (1u << 1)   /* v0.11.x 的线材策略（固定 3A）；v0.12.0 起改为虚拟 E-Marker（cfg_ext_t），读到旧设置时迁移 */
#define CFG3_OLD_5A         (1u << 2)   /* v0.11.x 的线材策略（5A） */
#define CFG3_MASK           CFG3_LOG
#define CFG_PPS_DV_MIN      33          /* 自订 PPS 电压 3.3V ~ 21V（100mV 单位） */
#define CFG_PPS_DV_MAX      210
#define CFG_FIX_DV_MIN      51          /* 自订 Fixed 电压 5.1V ~ 20V（100mV 单位） */
#define CFG_FIX_DV_MAX      200
#define CFG_MAX_MA          3000
#define CFG_MAX_MA_5A       5000        /* CFG_FE_EMARKER 时 */
#define CFG_OCP_MAX_MA      5000
#define CFG_OCP_MAX_MA_5A   5500        /* 电流检测约 5.8A 饱和 */

/* 上位机协议直接收发此结构（小端，24 字节） */
typedef struct __attribute__((packed))
{
    uint8_t  mode;          /* cfg_mode_t */
    uint8_t  hide_fixed;    /* CFG_HIDE_*，5V 不可隐藏 */
    uint16_t max_mv;        /* 15000 / 20000 */
    uint16_t max_ma;        /* 500 ~ 3000，50mA 步进 */
    uint8_t  ovp_pct;       /* 1 ~ 20 */
    uint8_t  uvp_pct;       /* 1 ~ 20 */
    uint16_t ocp_ma;        /* 500 ~ 5000 */
    uint16_t ocp_ms;        /* 1 ~ 1000 */
    uint8_t  flags;         /* CFG_* 标志 */
    uint8_t  fix_dv;        /* CFG_FIX_CUSTOM：电压，100mV 单位（51 ~ 200） */
    uint8_t  fix_ma50;      /* CFG_FIX_CUSTOM：电流，50mA 单位（10 ~ 100） */
    uint8_t  flags2;        /* CFG2_*（旧版此字节恒为 0，位定义按 0 = 默认） */
    uint8_t  comp_mode;     /* cfg_comp_t：压降补偿 */
    uint8_t  reserved2;
    uint16_t comp_val;      /* V 补偿：mV（0 ~ 1000）；R 补偿：mΩ（0 ~ 500） */
    uint8_t  pps_min_dv;    /* CFG2_PPS_CUSTOM：最低电压，100mV 单位（33 ~ 209） */
    uint8_t  pps_max_dv;    /* CFG2_PPS_CUSTOM：最高电压，100mV 单位（> 最低，≤ 210） */
    uint8_t  pps_ma50;      /* CFG2_PPS_CUSTOM：电流，50mA 单位（10 ~ 100） */
    uint8_t  flags3;        /* CFG3_*：0 = 默认（日志关闭） */
    uint16_t fe_vid;        /* CFG2_ID_FE：前端 Sink 的 VID / PID（VID ≠ 0） */
    uint16_t fe_pid;
    uint16_t be_vid;        /* CFG2_ID_BE：后端 Source 的 VID / PID（VID ≠ 0） */
    uint16_t be_pid;
} cfg_t;

typedef enum
{
    CFG_COMP_OFF = 0,
    CFG_COMP_V   = 1,       /* 固定电压补偿 */
    CFG_COMP_R   = 2,       /* 随电流补偿：I × R */
} cfg_comp_t;

#define CFG_COMP_V_MAX      1000
#define CFG_COMP_R_MAX      500

/*
 * 扩展设置（v0.12.0）：与 cfg_t 一起存在同一页 Flash，由单独的上位机指令读写（协议 v11）
 *  - 虚拟 E-Marker（CFGX_CABLE）：后端不读线材，按这里的 VDO 应答设备的 SOP' Discover Identity；
 *    Cable VDO1 的电流位为 5A 时后端上限按 5A，为 3A 时不放宽
 *  - 自订 PDO 档位（CFGX_PDO，需 CFG2_FORCE_PPS）：最多 7 个原始 PDO（Fixed、PPS、至多一个 AVS）
 */
#define CFGX_CABLE          (1u << 0)
#define CFGX_PDO            (1u << 1)
#define CFGX_BE_IDLE        (1u << 2)   /* 后端虚拟 E-Marker 常驻应答：没有设备（无 Rd 下拉）、MOS 未开时也监听并应答 SOP' Discover Identity；需 CFGX_CABLE */
#define CFGX_PDO2           (1u << 3)   /* 第二组自订 PDO：设备在 Sink_Capabilities_Extended 声明支持 AVS 后改播发这一组；需 CFGX_PDO，与 CFG_BE_AVS_2ND 互斥 */
#define CFGX_MASK           (CFGX_CABLE | CFGX_PDO | CFGX_BE_IDLE | CFGX_PDO2)
#define CFGX_MAX_PDOS       7
typedef struct __attribute__((packed))
{
    uint8_t  flags;         /* CFGX_* */
    uint8_t  pdo_n;         /* 自订 PDO 个数（0 ~ 7） */
    uint16_t reserved;
    uint32_t cable[5];      /* ID Header、Cert Stat、Product、Cable VDO1、Cable VDO2（有源线材才发送） */
    uint32_t pdo[CFGX_MAX_PDOS];   /* 原始 PDO：Fixed、PPS APDO、SPR AVS APDO（电流位忽略）；顺序随意，固件按电压升序输出 */
    uint32_t fcable[5];     /* 前端虚拟 E-Marker（CFG_FE_EMARKER）应答充电器 SOP' 的 VDO，格式同 cable[]；上位机用 CMD_CFGF_* 单独读写 */
} cfg_ext_t;
#define CFGX_BASE_SIZE      52          /* CMD_CFGX_* 传输的前 52 字节（flags ~ pdo） */
#define CFGF_SIZE           20          /* CMD_CFGF_* 传输的 fcable */
/* 虚拟 E-Marker 的 VDO 个数（含 ID Header ~ Cable VDO1；有源线材，ID Header 产品类型 = 4，多一个 Cable VDO2） */
static inline uint8_t cfg_cable_n(uint32_t id_header)
{
    return ((id_header >> 27) & 7) == 4 ? 5 : 4;
}

/* 第二组自订 PDO（v0.13.0）：单独存一页（EXT2_FLASH_ADDR），命令 0x17 / 0x18；规则同 cfg_ext_t.pdo */
typedef struct
{
    uint8_t  n;             /* 0 ~ 7 */
    uint8_t  reserved[3];
    uint32_t pdo[CFGX_MAX_PDOS];
} cfg_pdo2_t;
#define CFGP_SIZE           32
const cfg_pdo2_t *cfg_pdo2(void);
bool cfg_pdo2_set(const cfg_pdo2_t *p);

const cfg_ext_t *cfg_ext(void);
void cfg_ext_defaults(cfg_ext_t *x);
bool cfg_ext_valid(const cfg_ext_t *x);
bool cfg_ext_set(const cfg_ext_t *x);
bool cfg_ext_cable_5a(const cfg_ext_t *x);
bool cfg_ext_fe_5a(const cfg_ext_t *x);         /* 前端 Cable VDO1 电流位 = 5A */      /* Cable VDO1 电流位 = 5A */

void cfg_init(void);                    /* 从 Flash 读取，无效则用默认值 */
const cfg_t *cfg(void);
void cfg_defaults(cfg_t *c);
bool cfg_valid(const cfg_t *c);
bool cfg_set(const cfg_t *c);           /* 校验并立即生效，Flash 保存排队 */
bool cfg_save_pending(void);
bool cfg_flush_step(void);              /* store 模块在 PD 空闲时调用；做了一次 Flash 操作返回 true */
