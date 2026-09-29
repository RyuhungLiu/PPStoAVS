/*
 * UFCS（T/TAF 083-2022）报文编解码：只做字节 ↔ 结构体转换，不含时序，可在主机上单元测试
 *
 * 线上格式（不含训练字节 0xAA）：
 *   控制消息：头(2) 命令(1) CRC(1)
 *   数据消息：头(2) 命令(1) 长度(1) 数据(N) CRC(1)
 *   自定义  ：头(2) 保留(2) 长度(1) 数据(N) CRC(1)
 * 头 16 位（先发高字节）：[15:13] 设备地址（接收方）| [12:9] 消息编号 | [8:3] 协议版本 | [2:0] 消息类型
 * 多字节数据字段均为高字节在前。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define UFCS_ADDR_SOURCE        1       /* 001b 供电设备（充电器） */
#define UFCS_ADDR_SINK          2       /* 010b 充电设备（本机） */
#define UFCS_ADDR_CABLE         3

#define UFCS_TYPE_CTRL          0
#define UFCS_TYPE_DATA          1
#define UFCS_TYPE_VENDOR        2

#define UFCS_VERSION            1       /* 000001b */

#define UFCS_MAX_PKT            64      /* 头 2 + 命令 1 + 长度 1 + 数据 ≤ 59 + CRC 1 */
#define UFCS_MAX_DATA           59
#define UFCS_MAX_MODES          7

typedef enum
{
    UFCS_C_PING             = 0x00,
    UFCS_C_ACK              = 0x01,
    UFCS_C_NCK              = 0x02,
    UFCS_C_ACCEPT           = 0x03,
    UFCS_C_SOFT_RESET       = 0x04,
    UFCS_C_POWER_READY      = 0x05,
    UFCS_C_GET_OUTPUT_CAPS  = 0x06,
    UFCS_C_GET_SOURCE_INFO  = 0x07,
    UFCS_C_GET_SINK_INFO    = 0x08,
    UFCS_C_GET_CABLE_INFO   = 0x09,
    UFCS_C_GET_DEVICE_INFO  = 0x0A,
    UFCS_C_GET_ERROR_INFO   = 0x0B,
    UFCS_C_DETECT_CABLE     = 0x0C,
    UFCS_C_START_CABLE      = 0x0D,
    UFCS_C_END_CABLE        = 0x0E,
    UFCS_C_EXIT_UFCS        = 0x0F,
} ufcs_ctrl_t;

typedef enum
{
    UFCS_D_OUTPUT_CAPS      = 0x01,
    UFCS_D_REQUEST          = 0x02,
    UFCS_D_SOURCE_INFO      = 0x03,
    UFCS_D_SINK_INFO        = 0x04,
    UFCS_D_CABLE_INFO       = 0x05,
    UFCS_D_DEVICE_INFO      = 0x06,
    UFCS_D_ERROR_INFO       = 0x07,
    UFCS_D_CONFIG_WATCHDOG  = 0x08,
    UFCS_D_REFUSE           = 0x09,
    UFCS_D_VERIFY_REQUEST   = 0x0A,
    UFCS_D_VERIFY_RESPONSE  = 0x0B,
    UFCS_D_POWER_CHANGE     = 0x0C,
} ufcs_data_t;

typedef struct
{
    uint8_t addr;                   /* 接收方地址 */
    uint8_t num;                    /* 消息编号 0..15 */
    uint8_t ver;
    uint8_t type;                   /* UFCS_TYPE_* */
    uint8_t cmd;                    /* 控制/数据命令；自定义消息为 0 */
    uint8_t len;                    /* data[] 有效字节数（控制消息为 0） */
    uint8_t data[UFCS_MAX_DATA];
} ufcs_msg_t;

/* 输出模式（Output_Capabilities 里每个 8 字节） */
typedef struct
{
    uint8_t  num;                   /* 1..7 */
    uint8_t  ma_step;               /* 电流步进 mA：10/20/30/40/50 */
    uint8_t  mv_step;               /* 电压步进 mV：10 或 20 */
    uint16_t vmax_10mv;
    uint16_t vmin_10mv;
    uint16_t imax_10ma;
    uint8_t  imin_10ma;
} ufcs_mode_t;

uint8_t ufcs_crc8(const uint8_t *p, uint8_t n);

/* 编码：返回字节数（不含训练字节）；参数不合法返回 0 */
uint8_t ufcs_encode(const ufcs_msg_t *m, uint8_t *out);

/* 已收到的字节总数（由前 4~6 字节决定）：0 = 还不能确定，0xFF = 长度非法 */
uint8_t ufcs_frame_len(const uint8_t *p, uint8_t have);

/* 解码：长度、CRC、字段合法才返回 true。crc_ok 单独返回，供 NCK 判断 */
bool ufcs_decode(const uint8_t *p, uint8_t n, ufcs_msg_t *m, bool *crc_ok);

/* 常用消息构造 */
void ufcs_make_ctrl(ufcs_msg_t *m, uint8_t addr, uint8_t num, uint8_t cmd);
void ufcs_make_request(ufcs_msg_t *m, uint8_t num, uint8_t mode, uint16_t mv, uint16_t ma);
void ufcs_make_watchdog(ufcs_msg_t *m, uint8_t num, uint16_t ms);
/* Refuse：拒绝的消息（num/type/cmd）与原因 0x01 无法识别 0x02 不支持 0x03 忙 0x04 超范围 0x05 其它 */
void ufcs_make_refuse(ufcs_msg_t *m, uint8_t addr, uint8_t num, const ufcs_msg_t *refused, uint8_t reason);
/* Sink_Information：电池/接口温度 0 = 无数据；电压 10mV、电流 10mA */
void ufcs_make_sink_info(ufcs_msg_t *m, uint8_t addr, uint8_t num, uint16_t mv, uint16_t ma);

/* Output_Capabilities：解出最多 UFCS_MAX_MODES 个模式，返回个数；数据非法返回 0 */
uint8_t ufcs_parse_caps(const ufcs_msg_t *m, ufcs_mode_t *modes);

/* Source_Information：解出温度（0 = 无数据，否则 ℃）、输出电压 mV、电流 mA */
bool ufcs_parse_source_info(const ufcs_msg_t *m, int16_t *temp_c, int16_t *port_temp_c, uint16_t *mv, uint16_t *ma);

/* Refuse：解出被拒消息的编号/类型/命令与原因 */
bool ufcs_parse_refuse(const ufcs_msg_t *m, uint8_t *num, uint8_t *type, uint8_t *cmd, uint8_t *reason);

/* 把 UFCS 输出模式翻译成 PD Source_Capabilities 的 PDO（让协议桥把它当作一台 PPS 充电器）：
 *   Fixed：5/9/15/20V，被某个模式完整覆盖的才有（电流取覆盖它的模式里最大的，≤5A）；没有覆盖 5V 的模式 → 返回 0
 *   PPS  ：每个模式一个 APDO（电压范围取 3.3~21V 内，跨度 ≥1V；电流 ≤6.35A），按最高电压升序
 * raw[] 为 PDO 原始值（最多 7 个），src[] 为每个 PDO 对应的 UFCS 模式编号；返回个数 */
uint8_t ufcs_synth_caps(const ufcs_mode_t *modes, uint8_t n, uint32_t raw[7], uint8_t src[7]);

/* 把协议桥的目标电压/电流按该模式的步进取整并夹在范围内（电压向下取整、电流向下取整但不低于最小值）；
 * 电流下限超过上限等不可能满足时返回 false */
bool ufcs_map_request(const ufcs_mode_t *m, uint16_t mv, uint16_t ma, uint16_t *out_mv, uint16_t *out_ma);

/* 模式内电压/电流是否可请求：范围内且落在步进上（电压相对最小电压、电流相对最小电流） */
bool ufcs_mode_accepts(const ufcs_mode_t *md, uint16_t mv, uint16_t ma);
