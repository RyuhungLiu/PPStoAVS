/*
 * PD 信息透传：前端读到的充电器信息、后端读到的设备信息缓存起来，由另一端代答。
 *   设备 → 充电器：电池（Battery_Capabilities / Battery_Status / Alert）、Sink_Capabilities_Extended、身份
 *   充电器 → 设备：Source_Capabilities_Extended、Source_Info、Status、Manufacturer_Info、身份
 * PDP 按后端实际能力改写；身份透传关闭时 VID/PID 换成本机的（自订身份打开时用自订的，与身份透传互斥）。
 * 应答时间只有 tReceiverResponse（15ms），来不及现场转问另一端，因此一律用缓存应答。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 查询项（位） */
#define PI_Q_IDENT      (1u << 0)   /* Discover Identity */
#define PI_Q_EXTCAPS    (1u << 1)   /* 设备：Sink_Capabilities_Extended；充电器：Source_Capabilities_Extended */
#define PI_Q_BCAP       (1u << 2)   /* 设备：Battery_Capabilities */
#define PI_Q_BSTAT      (1u << 3)   /* 设备：Battery_Status */
#define PI_Q_SIDO       (1u << 4)   /* 充电器：Source_Info */
#define PI_Q_STATUS     (1u << 5)   /* 充电器：Status */
#define PI_Q_MIDB       (1u << 6)   /* 充电器：Manufacturer_Info */

#define PI_VID          0x1209u     /* 本机 VID/PID（身份透传关闭时代替对方） */
#define PI_PID          0x0001u

#define PI_MAX_VDOS     6           /* Discover Identity ACK：ID Header、Cert Stat、Product、Product Type VDO ×3 */
#define PI_EXT_MAX      26          /* 单块扩展报文 */

typedef struct
{
    uint8_t  n;                     /* VDO 数，0 = 无 */
    uint32_t vdo[PI_MAX_VDOS];
} pi_ident_t;

bool pdinfo_on(void);
bool pdinfo_id_on(void);
bool pdinfo_fe_custom(void);                    /* 前端以自订 Sink 身份应答充电器 */
bool pdinfo_be_custom(void);                    /* 后端以自订 Source 身份应答设备 */

/* ---- 设备（后端） ---- */
void pdinfo_dev_reset(void);                    /* 设备拔出、Hard Reset */
void pdinfo_dev_attached(bool on);              /* 后端有 PD 设备插入（非 PD 设备、拔出为 false） */
void pdinfo_dev_start(bool pd3);                /* 后端建立合约：安排查询（PD2.0 设备没有这些报文） */
uint8_t pdinfo_dev_next_query(void);            /* 下一项待查询，0 = 无 */
void pdinfo_dev_query_done(uint8_t q);          /* 查询结束（成功、拒绝或超时） */
void pdinfo_dev_ident(const uint32_t *vdo, uint8_t n);
void pdinfo_dev_skedb(const uint8_t *d, uint8_t len);
void pdinfo_dev_bcap(const uint8_t *d, uint8_t len);
void pdinfo_dev_bsdo(uint32_t bsdo);
void pdinfo_dev_alert(uint32_t ado);            /* 设备 Alert：电池变化 → 重读电池状态后转发给充电器 */

/* 前端应答充电器用（返回 false = 没有数据） */
bool pdinfo_fe_skedb(uint8_t *out);             /* SKEDB_LEN 字节，已改写 */
bool pdinfo_fe_ident(pi_ident_t *out);
bool pdinfo_fe_ident_pending(void);             /* 设备已连接但身份未读到：可回 BUSY 让充电器稍后重试 */
void pdinfo_fe_bcap(uint8_t ref, uint8_t *out); /* 9 字节，没有数据时置“无效电池编号” */
uint32_t pdinfo_fe_bsdo(uint8_t ref);
uint32_t pdinfo_fe_take_alert(void);            /* 待转发给充电器的 ADO，0 = 无 */

/* ---- 充电器（前端） ---- */
void pdinfo_chg_reset(void);                    /* 前端 Hard Reset、充电器重新连接 */
void pdinfo_chg_start(bool pd3);                /* 前端首次建立合约：安排查询 */
uint8_t pdinfo_chg_next_query(void);
void pdinfo_chg_query_done(uint8_t q);
void pdinfo_chg_ident(const uint32_t *vdo, uint8_t n);
void pdinfo_chg_scedb(const uint8_t *d, uint8_t len);
void pdinfo_chg_sido(uint32_t sido);
void pdinfo_chg_status(const uint8_t *d, uint8_t len);
void pdinfo_chg_midb(const uint8_t *d, uint8_t len);

/* 后端应答设备用（返回长度或 false = 没有数据） */
uint8_t pdinfo_be_scedb(uint8_t *out);          /* 已改写 PDP、VID/PID，0 = 无 */
bool pdinfo_be_sido(uint32_t *out);
uint8_t pdinfo_be_status(uint8_t *out);         /* 同时安排前端重读，下次更新 */
uint8_t pdinfo_be_midb(uint8_t *out);
bool pdinfo_be_ident(pi_ident_t *out);

/* 上位机 STATUS part 3 */
uint8_t pdinfo_status(uint8_t *out);
