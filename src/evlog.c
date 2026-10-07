#include "evlog.h"
#include "cfg.h"
#include "flash_io.h"
#include "timebase.h"
#include <stddef.h>
#include <string.h>

#define LOG_MAGIC               0x4C50u     /* 'PL' */
#ifdef HV_PROBE
#define LOG_MAX_RECORDS         200         /* 探测固件：扫描命中较多 */
#else
#define LOG_MAX_RECORDS         64
#endif
#ifdef HV_PROBE
#define LOG_MAX_SESSION_PAGES   28          /* 探测固件每个事务都封页落盘 */
#else
#define LOG_MAX_SESSION_PAGES   16
#endif
#define LOG_ERASE_AHEAD         2           /* 上电时预擦除页数（此时 PD 尚未启动）；不够时空闲再擦 */
#define LOG_IDLE_SEAL_MS        2000        /* 没有新记录 2s 后封页，尽快落盘 */
#define REC_HDR                 6u

typedef struct
{
    uint16_t magic;
    uint16_t session;
    uint32_t seq;
    uint8_t  data[EVLOG_PAGE_DATA];
    uint32_t crc;
} log_page_t;

_Static_assert(sizeof(log_page_t) == FLASH_PAGE_SIZE, "log_page_t size");

static log_page_t cur __attribute__((aligned(4)));
static uint8_t cur_fill;
static int16_t last_rec_off = -1;           /* 当前页最后一条记录的偏移，用于合并 */
static log_page_t sealed[2] __attribute__((aligned(4)));
static uint8_t n_sealed;

static uint16_t session;
static uint32_t next_seq;
static uint8_t next_idx;                    /* 下一页写入的环形索引 */
static uint8_t erased_ahead;                /* 从 next_idx 起已擦除的页数 */
static uint8_t session_pages;
static uint8_t session_records;
static uint16_t dropped;
static uint32_t last_add_ms;
static int16_t clear_idx = -1;

static bool log_on(void)
{
#ifdef HV_PROBE
    return true;        /* 探测固件：不论设置都记录 */
#endif
    return (cfg()->flags3 & CFG3_LOG) != 0;
}

static const log_page_t *flash_page(uint8_t idx)
{
    return (const log_page_t *)(LOG_FLASH_ADDR + (uint32_t)idx * FLASH_PAGE_SIZE);
}

static uint32_t page_addr(uint8_t idx)
{
    return LOG_FLASH_ADDR + (uint32_t)idx * FLASH_PAGE_SIZE;
}

static bool page_crc_ok(const log_page_t *p)
{
    return p->magic == LOG_MAGIC && p->crc == crc32_calc(p, offsetof(log_page_t, crc));
}

static void cur_reset(void)
{
    memset(&cur, 0, sizeof(cur));
    cur.magic = LOG_MAGIC;
    cur.session = session;
    cur_fill = 0;
    last_rec_off = -1;
}

void evlog_init(void)
{
    int16_t max_idx = -1;
    uint32_t max_seq = 0;
    uint16_t max_session = 0;
    for (uint16_t i = 0; i < LOG_FLASH_PAGES; i++)
    {
        const log_page_t *p = flash_page(i);
        if (page_crc_ok(p) && (max_idx < 0 || p->seq > max_seq))
        {
            max_idx = i;
            max_seq = p->seq;
            max_session = p->session;
        }
    }
    if (max_idx < 0)
    {
        next_idx = 0;
        next_seq = 1;
        session = 1;
    }
    else
    {
        next_idx = (max_idx + 1) % LOG_FLASH_PAGES;
        next_seq = max_seq + 1;
        session = max_session + 1;
    }
    if (log_on())       /* 记录关闭时不预擦除（不动旧记录、不消耗 Flash 寿命） */
    {
        for (uint8_t k = 0; k < LOG_ERASE_AHEAD; k++)
        {
            flash_page_erase(page_addr((next_idx + k) % LOG_FLASH_PAGES));
        }
        erased_ahead = LOG_ERASE_AHEAD;
    }
    cur_reset();
}

uint16_t evlog_session(void)
{
    return session;
}

uint16_t evlog_dropped(void)
{
    return dropped;
}

static bool seal(void)
{
    if (cur_fill == 0)
        return true;
    if (n_sealed >= 2)
        return false;
    cur.seq = next_seq++;
    cur.crc = crc32_calc(&cur, offsetof(log_page_t, crc));
    sealed[n_sealed++] = cur;
    session_pages++;
    cur_reset();
    return true;
}

void evlog_add(uint8_t type, const void *payload, uint8_t len)
{
    if (clear_idx >= 0 || !log_on())
        return;
    if (session_records >= LOG_MAX_RECORDS || REC_HDR + len > EVLOG_PAGE_DATA)
    {
        dropped++;
        return;
    }
    if (cur_fill + REC_HDR + len > EVLOG_PAGE_DATA && !seal())
    {
        dropped++;
        return;
    }
    if (session_pages >= LOG_MAX_SESSION_PAGES)
    {
        dropped++;
        return;
    }
    uint8_t *r = &cur.data[cur_fill];
    uint32_t t = millis();
    r[0] = type;
    r[1] = len;
    memcpy(&r[2], &t, 4);
    memcpy(&r[REC_HDR], payload, len);
    last_rec_off = cur_fill;
    cur_fill += REC_HDR + len;
    session_records++;
    last_add_ms = t;
}

void evlog_request(const ev_request_t *req, bool adjustable)
{
    /* PPS/AVS 同一档位的连续调压：覆盖上一条，只记录最新值和次数 */
    if (adjustable && req->result == REQ_OK && last_rec_off >= 0)
    {
        uint8_t *rec = &cur.data[last_rec_off];
        ev_request_t prev;
        memcpy(&prev, &rec[REC_HDR], sizeof(prev));
        if (rec[0] == EV_REQUEST && rec[1] == sizeof(ev_request_t) && prev.result == REQ_OK &&
            (prev.back_rdo >> 28) == (req->back_rdo >> 28))
        {
            ev_request_t m = *req;
            uint32_t t = millis();
            m.repeat = prev.repeat < 255 ? prev.repeat + 1 : 255;
            memcpy(&rec[2], &t, 4);
            memcpy(&rec[REC_HDR], &m, sizeof(m));
            last_add_ms = t;
            return;
        }
    }
    evlog_add(EV_REQUEST, req, sizeof(*req));
}

bool evlog_flush_step(void)
{
    if (clear_idx >= 0)
    {
        flash_page_erase(page_addr(clear_idx));
        if (++clear_idx >= (int16_t)LOG_FLASH_PAGES)
        {
            clear_idx = -1;
            next_idx = 0;
            erased_ahead = LOG_FLASH_PAGES;
        }
        return true;
    }

    if (n_sealed == 0 && cur_fill > 0 && millis() - last_add_ms >= LOG_IDLE_SEAL_MS)
        seal();

    if (n_sealed == 0 && (erased_ahead > 0 || !log_on()))
        return false;

    if (erased_ahead == 0)
    {
        flash_page_erase(page_addr(next_idx));
        erased_ahead = 1;
        return true;
    }

    flash_page_program(page_addr(next_idx), (const uint32_t *)&sealed[0]);
    next_idx = (next_idx + 1) % LOG_FLASH_PAGES;
    erased_ahead--;
    sealed[0] = sealed[1];
    n_sealed--;
    return true;
}

void evlog_panic_flush(void)
{
    if (clear_idx >= 0)
        return;
    for (;;)
    {
        if (n_sealed == 0)
        {
            seal();
            if (n_sealed == 0)
                break;
        }
        if (erased_ahead == 0)
            break;                      /* 没有已擦除的页：擦除太慢，来不及 */
        flash_page_program(page_addr(next_idx), (const uint32_t *)&sealed[0]);
        next_idx = (next_idx + 1) % LOG_FLASH_PAGES;
        erased_ahead--;
        sealed[0] = sealed[1];
        n_sealed--;
    }
}

void evlog_clear(void)
{
    clear_idx = 0;
    n_sealed = 0;
    session_pages = 0;
    session_records = 0;
    dropped = 0;
    cur_reset();
}

bool evlog_clearing(void)
{
    return clear_idx >= 0;
}

uint8_t evlog_sessions(evlog_session_t *out, uint8_t max)
{
    if (clear_idx >= 0)
        return 0;
    uint8_t n = 0;
    uint32_t prev_seq = 0;
    for (uint16_t k = 0; k < LOG_FLASH_PAGES; k++)
    {
        uint8_t idx = (next_idx + LOG_FLASH_PAGES - 1 - k) % LOG_FLASH_PAGES;
        const log_page_t *p = flash_page(idx);
        if (p->magic != LOG_MAGIC || (k > 0 && p->seq != prev_seq - 1))
            break;
        prev_seq = p->seq;
        if (n == 0 || out[n - 1].session != p->session)
        {
            if (n == max)
                break;
            out[n].session = p->session;
            out[n].pages = 0;
            n++;
        }
        out[n - 1].first_idx = idx;
        out[n - 1].pages++;
    }
    return n;
}

uint8_t evlog_used_pages(void)
{
    evlog_session_t s[LOG_FLASH_PAGES / LOG_MAX_SESSION_PAGES + 1];
    uint8_t n = evlog_sessions(s, sizeof(s) / sizeof(s[0]));
    uint8_t used = 0;
    for (uint8_t i = 0; i < n; i++)
        used += s[i].pages;
    return used;
}

const uint8_t *evlog_page(uint8_t idx)
{
    if (idx < LOG_FLASH_PAGES)
        return (const uint8_t *)flash_page(idx);
    if (idx == EVLOG_RAM_CURRENT)
        return (const uint8_t *)&cur;
    if (idx == EVLOG_RAM_SEALED0 && n_sealed > 0)
        return (const uint8_t *)&sealed[0];
    if (idx == EVLOG_RAM_SEALED1 && n_sealed > 1)
        return (const uint8_t *)&sealed[1];
    return NULL;
}
