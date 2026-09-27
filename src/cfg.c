#include "cfg.h"
#include "flash_io.h"
#include <stddef.h>
#include <string.h>

#define CFG_MAGIC           0x47464350u     /* 'PCFG' */

typedef struct
{
    uint32_t magic;
    uint32_t seq;
    cfg_t    cfg;
    uint32_t crc;           /* magic ~ cfg */
    uint32_t pad[25];
} cfg_page_t;

_Static_assert(sizeof(cfg_t) == 16, "cfg_t size");
_Static_assert(sizeof(cfg_page_t) == FLASH_PAGE_SIZE, "cfg_page_t size");

static cfg_t cur;
static uint32_t cur_seq;
static uint8_t cur_slot;            /* 最近一次有效数据所在页 */

/* 保存：先擦目标页，下一个空闲窗口再编程 */
static enum { SAVE_IDLE, SAVE_ERASE, SAVE_PROGRAM } save_state;
static cfg_page_t save_buf __attribute__((aligned(4)));

static const cfg_page_t *slot_page(uint8_t slot)
{
    return (const cfg_page_t *)(CFG_FLASH_ADDR + slot * FLASH_PAGE_SIZE);
}

static bool page_valid(const cfg_page_t *p)
{
    return p->magic == CFG_MAGIC && p->crc == crc32_calc(p, offsetof(cfg_page_t, crc)) && cfg_valid(&p->cfg);
}

void cfg_defaults(cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->mode = CFG_MODE_AVS;
    c->hide_fixed = 0;
    c->max_mv = 20000;
    c->max_ma = 3000;
    c->ovp_pct = 5;
    c->uvp_pct = 5;
    c->ocp_ma = 3500;
    c->ocp_ms = 50;
}

bool cfg_valid(const cfg_t *c)
{
    return c->mode <= CFG_MODE_FIXED && (c->hide_fixed & ~0x0Fu) == 0 && (c->flags & ~CFG_FLAGS_MASK) == 0 &&
           (c->max_mv == 15000 || c->max_mv == 20000) &&
           c->max_ma >= 500 && c->max_ma <= 3000 && c->max_ma % 50 == 0 &&
           c->ovp_pct >= 1 && c->ovp_pct <= 20 && c->uvp_pct >= 1 && c->uvp_pct <= 20 &&
           c->ocp_ma >= 500 && c->ocp_ma <= 5000 && c->ocp_ms >= 1 && c->ocp_ms <= 1000;
}

void cfg_init(void)
{
    cfg_defaults(&cur);
    cur_seq = 0;
    cur_slot = 1;
    for (uint8_t s = 0; s < CFG_FLASH_PAGES; s++)
    {
        const cfg_page_t *p = slot_page(s);
        if (page_valid(p) && p->seq >= cur_seq)
        {
            cur = p->cfg;
            cur_seq = p->seq;
            cur_slot = s;
        }
    }
}

const cfg_t *cfg(void)
{
    return &cur;
}

bool cfg_set(const cfg_t *c)
{
    if (!cfg_valid(c))
        return false;
    cur = *c;
    memset(cur.reserved, 0, sizeof(cur.reserved));
    save_state = SAVE_ERASE;    /* 保存中途再次修改：从擦除重新开始 */
    return true;
}

bool cfg_save_pending(void)
{
    return save_state != SAVE_IDLE;
}

bool cfg_flush_step(void)
{
    uint8_t slot = cur_slot ^ 1;    /* 写另一页，掉电时旧页仍有效 */
    switch (save_state)
    {
    case SAVE_ERASE:
        flash_page_erase(CFG_FLASH_ADDR + slot * FLASH_PAGE_SIZE);
        save_state = SAVE_PROGRAM;
        return true;

    case SAVE_PROGRAM:
        memset(&save_buf, 0, sizeof(save_buf));
        save_buf.magic = CFG_MAGIC;
        save_buf.seq = cur_seq + 1;
        save_buf.cfg = cur;
        save_buf.crc = crc32_calc(&save_buf, offsetof(cfg_page_t, crc));
        if (flash_page_program(CFG_FLASH_ADDR + slot * FLASH_PAGE_SIZE, (const uint32_t *)&save_buf))
        {
            cur_seq = save_buf.seq;
            cur_slot = slot;
        }
        save_state = SAVE_IDLE;
        return true;

    default:
        return false;
    }
}
