#include "flash_io.h"
#include "board.h"

/* 参考手册 §19.3 FLASH_CTLR / FLASH_STATR */
#define CR_STRT         (1u << 6)
#define CR_LOCK         (1u << 7)
#define CR_FLOCK        (1u << 15)
#define CR_PAGE_PG      (1u << 16)
#define CR_PAGE_ER      (1u << 17)
#define CR_BUF_LOAD     (1u << 18)
#define CR_BUF_RST      (1u << 19)
#define SR_BSY          (1u << 0)
#define SR_WRPRTERR     (1u << 4)

#define FLASH_KEY1      0x45670123u
#define FLASH_KEY2      0xCDEF89ABu

static bool in_data_area(uint32_t addr)
{
    return (addr & (FLASH_PAGE_SIZE - 1)) == 0 && addr >= CFG_FLASH_ADDR &&
           addr < LOG_FLASH_ADDR + LOG_FLASH_PAGES * FLASH_PAGE_SIZE;
}

static void unlock(void)
{
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
    FLASH->MODEKEYR = FLASH_KEY1;
    FLASH->MODEKEYR = FLASH_KEY2;
}

static void lock(void)
{
    FLASH->CTLR |= CR_FLOCK | CR_LOCK;
}

static void wait_idle(void)
{
    while (FLASH->STATR & SR_BSY)
        ;
}

/* 与 WCH 库相同：每次操作后读一次相邻区域，刷新取指缓存 */
static void sync_cache(uint32_t addr)
{
    *(volatile uint32_t *)0x40022034 = *(volatile uint32_t *)((addr & ~3u) ^ 0x100u);
}

bool flash_page_erase(uint32_t addr)
{
    if (!in_data_area(addr))
        return false;
    unlock();
    FLASH->CTLR |= CR_PAGE_ER;
    FLASH->ADDR = addr;
    FLASH->CTLR |= CR_STRT;
    wait_idle();
    FLASH->CTLR &= ~CR_PAGE_ER;
    sync_cache(addr);
    bool ok = (FLASH->STATR & SR_WRPRTERR) == 0;
    lock();
    return ok;
}

bool flash_page_program(uint32_t addr, const uint32_t *data)
{
    if (!in_data_area(addr))
        return false;
    unlock();
    FLASH->CTLR |= CR_PAGE_PG;
    FLASH->CTLR |= CR_BUF_RST;
    wait_idle();
    for (uint32_t i = 0; i < FLASH_PAGE_SIZE / 4; i += 2)
    {
        *(volatile uint32_t *)(addr + i * 4) = data[i];
        *(volatile uint32_t *)(addr + i * 4 + 4) = data[i + 1];
        FLASH->CTLR |= CR_BUF_LOAD;
        wait_idle();
        sync_cache(addr + i * 4);
    }
    FLASH->ADDR = addr;
    FLASH->CTLR |= CR_STRT;
    wait_idle();
    FLASH->CTLR &= ~CR_PAGE_PG;
    sync_cache(addr);
    lock();
    return memcmp((const void *)addr, data, FLASH_PAGE_SIZE) == 0;
}

/* CRC-32（IEEE 802.3，反射，初值/异或 0xFFFFFFFF），半字节查表 */
uint32_t crc32_calc(const void *data, uint32_t len)
{
    static const uint32_t tab[16] = {
        0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC, 0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
        0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C, 0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C,
    };
    const uint8_t *p = data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--)
    {
        crc ^= *p++;
        crc = (crc >> 4) ^ tab[crc & 0x0F];
        crc = (crc >> 4) ^ tab[crc & 0x0F];
    }
    return ~crc;
}
