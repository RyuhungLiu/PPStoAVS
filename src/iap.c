#include "iap.h"
#include "board.h"

const iap_hdr_t *iap_hdr(void)
{
    return (const iap_hdr_t *)APP_HDR_ADDR;
}

bool iap_hdr_ok(const iap_hdr_t *h)
{
    return h->magic == IAP_HDR_MAGIC && h->size > 0 && h->size <= APP_MAX_SIZE && (h->size & 3) == 0 &&
           h->hdr_crc == crc32_calc(h, offsetof(iap_hdr_t, hdr_crc));
}

bool iap_app_valid(void)
{
    const iap_hdr_t *h = iap_hdr();
    return iap_hdr_ok(h) && crc32_calc((const void *)APP_ADDR, h->size) == h->crc;
}

void iap_hdr_page(uint32_t page[FLASH_PAGE_SIZE / 4], uint32_t size, uint32_t crc, uint16_t version)
{
    memset(page, 0xFF, FLASH_PAGE_SIZE);
    iap_hdr_t *h = (iap_hdr_t *)page;
    h->magic = IAP_HDR_MAGIC;
    h->size = size;
    h->crc = crc;
    h->version = version;
    h->reserved = 0;
    h->hdr_crc = crc32_calc(h, offsetof(iap_hdr_t, hdr_crc));
}

bool iap_flag_set(void)
{
    return *(const volatile uint32_t *)BL_FLAG_ADDR == IAP_FLAG_MAGIC;
}

#ifndef BOOTLOADER
void iap_enter_bootloader(void)
{
    uint32_t page[FLASH_PAGE_SIZE / 4];
    memset(page, 0xFF, sizeof(page));
    page[0] = IAP_FLAG_MAGIC;
    flash_page_erase(BL_FLAG_ADDR);
    flash_page_program(BL_FLAG_ADDR, page);
    NVIC_SystemReset();
    while (1)
        ;
}
#endif
