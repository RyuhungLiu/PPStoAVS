#include "store.h"
#include "be_source.h"
#include "bridge.h"
#include "cfg.h"
#include "evlog.h"
#include "fe_sink.h"
#include "pd_phy.h"
#include "timebase.h"
#include "ufcs.h"

#define STORE_QUIET_MS      300     /* 两端 PD 至少空闲这么久 */
#define STORE_OP_GAP_MS     50      /* 两次页操作之间的间隔 */

void store_process(void)
{
    static uint32_t last_op_ms;
    uint32_t now = millis();

    if (now - last_op_ms < STORE_OP_GAP_MS)
        return;
    if (now - pd_phy_be.last_act_ms < STORE_QUIET_MS || now - pd_phy_fe.last_act_ms < STORE_QUIET_MS)
        return;
    /* UFCS 前端的空闲判断在 fe_flash_safe 里（fe_ufcs）；探测固件不跑协议桥，只看 UFCS 引擎 */
#ifdef UFCS_PROBE
    if (ufcs_session_active())
    {
        if (!ufcs_flash_safe())
            return;
    }
    else
#endif
    if (!fe_flash_safe() || !be_flash_safe() || !bridge_flash_safe())
    {
        return;
    }

    if (cfg_flush_step() || evlog_flush_step())
        last_op_ms = millis();
}
