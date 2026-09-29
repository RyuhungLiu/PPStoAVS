#include "ufcs_codec.h"
#include <string.h>

uint8_t ufcs_crc8(const uint8_t *p, uint8_t n)
{
    uint8_t crc = 0;
    while (n--)
    {
        crc ^= *p++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x29) : (uint8_t)(crc << 1);
    }
    return crc;
}

static void put_header(uint8_t *out, const ufcs_msg_t *m)
{
    uint16_t h = (uint16_t)((m->addr & 7) << 13) | (uint16_t)((m->num & 15) << 9) |
                 (uint16_t)((m->ver & 63) << 3) | (m->type & 7);
    out[0] = h >> 8;
    out[1] = (uint8_t)h;
}

uint8_t ufcs_encode(const ufcs_msg_t *m, uint8_t *out)
{
    uint8_t n;
    put_header(out, m);
    switch (m->type)
    {
    case UFCS_TYPE_CTRL:
        out[2] = m->cmd;
        n = 3;
        break;
    case UFCS_TYPE_DATA:
        if (m->len == 0 || m->len > UFCS_MAX_DATA)
            return 0;
        out[2] = m->cmd;
        out[3] = m->len;
        memcpy(&out[4], m->data, m->len);
        n = 4 + m->len;
        break;
    case UFCS_TYPE_VENDOR:
        if (m->len > UFCS_MAX_DATA - 1)
            return 0;
        out[2] = 0;
        out[3] = 0;
        out[4] = m->len;
        memcpy(&out[5], m->data, m->len);
        n = 5 + m->len;
        break;
    default:
        return 0;
    }
    out[n] = ufcs_crc8(out, n);
    return n + 1;
}

uint8_t ufcs_frame_len(const uint8_t *p, uint8_t have)
{
    if (have < 3)
        return 0;
    switch (p[1] & 7)
    {
    case UFCS_TYPE_CTRL:
        return 4;
    case UFCS_TYPE_DATA:
        if (have < 4)
            return 0;
        return (p[3] == 0 || p[3] > UFCS_MAX_DATA) ? 0xFF : (uint8_t)(5 + p[3]);
    case UFCS_TYPE_VENDOR:
        if (have < 5)
            return 0;
        return (p[4] > UFCS_MAX_DATA - 1) ? 0xFF : (uint8_t)(6 + p[4]);
    default:
        return 0xFF;
    }
}

bool ufcs_decode(const uint8_t *p, uint8_t n, ufcs_msg_t *m, bool *crc_ok)
{
    uint8_t want = ufcs_frame_len(p, n);
    if (want == 0 || want == 0xFF || want != n)
        return false;
    bool ok = ufcs_crc8(p, n - 1) == p[n - 1];
    if (crc_ok)
        *crc_ok = ok;
    uint16_t h = (uint16_t)(p[0] << 8) | p[1];
    memset(m, 0, sizeof(*m));
    m->addr = h >> 13;
    m->num = (h >> 9) & 15;
    m->ver = (h >> 3) & 63;
    m->type = h & 7;
    switch (m->type)
    {
    case UFCS_TYPE_CTRL:
        m->cmd = p[2];
        break;
    case UFCS_TYPE_DATA:
        m->cmd = p[2];
        m->len = p[3];
        memcpy(m->data, &p[4], m->len);
        break;
    default:
        m->len = p[4];
        memcpy(m->data, &p[5], m->len);
        break;
    }
    return ok;
}

void ufcs_make_ctrl(ufcs_msg_t *m, uint8_t addr, uint8_t num, uint8_t cmd)
{
    memset(m, 0, sizeof(*m));
    m->addr = addr;
    m->num = num;
    m->ver = UFCS_VERSION;
    m->type = UFCS_TYPE_CTRL;
    m->cmd = cmd;
}

void ufcs_make_request(ufcs_msg_t *m, uint8_t num, uint8_t mode, uint16_t mv, uint16_t ma)
{
    memset(m, 0, sizeof(*m));
    m->addr = UFCS_ADDR_SOURCE;
    m->num = num;
    m->ver = UFCS_VERSION;
    m->type = UFCS_TYPE_DATA;
    m->cmd = UFCS_D_REQUEST;
    m->len = 8;
    uint16_t v = mv / 10, i = ma / 10;
    m->data[0] = (uint8_t)(mode << 4);
    m->data[4] = v >> 8;
    m->data[5] = (uint8_t)v;
    m->data[6] = i >> 8;
    m->data[7] = (uint8_t)i;
}

void ufcs_make_watchdog(ufcs_msg_t *m, uint8_t num, uint16_t ms)
{
    memset(m, 0, sizeof(*m));
    m->addr = UFCS_ADDR_SOURCE;
    m->num = num;
    m->ver = UFCS_VERSION;
    m->type = UFCS_TYPE_DATA;
    m->cmd = UFCS_D_CONFIG_WATCHDOG;
    m->len = 2;
    m->data[0] = ms >> 8;
    m->data[1] = (uint8_t)ms;
}

void ufcs_make_refuse(ufcs_msg_t *m, uint8_t addr, uint8_t num, const ufcs_msg_t *refused, uint8_t reason)
{
    memset(m, 0, sizeof(*m));
    m->addr = addr;
    m->num = num;
    m->ver = UFCS_VERSION;
    m->type = UFCS_TYPE_DATA;
    m->cmd = UFCS_D_REFUSE;
    m->len = 4;
    m->data[0] = refused->num & 15;
    m->data[1] = refused->type & 7;
    m->data[2] = refused->cmd;
    m->data[3] = reason;
}

void ufcs_make_sink_info(ufcs_msg_t *m, uint8_t addr, uint8_t num, uint16_t mv, uint16_t ma)
{
    memset(m, 0, sizeof(*m));
    m->addr = addr;
    m->num = num;
    m->ver = UFCS_VERSION;
    m->type = UFCS_TYPE_DATA;
    m->cmd = UFCS_D_SINK_INFO;
    m->len = 8;
    uint16_t v = mv / 10, i = ma / 10;
    m->data[4] = v >> 8;
    m->data[5] = (uint8_t)v;
    m->data[6] = i >> 8;
    m->data[7] = (uint8_t)i;
}

bool ufcs_parse_source_info(const ufcs_msg_t *m, int16_t *temp_c, int16_t *port_temp_c, uint16_t *mv, uint16_t *ma)
{
    if (m->type != UFCS_TYPE_DATA || m->cmd != UFCS_D_SOURCE_INFO || m->len != 8)
        return false;
    *temp_c = m->data[2] ? (int16_t)m->data[2] - 50 : -999;
    *port_temp_c = m->data[3] ? (int16_t)m->data[3] - 50 : -999;
    *mv = (uint16_t)(((m->data[4] << 8) | m->data[5]) * 10);
    *ma = (uint16_t)(((m->data[6] << 8) | m->data[7]) * 10);
    return true;
}

bool ufcs_parse_refuse(const ufcs_msg_t *m, uint8_t *num, uint8_t *type, uint8_t *cmd, uint8_t *reason)
{
    if (m->type != UFCS_TYPE_DATA || m->cmd != UFCS_D_REFUSE || m->len != 4)
        return false;
    *num = m->data[0] & 15;
    *type = m->data[1] & 7;
    *cmd = m->data[2];
    *reason = m->data[3];
    return true;
}

uint8_t ufcs_parse_caps(const ufcs_msg_t *m, ufcs_mode_t *modes)
{
    if (m->type != UFCS_TYPE_DATA || m->cmd != UFCS_D_OUTPUT_CAPS || m->len == 0 || m->len % 8)
        return 0;
    uint8_t n = m->len / 8;
    if (n > UFCS_MAX_MODES)
        return 0;
    for (uint8_t k = 0; k < n; k++)
    {
        const uint8_t *d = &m->data[k * 8];
        ufcs_mode_t *md = &modes[k];
        uint8_t cs = (d[0] >> 1) & 7;
        if (cs > 4)
            return 0;
        md->num = d[0] >> 4;
        md->ma_step = (uint8_t)(10 * (cs + 1));
        md->mv_step = (d[0] & 1) ? 20 : 10;
        md->vmax_10mv = (uint16_t)(d[1] << 8) | d[2];
        md->vmin_10mv = (uint16_t)(d[3] << 8) | d[4];
        md->imax_10ma = (uint16_t)(d[5] << 8) | d[6];
        md->imin_10ma = d[7];
    }
    return n;
}

bool ufcs_mode_accepts(const ufcs_mode_t *md, uint16_t mv, uint16_t ma)
{
    uint16_t v = mv / 10, i = ma / 10;
    if (mv % 10 || ma % 10)
        return false;
    if (v < md->vmin_10mv || v > md->vmax_10mv || i < md->imin_10ma || i > md->imax_10ma)
        return false;
    /* 步进相对绝对值：规范没写基准，探测阶段只请求整数值 */
    return (mv % md->mv_step) == 0 && (ma % md->ma_step) == 0;
}

uint8_t ufcs_synth_caps(const ufcs_mode_t *md, uint8_t n, uint32_t raw[7], uint8_t src[7])
{
    static const uint16_t fixed_mv[4] = {5000, 9000, 15000, 20000};
    uint8_t k = 0;

    for (uint8_t f = 0; f < 4; f++)
    {
        uint16_t v = fixed_mv[f];
        int best = -1;
        uint16_t best_ma = 0;
        for (uint8_t i = 0; i < n; i++)
        {
            uint32_t vmin = md[i].vmin_10mv * 10u, vmax = md[i].vmax_10mv * 10u;
            uint16_t ma = md[i].imax_10ma * 10u;
            if (v >= vmin && v <= vmax && (v % md[i].mv_step) == 0 && (best < 0 || ma > best_ma))
            {
                best = i;
                best_ma = ma;
            }
        }
        if (best < 0)
        {
            if (v == 5000)
                return 0;                   /* PD 要求第一个 PDO 是 5V Fixed */
            continue;
        }
        if (best_ma > 5000)
            best_ma = 5000;
        raw[k] = ((uint32_t)(v / 50) << 10) | (best_ma / 10);
        src[k] = md[best].num;
        k++;
    }

    /* PPS：按最高电压升序 */
    uint8_t order[UFCS_MAX_MODES];
    for (uint8_t i = 0; i < n; i++)
        order[i] = i;
    for (uint8_t i = 1; i < n; i++)
    {
        uint8_t x = order[i];
        int j = i - 1;
        while (j >= 0 && md[order[j]].vmax_10mv > md[x].vmax_10mv)
        {
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = x;
    }
    for (uint8_t q = 0; q < n && k < 7; q++)
    {
        const ufcs_mode_t *m = &md[order[q]];
        uint32_t vmin = m->vmin_10mv * 10u, vmax = m->vmax_10mv * 10u;
        if (vmin < 3300)
            vmin = 3300;
        if (vmax > 21000)
            vmax = 21000;
        uint32_t lo = (vmin + 99) / 100, hi = vmax / 100;
        uint32_t cur = m->imax_10ma * 10u / 50;
        if (cur > 127)
            cur = 127;
        if (hi < lo + 10 || cur == 0)
            continue;
        uint32_t r = (3u << 30) | (hi << 17) | (lo << 8) | cur;
        bool dup = false;
        for (uint8_t j = 0; j < k; j++)
            dup |= raw[j] == r;
        if (dup)
            continue;
        raw[k] = r;
        src[k] = m->num;
        k++;
    }
    return k;
}

bool ufcs_map_request(const ufcs_mode_t *m, uint16_t mv, uint16_t ma, uint16_t *out_mv, uint16_t *out_ma)
{
    uint32_t vmin = m->vmin_10mv * 10u, vmax = m->vmax_10mv * 10u;
    uint32_t v = mv;
    v -= v % m->mv_step;
    if (v < vmin)
        v = vmin + (m->mv_step - vmin % m->mv_step) % m->mv_step;
    if (v > vmax)
        v = vmax - vmax % m->mv_step;
    if (v < vmin)
        return false;

    uint32_t imin = m->imin_10ma * 10u, imax = m->imax_10ma * 10u;
    uint32_t c = ma;
    if (c > imax)
        c = imax;
    c -= c % m->ma_step;
    if (c < imin)
        c = imin + (m->ma_step - imin % m->ma_step) % m->ma_step;
    if (c > imax)
        return false;

    *out_mv = (uint16_t)v;
    *out_ma = (uint16_t)c;
    return true;
}
