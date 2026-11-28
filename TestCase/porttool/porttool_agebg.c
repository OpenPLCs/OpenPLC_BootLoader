// porttool_agebg.c
//
// The aging items' background workers - see porttool_agebg.h.
// What each one does and judges: $PROD/docs/modules/M4/PORTTOOL-FLOW.md C.3.9.

#include "port_can.h"        /* must precede main.h - see its header */
#include "port_rs485.h"

#include "porttool.h"
#include "porttool_agebg.h"
#include "porttool_cmd.h"
#include "port_din.h"
#include "port_dout.h"
#include "port_adc.h"
#include "port_dac.h"
#include "port_vref.h"
#include "relay.h"
#include "KNX/knx_test.h"
#include "SDRAM/sdram_test.h"

#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if PORTTOOL_ENABLE

#if !PORTTOOL_HOST_TEST
#include "lwip.h"
#include "lwip/netif.h"
#include "ethernetif.h"
#include "lan8742.h"
#endif

#define SDRAM_SLICE_MS   10U      /* per service call; C.3.9 aims at <= ~20 ms */
#define DODI_FAST_MS     100U     /* DO7/DO8 toggle */
#define DODI_SETTLE_MS   30U      /* relay + DO -> DI, then check */
#define DODI_RELAY_DEF   10000U
#define DODI_RELAY_MIN   100U
#define AOAI_SWITCH_MS   2000U
#define AOAI_READ_MS     50U
#define RS232_PERIOD_MS  200U
#define RS485_PERIOD_MS  10U
#define CAN_CHECK_MS     1000U
#define CAN_ID           0x7A5U
#define KNX_PERIOD_MS    200U
#define ETH_TX_MS        100U
#define ETH_LINK_MS      1000U
#define ETH_ETHERTYPE    0x88B5U
#define ETH_FRAME_LEN    60U      /* minimum frame; the MAC appends the CRC */

age_bg_t PortAgeBg_Kind(const char *target)
{
    static const struct { const char *name; age_bg_t kind; } map[] = {
        { "bg.sdram", AGE_BG_SDRAM }, { "bg.dodi",  AGE_BG_DODI },
        { "bg.aoai",  AGE_BG_AOAI },  { "bg.rs232", AGE_BG_RS232 },
        { "bg.rs485", AGE_BG_RS485 }, { "bg.can",   AGE_BG_CAN },
        { "bg.eth",   AGE_BG_ETH },   { "bg.knx",   AGE_BG_KNX },
        { "bg.usb",   AGE_BG_USB },
    };

    for (uint32_t i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
        if (strcmp(target, map[i].name) == 0) {
            return map[i].kind;
        }
    }
    return AGE_BG_NONE;
}

static int due(uint32_t now_ms, uint32_t at_ms)
{
    return (int32_t)(now_ms - at_ms) >= 0;
}

/* ---- SDRAM: 64 MiB, four patterns, in slices --------------------------------- */

#if !PORTTOOL_HOST_TEST
static sdram_bg_t sdr;
#endif
static uint32_t sdr_reported;

static void sdram_start(void)
{
#if !PORTTOOL_HOST_TEST
    memset(&sdr, 0, sizeof(sdr));
#endif
    sdr_reported = 0;
}

static void sdram_service(void)
{
#if !PORTTOOL_HOST_TEST
    (void)SDRAM_Test_BgStep(&sdr, SDRAM_SLICE_MS);
#endif
}

static int sdram_report(char *out, uint32_t len)
{
#if PORTTOOL_HOST_TEST
    /* The stub sweep stands in for one finished sweep per round. */
    sdram_sweep_t s;

    (void)SDRAM_Test_SweepOnce(&s);
    snprintf(out, len, "ready=%u sweeps=%lu mismatches=%lu first_bad=0x%08lX sim=1",
             (unsigned)s.ready, (unsigned long)++sdr_reported,
             (unsigned long)s.mismatches, (unsigned long)s.first_bad_offset);
    return 1;
#else
    if (sdr.started && !sdr.ready) {
        snprintf(out, len, "ready=0 sweeps=0 mismatches=0 first_bad=0x00000000");
        return 1;
    }
    if (sdr.sweeps == sdr_reported) {
        return 0;   /* no sweep finished since the last report */
    }
    sdr_reported = sdr.sweeps;
    snprintf(out, len, "ready=1 sweeps=%lu mismatches=%lu first_bad=0x%08lX",
             (unsigned long)sdr.sweeps, (unsigned long)sdr.last_mismatches,
             (unsigned long)sdr.last_first_bad);
    return 1;
#endif
}

/* ---- DO -> DI through the harness -------------------------------------------- */

#if !PORTTOOL_HOST_TEST
static struct {
    int      init_ok;
    uint32_t full, relay_ms;
    uint32_t t78;            /* next DO7/DO8 check and toggle */
    int      lvl;            /* DO7 level; DO8 is the inverse */
    int      k;              /* DO1-6 channel in turn, 1-based */
    int      phase;          /* 0 waiting, 1 on and settling, 2 off and settling */
    uint32_t t_phase, t_next;
    int      failed;
    char     why[48];
} dodi;

static void dodi_fail(int ch, uint8_t want, uint8_t got)
{
    if (!dodi.failed) {
        dodi.failed = 1;
        snprintf(dodi.why, sizeof(dodi.why), "ch=%d want=0x%02X got=0x%02X", ch, want, got);
    }
}

static void dodi_set78(void)
{
    PortDout_SetDuty(7, dodi.lvl ? dodi.full : 0U);
    PortDout_SetDuty(8, dodi.lvl ? 0U : dodi.full);
}
#endif

static void dodi_start(const char *args, uint32_t now_ms)
{
#if PORTTOOL_HOST_TEST
    (void)args;
    (void)now_ms;
#else
    uint32_t v;

    memset(&dodi, 0, sizeof(dodi));
    dodi.relay_ms = DODI_RELAY_DEF;
    if (PortCmd_GetU32(args, "relay_ms", &v)) {
        dodi.relay_ms = v < DODI_RELAY_MIN ? DODI_RELAY_MIN : v;
    }
    PortDin_Init();
    Relay_Init();
    dodi.init_ok = PortDout_Init();
    if (!dodi.init_ok) {
        return;
    }
    for (int k = 1; k <= PORT_DOUT_COUNT; k++) {
        PortDout_SetFreq(k, 1000U);
        PortDout_SetDuty(k, 0U);
    }
    dodi.full = (1UL << PortDout_Res()) - 1UL;
    dodi_set78();
    dodi.t78 = now_ms + DODI_FAST_MS;
    dodi.k = 1;
    dodi.t_next = now_ms;
#endif
}

static void dodi_service(uint32_t now_ms)
{
#if PORTTOOL_HOST_TEST
    (void)now_ms;
#else
    if (!dodi.init_ok) {
        return;
    }
    if (due(now_ms, dodi.t78)) {
        /* DI7 follows DO7, DI8 follows DO8; DI1-6 masked off. */
        uint8_t want = (uint8_t)(dodi.lvl ? 0x40U : 0x80U);
        uint8_t got = (uint8_t)(PortDin_ReadBits() & 0xC0U);
        if (got != want) {
            dodi_fail(((got ^ want) & 0x40U) ? 7 : 8, want, got);
        }
        dodi.lvl = !dodi.lvl;
        dodi_set78();
        dodi.t78 = now_ms + DODI_FAST_MS;
    }

    switch (dodi.phase) {
    case 0:
        if (due(now_ms, dodi.t_next)) {
            Relay_On((RELAY_Name)(dodi.k - 1));
            PortDout_SetDuty(dodi.k, dodi.full);
            dodi.t_phase = now_ms + DODI_SETTLE_MS;
            dodi.t_next += dodi.relay_ms;
            if (due(now_ms, dodi.t_next)) {
                dodi.t_next = now_ms + dodi.relay_ms;   /* fell behind a blocking item */
            }
            dodi.phase = 1;
        }
        break;
    case 1:
        if (due(now_ms, dodi.t_phase)) {
            uint8_t want = (uint8_t)(1U << (dodi.k - 1));
            uint8_t got = (uint8_t)(PortDin_ReadBits() & 0x3FU);
            if (got != want) {
                dodi_fail(dodi.k, want, got);
            }
            PortDout_SetDuty(dodi.k, 0U);
            Relay_Off((RELAY_Name)(dodi.k - 1));
            dodi.t_phase = now_ms + DODI_SETTLE_MS;
            dodi.phase = 2;
        }
        break;
    default:
        if (due(now_ms, dodi.t_phase)) {
            uint8_t got = (uint8_t)(PortDin_ReadBits() & 0x3FU);
            if (got != 0U) {
                dodi_fail(dodi.k, 0U, got);
            }
            dodi.k = dodi.k % (int)RELAY_COUNT + 1;
            dodi.phase = 0;
        }
        break;
    }
#endif
}

static int dodi_report(char *out, uint32_t len)
{
#if PORTTOOL_HOST_TEST
    snprintf(out, len, "ok=1 sim=1");   /* no harness on the simulated board */
#else
    if (!dodi.init_ok) {
        snprintf(out, len, "ok=0 err=dout_init");
    } else if (dodi.failed) {
        snprintf(out, len, "ok=0 %s", dodi.why);
    } else {
        snprintf(out, len, "ok=1");
    }
    dodi.failed = 0;
#endif
    return 1;
}

static void dodi_stop(void)
{
#if !PORTTOOL_HOST_TEST
    if (dodi.init_ok) {
        PortDout_AllOff();
    }
    for (int r = 0; r < (int)RELAY_COUNT; r++) {
        Relay_Off((RELAY_Name)r);
    }
    dodi.init_ok = 0;
#endif
}

/* ---- AO -> AI through the harness -------------------------------------------- */

static struct {
    int      init_ok, args_ok, read_err;
    uint32_t mv[2][2];        /* [channel][0 lo, 1 hi] */
    int      pt;
    int      need_read;
    uint32_t t_switch;
    uint32_t mn[2][2], mx[2][2];
    uint32_t reads[2];        /* per point this round */
} aoai;

static void aoai_set(uint32_t now_ms)
{
    (void)PortDac_SetMv(1, aoai.mv[0][aoai.pt]);
    (void)PortDac_SetMv(2, aoai.mv[1][aoai.pt]);
    aoai.t_switch = now_ms;
    aoai.need_read = 1;
}

static void aoai_round_reset(void)
{
    for (int c = 0; c < 2; c++) {
        for (int p = 0; p < 2; p++) {
            aoai.mn[c][p] = UINT32_MAX;
            aoai.mx[c][p] = 0U;
        }
    }
    aoai.reads[0] = aoai.reads[1] = 0U;
    aoai.read_err = 0;
}

static void aoai_start(const char *args, uint32_t now_ms)
{
    memset(&aoai, 0, sizeof(aoai));
    aoai_round_reset();
    aoai.args_ok = PortCmd_GetU32(args, "lo1", &aoai.mv[0][0])
                && PortCmd_GetU32(args, "hi1", &aoai.mv[0][1])
                && PortCmd_GetU32(args, "lo2", &aoai.mv[1][0])
                && PortCmd_GetU32(args, "hi2", &aoai.mv[1][1]);
    if (!aoai.args_ok) {
        return;
    }
    aoai.init_ok = PortVref_Enable() && PortAdc_Init() && PortDac_Init();
    if (aoai.init_ok) {
        aoai_set(now_ms);
    }
}

static void aoai_service(uint32_t now_ms)
{
    if (!aoai.init_ok) {
        return;
    }
    if (aoai.need_read && now_ms - aoai.t_switch >= AOAI_READ_MS) {
        aoai.need_read = 0;
        for (int c = 0; c < 2; c++) {
            uint32_t raw = 0, pin = 0;
            if (!PortAdc_ReadAin(c + 1, &raw, &pin)) {
                aoai.read_err = 1;
                return;
            }
            if (raw < aoai.mn[c][aoai.pt]) { aoai.mn[c][aoai.pt] = raw; }
            if (raw > aoai.mx[c][aoai.pt]) { aoai.mx[c][aoai.pt] = raw; }
        }
        aoai.reads[aoai.pt]++;
    }
    if (now_ms - aoai.t_switch >= AOAI_SWITCH_MS) {
        aoai.pt ^= 1;
        aoai_set(now_ms);
    }
}

static int aoai_report(char *out, uint32_t len)
{
    int has = 1;

    if (!aoai.args_ok) {
        snprintf(out, len, "ok=0 err=args");
    } else if (!aoai.init_ok) {
        snprintf(out, len, "ok=0 err=analog_init");
    } else if (aoai.read_err) {
        snprintf(out, len, "ok=0 err=ain");
    } else if (aoai.reads[0] == 0U || aoai.reads[1] == 0U) {
        has = 0;   /* a point was not read this round (a blocking item ran) */
    } else {
        snprintf(out, len,
                 "raw1lo_min=%lu raw1lo_max=%lu raw1hi_min=%lu raw1hi_max=%lu"
                 " raw2lo_min=%lu raw2lo_max=%lu raw2hi_min=%lu raw2hi_max=%lu",
                 (unsigned long)aoai.mn[0][0], (unsigned long)aoai.mx[0][0],
                 (unsigned long)aoai.mn[0][1], (unsigned long)aoai.mx[0][1],
                 (unsigned long)aoai.mn[1][0], (unsigned long)aoai.mx[1][0],
                 (unsigned long)aoai.mn[1][1], (unsigned long)aoai.mx[1][1]);
    }
    aoai_round_reset();
    return has;
}

static void aoai_stop(void)
{
    if (aoai.init_ok) {
        (void)PortDac_SetMv(1, 0U);
        (void)PortDac_SetMv(2, 0U);
    }
    aoai.init_ok = 0;
}

/* ---- RS232: C05 and C06 shorted, every line comes back ------------------------ */

static struct {
    uint32_t seq, last_back, t_next, t_sent;
    uint32_t sent, back, lost;   /* this round */
} rs;

static void rs232_start(uint32_t now_ms)
{
    memset(&rs, 0, sizeof(rs));
    rs.t_next = now_ms;
}

static void rs232_service(uint32_t now_ms)
{
#if PORTTOOL_HOST_TEST
    (void)now_ms;   /* the simulated board's control port is the PC, not a plug */
#else
    if (!due(now_ms, rs.t_next)) {
        return;
    }
    rs.seq++;
    printf("~age%lu\r\n", (unsigned long)rs.seq);
    rs.sent++;
    rs.t_sent = HAL_GetTick();
    rs.t_next = rs.t_sent + RS232_PERIOD_MS;
#endif
}

void PortAgeBg_LineSeen(const char *line)
{
    char *end;
    unsigned long n = strtoul(line + 4, &end, 10);

    if (*end != '\0' || n <= rs.last_back || n > rs.seq) {
        return;
    }
    rs.lost += (uint32_t)(n - rs.last_back - 1U);   /* lines skipped on the way */
    rs.last_back = (uint32_t)n;
    rs.back++;
}

static int rs232_report(char *out, uint32_t len, uint32_t now_ms)
{
#if PORTTOOL_HOST_TEST
    (void)now_ms;
    snprintf(out, len, "ok=1 sent=0 back=0 sim=1");
#else
    /* Lines younger than one period may still be on the way. */
    uint32_t must = (rs.seq > 0U && now_ms - rs.t_sent < RS232_PERIOD_MS) ? rs.seq - 1U : rs.seq;
    int ok = rs.lost == 0U && rs.last_back >= must;

    snprintf(out, len, "ok=%d sent=%lu back=%lu", ok, (unsigned long)rs.sent,
             (unsigned long)rs.back);
#endif
    rs.sent = rs.back = rs.lost = 0U;
    return 1;
}

/* ---- RS485, CAN, KNX: kept busy, judged at step 5 (decision 201) -------------- */

static const uint8_t rs485_line[] = "~age485\r\n";
static uint32_t rs485_next;
static int      rs485_ok;

static void rs485_start(uint32_t now_ms)
{
    rs485_ok = PortRs485_Init(PORT_RS485_DEFAULT_BAUD);
    if (rs485_ok) {
        PortRs485_DriveEnable(1);
    }
    rs485_next = now_ms;
}

static void rs485_service(uint32_t now_ms)
{
    if (rs485_ok && due(now_ms, rs485_next)) {
        (void)PortRs485_SendRaw(rs485_line, (uint16_t)(sizeof(rs485_line) - 1U));
        rs485_next = now_ms + RS485_PERIOD_MS;
    }
}

static void rs485_stop(void)
{
    if (rs485_ok) {
        PortRs485_DriveEnable(0);
    }
    rs485_ok = 0;
}

static const uint8_t can_data[8] = { 'a', 'g', 'e', 0x55, 0xAA, 0x00, 0xFF, 0x5A };
static uint32_t can_next;
static int      can_ok;

/* Normal mode, auto-retransmission: with no node to acknowledge, the controller
 * resends the one queued frame forever. */
static void can_open(void)
{
    can_ok = PortCan_Open(PORT_CAN_RATE_DEFAULT, FDCAN_MODE_NORMAL, 1)
          && PortCan_Send(CAN_ID, can_data, (uint8_t)sizeof(can_data));
}

static void can_service(uint32_t now_ms)
{
    if (!due(now_ms, can_next)) {
        return;
    }
    can_next = now_ms + CAN_CHECK_MS;
    if (!can_ok) {
        can_open();
        return;
    }
#if !PORTTOOL_HOST_TEST
    if (PortCan_TxPending() == 0U) {
        (void)PortCan_Send(CAN_ID, can_data, (uint8_t)sizeof(can_data));
    }
#endif
}

static uint32_t knx_next;
static int      knx_ok;

static void knx_service(uint32_t now_ms)
{
    uint8_t b, framing;

    if (!knx_ok) {
        return;
    }
    while (KNX_Test_SessionPollChar(&b, &framing)) {
        /* drained only: nothing on KNX is judged during aging */
    }
    if (due(now_ms, knx_next)) {
        (void)KNX_Test_SessionSendChar(0x55U);   /* spins about 1.35 ms */
        knx_next = now_ms + KNX_PERIOD_MS;
    }
}

/* ---- Ethernet: broadcast frames to the switch, PHY link watched ---------------- */

static struct {
    int      up_ok;
    int      ever_up;         /* link seen up since start: earlier downs are autonegotiation */
    int      link, up_reads, down_reads;
    uint32_t drops, tx, tx_err, seq;
    uint32_t t_tx, t_link;
} eth;

#if !PORTTOOL_HOST_TEST

extern porttool_port_t  porttool_eth;   /* porttool_eth.c */
extern lan8742_Object_t LAN8742;        /* LWIP/Target/ethernetif.c */

static uint8_t eth_frame[ETH_FRAME_LEN];

/* lwIP comes up only through the eth session, which owns the once-only
 * MX_LWIP_Init; PortEth_Poll then keeps the link handling running. */
static int eth_lwip_up(void)
{
    char err[PORTTOOL_REPLY_MAX];

    if (netif_default == NULL) {
        (void)porttool_eth.start("", err, sizeof(err));
        porttool_eth.stop();
    }
    return netif_default != NULL && LAN8742.Is_Initialized;
}

/* Both reads must show the link: BSR link status latches low. */
static int eth_phy_link(void)
{
    uint32_t a = 0, b = 0;

    if (LAN8742.IO.ReadReg(LAN8742.DevAddr, LAN8742_BSR, &a) < 0
        || LAN8742.IO.ReadReg(LAN8742.DevAddr, LAN8742_BSR, &b) < 0) {
        return 0;
    }
    return (a & b & LAN8742_BSR_LINK_STATUS) != 0U;
}

static int eth_send(void)
{
    ETH_BufferTypeDef         buf = { 0 };
    ETH_TxPacketConfigTypeDef cfg = { 0 };

    memset(&eth_frame[0], 0xFF, 6);
    memcpy(&eth_frame[6], netif_default->hwaddr, 6);
    eth_frame[12] = (uint8_t)(ETH_ETHERTYPE >> 8);
    eth_frame[13] = (uint8_t)ETH_ETHERTYPE;
    eth.seq++;
    eth_frame[14] = (uint8_t)(eth.seq >> 24);
    eth_frame[15] = (uint8_t)(eth.seq >> 16);
    eth_frame[16] = (uint8_t)(eth.seq >> 8);
    eth_frame[17] = (uint8_t)eth.seq;

    buf.buffer = eth_frame;
    buf.len    = ETH_FRAME_LEN;
    cfg.Attributes = ETH_TX_PACKETS_FEATURES_CRCPAD;
    cfg.CRCPadCtrl = ETH_CRC_PAD_INSERT;
    cfg.Length     = ETH_FRAME_LEN;
    cfg.TxBuffer   = &buf;
    return HAL_ETH_Transmit(&heth, &cfg, 20U) == HAL_OK;
}

#endif /* !PORTTOOL_HOST_TEST */

static void eth_start(uint32_t now_ms)
{
    memset(&eth, 0, sizeof(eth));
#if !PORTTOOL_HOST_TEST
    eth.up_ok = eth_lwip_up();
#endif
    eth.t_tx = eth.t_link = now_ms;
}

static void eth_service(uint32_t now_ms)
{
#if PORTTOOL_HOST_TEST
    (void)now_ms;
#else
    if (!eth.up_ok) {
        return;
    }
    if (due(now_ms, eth.t_link)) {
        int link = eth_phy_link();
        eth.t_link = now_ms + ETH_LINK_MS;
        if (link) {
            eth.ever_up = 1;
            eth.up_reads++;
        } else if (eth.ever_up) {
            eth.down_reads++;
            if (eth.link) {
                eth.drops++;
            }
        }
        eth.link = link;
    }
    if (due(now_ms, eth.t_tx)) {
        eth.t_tx = now_ms + ETH_TX_MS;
        /* The MAC runs only while lwIP's link handling has the link up. */
        if (netif_is_link_up(netif_default) && heth.gState == HAL_ETH_STATE_STARTED) {
            if (eth_send()) {
                eth.tx++;
            } else {
                eth.tx_err++;
            }
        }
    }
#endif
}

static int eth_report(char *out, uint32_t len)
{
#if PORTTOOL_HOST_TEST
    snprintf(out, len, "ok=1 link=1 drops=0 tx=0 tx_err=0 sim=1");
#else
    if (!eth.up_ok) {
        snprintf(out, len, "ok=0 link=0 drops=0 tx=0 tx_err=0 err=lwip");
    } else {
        int ok = eth.up_reads > 0 && eth.down_reads == 0 && eth.tx_err == 0U;
        snprintf(out, len, "ok=%d link=%d drops=%lu tx=%lu tx_err=%lu", ok, eth.link,
                 (unsigned long)eth.drops, (unsigned long)eth.tx, (unsigned long)eth.tx_err);
    }
#endif
    eth.up_reads = eth.down_reads = 0;
    eth.drops = eth.tx = eth.tx_err = 0U;
    return 1;
}

/* ---- USB: write whenever the endpoint is free; the PC outside reads ------------ */

static struct {
    int      ever_cfg;        /* configured since start: earlier is enumeration */
    int      cfg_seen, lost;  /* this round */
    uint32_t bytes;           /* this round */
} usb;

static void usb_service(void)
{
#if !PORTTOOL_HOST_TEST
    if (PortUsb_Configured()) {
        usb.ever_cfg = 1;
        usb.cfg_seen = 1;
        usb.bytes += PortUsb_AgePush();
    } else if (usb.ever_cfg) {
        usb.lost = 1;
    }
#endif
}

static int usb_report(char *out, uint32_t len)
{
#if PORTTOOL_HOST_TEST
    snprintf(out, len, "ok=1 cfg=1 tx_bytes=0 sim=1");
#else
    int ok = usb.cfg_seen && !usb.lost && usb.bytes > 0U;

    snprintf(out, len, "ok=%d cfg=%d tx_bytes=%lu", ok, PortUsb_Configured(),
             (unsigned long)usb.bytes);
#endif
    usb.cfg_seen = usb.lost = 0;
    usb.bytes = 0U;
    return 1;
}

/* ---- dispatch ------------------------------------------------------------------- */

void PortAgeBg_Start(age_bg_t k, const char *args, uint32_t now_ms)
{
    switch (k) {
    case AGE_BG_SDRAM: sdram_start(); break;
    case AGE_BG_DODI:  dodi_start(args, now_ms); break;
    case AGE_BG_AOAI:  aoai_start(args, now_ms); break;
    case AGE_BG_RS232: rs232_start(now_ms); break;
    case AGE_BG_RS485: rs485_start(now_ms); break;
    case AGE_BG_CAN:   can_open(); can_next = now_ms + CAN_CHECK_MS; break;
    case AGE_BG_ETH:   eth_start(now_ms); break;
    case AGE_BG_KNX:   knx_ok = KNX_Test_SessionInit(); knx_next = now_ms; break;
    case AGE_BG_USB:   memset(&usb, 0, sizeof(usb)); break;
    default: break;
    }
}

void PortAgeBg_Service(age_bg_t k, uint32_t now_ms)
{
    switch (k) {
    case AGE_BG_SDRAM: sdram_service(); break;
    case AGE_BG_DODI:  dodi_service(now_ms); break;
    case AGE_BG_AOAI:  aoai_service(now_ms); break;
    case AGE_BG_RS232: rs232_service(now_ms); break;
    case AGE_BG_RS485: rs485_service(now_ms); break;
    case AGE_BG_CAN:   can_service(now_ms); break;
    case AGE_BG_ETH:   eth_service(now_ms); break;
    case AGE_BG_KNX:   knx_service(now_ms); break;
    case AGE_BG_USB:   usb_service(); break;
    default: break;
    }
}

int PortAgeBg_Report(age_bg_t k, char *out, uint32_t len, uint32_t now_ms)
{
    switch (k) {
    case AGE_BG_SDRAM: return sdram_report(out, len);
    case AGE_BG_DODI:  return dodi_report(out, len);
    case AGE_BG_AOAI:  return aoai_report(out, len);
    case AGE_BG_RS232: return rs232_report(out, len, now_ms);
    case AGE_BG_ETH:   return eth_report(out, len);
    case AGE_BG_USB:   return usb_report(out, len);
    default:           return 0;   /* RS485, CAN, KNX: not judged here */
    }
}

void PortAgeBg_Stop(age_bg_t k)
{
    switch (k) {
    case AGE_BG_DODI:  dodi_stop(); break;
    case AGE_BG_AOAI:  aoai_stop(); break;
    case AGE_BG_RS485: rs485_stop(); break;
    case AGE_BG_CAN:   PortCan_Close(); can_ok = 0; break;
    case AGE_BG_ETH:   eth.up_ok = 0; break;
    case AGE_BG_KNX:   knx_ok = 0; break;
    default: break;     /* SDRAM, RS232, USB just stop being serviced */
    }
}

#endif /* PORTTOOL_ENABLE */
