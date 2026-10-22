// porttool_match.c
//
// pt.run match - see porttool_match.h.
//
// Reply (one OK line; the token numbers are the PC's, this only reports them):
//   OK match busy=0 ms=3000 baud=115200 bps=500000 rs485=2 rs485_ok=1
//      rs485_bytes=72 rs485_junk=0 can=4 can_ok=1 can_frames=9 can_junk=0
//      can_lec=7
//   OK match busy=1        an rs485 or can session owns the terminals
//
// RS485 only receives here: the driver stays off, so several adapters on one
// pair can all be heard. CAN opens in normal mode so the board acknowledges
// the adapter's frames; without an acknowledge it would resend forever.

#include "port_can.h"        /* must precede main.h - see its header */

#include "porttool_match.h"
#include "porttool.h"
#include "porttool_cmd.h"
#include "port_rs485.h"

#include "main.h"
#include <stdio.h>
#include <string.h>

#if PORTTOOL_ENABLE

#define MATCH_MS_DEFAULT   3000U
#define MATCH_MS_MIN        200U
#define MATCH_MS_MAX      10000U
#define MATCH_BPS_DEFAULT 500000U

#define MATCH_TOKENS_MAX  8U
#define MATCH_PREFIX      "OPLC-"
#define MATCH_PREFIX_LEN  5U
#define MATCH_DIGITS_MAX  5U

/* Ends the window when the clock stops moving - the host harness's tick only
 * moves between commands. On a board the tick moves every millisecond, far
 * sooner than this many passes. */
#define MATCH_STALL_SPINS 1000000U

/* A received line longer than any token is junk. */
#define MATCH_LINE_MAX    24U

typedef struct {
    uint32_t tok[MATCH_TOKENS_MAX];
    uint32_t n;
    uint32_t junk;
} match_seen_t;

static void seen_add(match_seen_t *s, uint32_t t)
{
    for (uint32_t i = 0; i < s->n; i++) {
        if (s->tok[i] == t) {
            return;
        }
    }
    if (s->n < MATCH_TOKENS_MAX) {
        s->tok[s->n++] = t;
    }
}

/* "OPLC-<1..5 digits>" exactly; anything else is not a token. */
static int parse_token(const uint8_t *p, uint32_t len, uint32_t *out)
{
    uint32_t v = 0;

    if (len <= MATCH_PREFIX_LEN || len > MATCH_PREFIX_LEN + MATCH_DIGITS_MAX ||
        memcmp(p, MATCH_PREFIX, MATCH_PREFIX_LEN) != 0) {
        return 0;
    }
    for (uint32_t i = MATCH_PREFIX_LEN; i < len; i++) {
        if (p[i] < '0' || p[i] > '9') {
            return 0;
        }
        v = v * 10U + (uint32_t)(p[i] - '0');
    }
    *out = v;
    return 1;
}

static void seen_list(const match_seen_t *s, char *out, uint32_t len)
{
    uint32_t n = 0;

    if (s->n == 0U) {
        (void)snprintf(out, len, "-");
        return;
    }
    out[0] = '\0';
    for (uint32_t i = 0; i < s->n && n < len; i++) {
        int w = snprintf(out + n, len - n, "%s%lu", (i != 0U) ? "," : "",
                         (unsigned long)s->tok[i]);
        if (w < 0) {
            break;
        }
        n += (uint32_t)w;
    }
}

void PortMatch_Run(const char *args, char *out, uint32_t len)
{
    uint32_t ms = MATCH_MS_DEFAULT;
    uint32_t baud = PORT_RS485_DEFAULT_BAUD;
    uint32_t bps = MATCH_BPS_DEFAULT;
    uint8_t rate = PORT_CAN_RATE_DEFAULT;
    match_seen_t rs = {0}, cn = {0};
    uint8_t line[MATCH_LINE_MAX];
    uint32_t line_len = 0, rs_bytes = 0, can_frames = 0, lec = 0;
    int rs_ok, can_ok;
    char rs_list[64], can_list[64];

    if (PortTool_PortRunning("rs485") || PortTool_PortRunning("can")) {
        (void)snprintf(out, len, "busy=1");
        return;
    }

    (void)PortCmd_GetU32(args, "ms", &ms);
    if (ms < MATCH_MS_MIN) { ms = MATCH_MS_MIN; }
    if (ms > MATCH_MS_MAX) { ms = MATCH_MS_MAX; }
    /* The rates the rs485 session takes; anything else falls back. */
    if (PortCmd_GetU32(args, "baud", &baud) &&
        baud != 9600U && baud != 19200U && baud != 38400U &&
        baud != 57600U && baud != 115200U) {
        baud = PORT_RS485_DEFAULT_BAUD;
    }
    if (PortCmd_GetU32(args, "bps", &bps) && !PortCan_RateIndex(bps, &rate)) {
        rate = PORT_CAN_RATE_DEFAULT;
    }
    bps = PortCan_RateBps(rate);

    rs_ok = PortRs485_Init(baud);
    if (rs_ok) {
        uint8_t b;
        PortRs485_DriveEnable(0);
        while (PortRs485_RecvByte(&b)) {
            /* stale bytes from before the window are not this run's */
        }
    }
    can_ok = PortCan_Open(rate, FDCAN_MODE_NORMAL, 0) && PortCan_Alive();
    if (can_ok) {
        (void)PortCan_LastError();   /* reset-on-read: start the window clean */
    }

    printf("MATCH_TEST: listening on RS485 at %lu and CAN at %lu for %lu ms\r\n",
           (unsigned long)baud, (unsigned long)bps, (unsigned long)ms);

    {
        uint32_t start = HAL_GetTick();
        uint32_t last = start;
        uint32_t stall = 0;

        for (;;) {
            uint8_t b;
            uint32_t id, now, tok;
            uint8_t d[8], dl;

            while (rs_ok && PortRs485_RecvByte(&b)) {
                rs_bytes++;
                if (b == '\r' || b == '\n') {
                    if (line_len == 0U) {
                        continue;
                    }
                    /* An over-long line is junk; it was too long for any token. */
                    if (line_len <= sizeof(line) && parse_token(line, line_len, &tok)) {
                        seen_add(&rs, tok);
                    } else {
                        rs.junk++;
                    }
                    line_len = 0U;
                } else if (line_len < sizeof(line)) {
                    line[line_len++] = b;
                } else {
                    line_len = sizeof(line) + 1U;
                }
            }
            while (can_ok && PortCan_Receive(&id, d, &dl)) {
                can_frames++;
                if (parse_token(d, dl, &tok)) {
                    seen_add(&cn, tok);
                } else {
                    cn.junk++;
                }
            }

            now = HAL_GetTick();
            if ((uint32_t)(now - start) >= ms) {
                break;
            }
            if (now != last) {
                last = now;
                stall = 0;
            } else if (++stall >= MATCH_STALL_SPINS) {
                break;
            }
        }
    }

    if (can_ok) {
        lec = PortCan_LastError();
        PortCan_Close();
    }
    if (rs_ok) {
        PortRs485_DriveEnable(0);
    }

    seen_list(&rs, rs_list, sizeof(rs_list));
    seen_list(&cn, can_list, sizeof(can_list));
    (void)snprintf(out, len,
                   "busy=0 ms=%lu baud=%lu bps=%lu rs485=%s rs485_ok=%d"
                   " rs485_bytes=%lu rs485_junk=%lu can=%s can_ok=%d"
                   " can_frames=%lu can_junk=%lu can_lec=%lu",
                   (unsigned long)ms, (unsigned long)baud, (unsigned long)bps,
                   rs_list, rs_ok ? 1 : 0, (unsigned long)rs_bytes,
                   (unsigned long)rs.junk, can_list, can_ok ? 1 : 0,
                   (unsigned long)can_frames, (unsigned long)cn.junk,
                   (unsigned long)lec);
}

#endif /* PORTTOOL_ENABLE */
