// porttool_agebg.h
//
// The aging items that run all the time: one background worker per item,
// serviced from the main loop a small slice at a time.
// $PROD/docs/modules/M4/PORTTOOL-FLOW.md C.3.9 「所有项同时跑」.

#ifndef TESTCASE_PORTTOOL_PORTTOOL_AGEBG_H_
#define TESTCASE_PORTTOOL_PORTTOOL_AGEBG_H_

#include <stdint.h>

typedef enum {
    AGE_BG_NONE = 0,   /* not a worker: a per-round or a 10-minute item */
    AGE_BG_SDRAM,
    AGE_BG_DODI,
    AGE_BG_AOAI,
    AGE_BG_RS232,
    AGE_BG_RS485,
    AGE_BG_CAN,
    AGE_BG_ETH,
    AGE_BG_KNX,
    AGE_BG_USB,
} age_bg_t;

/* The worker behind a task target ("bg.dodi" ...), AGE_BG_NONE for others. */
age_bg_t PortAgeBg_Kind(const char *target);

/* Brings the worker's hardware up and starts its round counters. */
void PortAgeBg_Start(age_bg_t k, const char *args, uint32_t now_ms);

/* One slice of work; returns quickly. */
void PortAgeBg_Service(age_bg_t k, uint32_t now_ms);

/* At a round end: 1 with the reply in out when there is something to judge.
 * Starts the next round's counters either way. */
int PortAgeBg_Report(age_bg_t k, char *out, uint32_t len, uint32_t now_ms);

/* Outputs released, buses quiet. */
void PortAgeBg_Stop(age_bg_t k);

/* A "~age<n>" line that came back through the RS232 shorting plug. */
void PortAgeBg_LineSeen(const char *line);

#endif /* TESTCASE_PORTTOOL_PORTTOOL_AGEBG_H_ */
