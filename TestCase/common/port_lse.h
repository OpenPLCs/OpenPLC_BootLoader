// port_lse.h
//
// LSE (32.768 kHz, XTAL2) drive capability for the fixture: restart the
// oscillator at a chosen drive and time its start-up. The drive cannot change
// while LSE runs, and rev Y stores the two middle drives swapped
// (HARDWARE-FACTS.md 时钟源).

#ifndef TESTCASE_COMMON_PORT_LSE_H_
#define TESTCASE_COMMON_PORT_LSE_H_

#include <stdint.h>

/* Same as the HAL's LSE_TIMEOUT_VALUE. */
#define PORT_LSE_START_TIMEOUT_MS 5000U

/* Drives, weakest first. */
#define PORT_LSE_LOW   0
#define PORT_LSE_MLOW  1
#define PORT_LSE_MHIGH 2
#define PORT_LSE_HIGH  3

/* Stops LSE, sets drv (PORT_LSE_*), starts it again. *start_ms is the time to
 * LSERDY, or the timeout. Returns 1 if LSE is running. The RTC stops counting
 * meanwhile; the backup domain is not reset. */
int PortLse_Restart(int drv, uint32_t *start_ms);

/* The drive in RCC_BDCR as PORT_LSE_*, decoded for the silicon revision. */
int PortLse_Drive(void);

/* 1 while LSE is running (LSERDY). */
int PortLse_Ready(void);

/* One LSE frequency measurement: `ticks` timer clocks of `tclk_hz` spanned
 * `periods` LSE periods, over about `ms` milliseconds. */
typedef struct {
    uint64_t ticks;
    uint32_t periods;
    uint32_t tclk_hz;
    uint32_t ms;
} PortLseFreq;

/* Measures LSE against TIM16's clock (from HSE through the PLL) for about
 * window_ms. Returns 1 with *f filled, 0 if LSE is not running, -1 if a
 * capture was missed every try. */
int PortLse_MeasureFreq(uint32_t window_ms, PortLseFreq *f);

#endif /* TESTCASE_COMMON_PORT_LSE_H_ */
