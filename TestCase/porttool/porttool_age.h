// porttool_age.h
//
// The aging mode: a task in flash bank 2 sector 6 that the board runs by
// itself whenever it is powered, judging each item against limits the PC wrote
// with the task. $PROD/docs/modules/M4/PORTTOOL-FLOW.md C.3.9.

#ifndef TESTCASE_PORTTOOL_PORTTOOL_AGE_H_
#define TESTCASE_PORTTOOL_PORTTOOL_AGE_H_

#include <stdint.h>

/* Reads the task and the records; starts aging when the task is unfinished. */
void PortAge_Boot(uint32_t now_ms);

/* Runs the next item when due, keeps time, saves progress, drives LED3. */
void PortAge_Tick(uint32_t now_ms);

/* pt.age begin|add|commit|read|fail|clear */
void PortAge_Command(const char *rest);

/* 1 while a task is being run. */
int PortAge_Running(void);

/* A command from the PC: stop until the next power-up (the PC, not the
 * shorting plug, is on RS232 now). Progress is saved. */
void PortAge_Pause(void);

/* A received line that starts "~age": the RS232 loop's own token. */
void PortAge_LineSeen(const char *line);

/* LED3 from the 1 ms tick interrupt, so it blinks while an item blocks. */
void PortAge_LedTick(uint32_t now_ms);

#endif /* TESTCASE_PORTTOOL_PORTTOOL_AGE_H_ */
