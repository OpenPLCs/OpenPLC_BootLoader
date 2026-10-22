// porttool_match.h
//
// pt.run match: listens on the RS485 and CAN terminals at once and reports the
// adapter tokens ("OPLC-<n>") each one received, so the PC can tell which of
// its adapters is wired to which terminal (DECISIONS.md 146, PORTTOOL-FLOW.md
// A.3 and C.3.6).

#ifndef TESTCASE_PORTTOOL_PORTTOOL_MATCH_H_
#define TESTCASE_PORTTOOL_PORTTOOL_MATCH_H_

#include <stdint.h>

/* The pt.run body: ms= (default 3000, 200..10000), baud= for RS485 (default
 * 115200), bps= for CAN (default 500000). Writes the k=v body of the OK line. */
void PortMatch_Run(const char *args, char *out, uint32_t len);

#endif /* TESTCASE_PORTTOOL_PORTTOOL_MATCH_H_ */
