// port_vref.h
//
// Turns on the MCU's internal VREFBUF at scale 0 (2.5 V), shared by the ADC and
// the DAC: the board has no external reference, and scale 0 matches the analog
// front ends' full scale (HARDWARE-FACTS.md 模拟量). An external reference can
// be wired to VREF+ by hand instead (PORTTOOL-FLOW.md B.4 外部基准).

#ifndef TESTCASE_COMMON_PORT_VREF_H_
#define TESTCASE_COMMON_PORT_VREF_H_

/* Enables VREFBUF at scale 0 and waits for it to settle. Returns 0 on timeout.
 * Must run before any ADC or DAC init. Safe to call more than once. Does
 * nothing while the external reference is selected. */
int PortVref_Enable(void);

/* external=1: VREFBUF off, VREF+ high impedance. external=0: back to VREFBUF
 * at scale 0. Returns 0 if VREFBUF did not come back. The choice is not kept
 * across a reset. Re-measure VREF+ afterwards (PortAdc_Remeasure). */
int PortVref_Select(int external);

/* 1 while the external reference is selected. */
int PortVref_IsExternal(void);

#endif /* TESTCASE_COMMON_PORT_VREF_H_ */
