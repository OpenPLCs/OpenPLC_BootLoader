/*
 * bor_check.h -- is the brown-out reset at the agreed level?
 * Decision: $PROD/docs/modules/M1/BOOT-SEQUENCE.md, "开机检查欠压复位（BOR）档位".
 */
#ifndef BOR_CHECK_H_
#define BOR_CHECK_H_

/* Reads the BOR level from the option bytes and prints one warning line when
 * it is not level 3 (2.7 V). Never writes the option bytes: the production
 * station sets them with ST-Link ($PROD/docs/modules/M1/BOOT-SEQUENCE.md). */
void bor_level_report(void);

#endif /* BOR_CHECK_H_ */
