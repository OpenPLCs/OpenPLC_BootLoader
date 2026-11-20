// porttool_age.c
//
// The aging mode - see porttool_age.h. Sector 6 of bank 2 holds the task at
// its start and appends 32-byte records from AGE_REC_OFF on.
// $PROD/docs/modules/M4/PORTTOOL-FLOW.md C.3.9 「老化模式怎么实现」.

#include "porttool.h"
#include "porttool_age.h"
#include "porttool_cmd.h"
#include "porttool_run.h"
#include "port_led.h"
#include "port_din.h"
#include "port_dout.h"
#include "relay.h"
#include "port_adc.h"
#include "port_dac.h"
#include "port_vref.h"

#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if PORTTOOL_ENABLE

#ifndef PORTTOOL_AGE_BASE
#define PORTTOOL_AGE_BASE 0x081C0000UL   /* bank 2 sector 6 */
#endif
#define AGE_BASE     ((uintptr_t)PORTTOOL_AGE_BASE)
#define AGE_SIZE     FLASH_SECTOR_SIZE
#define AGE_WORD     32U
#define AGE_REC_OFF  16384U
#define AGE_TEXT_MAX (AGE_REC_OFF - AGE_WORD)
#define AGE_MAGIC    0x31454741UL        /* "AGE1" */
#define AGE_ITEMS    16U
#define AGE_SAVE_MS  600000U             /* a progress record every 10 minutes */
#define AGE_GAP_MS   100U                /* between items, so commands get in */

typedef struct {
    uint32_t magic, secs, text_len, text_crc;
    uint32_t spare[4];
} age_head_t;

/* One record word. 'P' progress, 'F' first failure of an item (text goes on
 * in the 'T' word after it), 'E' finished (code 1 pass, 0 fail). */
typedef struct {
    uint8_t  kind, code, item, spare;
    uint32_t a, b, c;   /* P/E: elapsed s, rounds, fails; F: round, at s */
    char     text[16];
} age_rec_t;

_Static_assert(sizeof(age_head_t) == AGE_WORD && sizeof(age_rec_t) == AGE_WORD,
               "records are whole flash words");

typedef struct {
    uint8_t code;
    char    target[24];
    char    args[64];
    char    checks[160];
} age_item_t;

static age_item_t items[AGE_ITEMS];
static uint32_t   n_items, task_secs;
static int        running, done, passed;
static uint32_t   elapsed_ms, rounds, fails, failed_mask, first_code;
static uint32_t   next_item, due_ms, last_ms, saved_ms, rec_off;

static char     stage[AGE_TEXT_MAX];
static uint32_t stage_len, stage_secs;
static int      staging;

static uint8_t  word_buf[AGE_WORD] __attribute__((aligned(32)));

/* ---- flash ---------------------------------------------------------------- */

static int age_erase(void)
{
    FLASH_EraseInitTypeDef e;
    uint32_t bad = 0;

    memset(&e, 0, sizeof(e));
    e.TypeErase    = FLASH_TYPEERASE_SECTORS;
    e.Banks        = FLASH_BANK_2;
    e.Sector       = FLASH_SECTOR_6;
    e.NbSectors    = 1U;
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    return HAL_FLASHEx_Erase(&e, &bad) == HAL_OK;
}

/* Programs one word at off; nothing outside sector 6 is ever handed to the HAL. */
static int age_program(uint32_t off, const void *src)
{
    if (off > AGE_SIZE - AGE_WORD || off % AGE_WORD != 0U) {
        return 0;
    }
    memcpy(word_buf, src, AGE_WORD);
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, AGE_BASE + off,
                             (uintptr_t)word_buf) == HAL_OK;
}

static int age_append(const age_rec_t *r)
{
    int ok;

    if (rec_off > AGE_SIZE - AGE_WORD) {
        return 0;   /* full: the end record and counters still live in RAM */
    }
    HAL_FLASH_Unlock();
    ok = age_program(rec_off, r);
    HAL_FLASH_Lock();
    if (ok) {
        rec_off += AGE_WORD;
    }
    return ok;
}

/* Copies flash into RAM without a bus fault on a word left half-written by a
 * power cut (uncorrectable ECC): fault masked and BFHFNMIGN set while reading,
 * the bank 2 DBECCERR flag says whether it happened. Returns 0 when it did. */
static int age_copy(void *dst, uintptr_t src, uint32_t n)
{
#ifdef SCB_CCR_BFHFNMIGN_Msk
    uint32_t primask = __get_PRIMASK();
    int bad;

    __disable_irq();
    __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_SNECCERR_BANK2 | FLASH_FLAG_DBECCERR_BANK2);
    __set_FAULTMASK(1);
    SCB->CCR |= SCB_CCR_BFHFNMIGN_Msk;
    __DSB();
    __ISB();
    memcpy(dst, (const void *)src, n);
    SCB->CCR &= ~SCB_CCR_BFHFNMIGN_Msk;
    __DSB();
    __ISB();
    __set_FAULTMASK(0);
    bad = __HAL_FLASH_GET_FLAG_BANK2(FLASH_FLAG_DBECCERR_BANK2) != 0U;
    __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_SNECCERR_BANK2 | FLASH_FLAG_DBECCERR_BANK2);
    if (primask == 0U) {
        __enable_irq();
    }
    return !bad;
#else
    memcpy(dst, (const void *)src, n);   /* host build: no ECC */
    return 1;
#endif
}

/* ---- the task ------------------------------------------------------------- */

/* "<code> <target> <args>|<checks>"; args may be empty. */
static int parse_item(const char *line, age_item_t *it)
{
    const char *bar = strchr(line, '|');
    const char *p = line;
    char *end;
    unsigned long code;
    uint32_t n;

    if (bar == NULL) {
        return 0;
    }
    code = strtoul(p, &end, 10);
    if (end == p || *end != ' ' || code == 0UL || code > 32UL) {
        return 0;
    }
    p = end + 1;
    n = 0;
    while (p < bar && *p != ' ' && n + 1U < sizeof(it->target)) {
        it->target[n++] = *p++;
    }
    it->target[n] = '\0';
    if (n == 0U || (p < bar && *p != ' ')) {
        return 0;
    }
    if (p < bar) {
        p++;
    }
    if ((uint32_t)(bar - p) >= sizeof(it->args) || strlen(bar + 1) >= sizeof(it->checks)
        || bar[1] == '\0') {
        return 0;
    }
    memcpy(it->args, p, (size_t)(bar - p));
    it->args[bar - p] = '\0';
    strcpy(it->checks, bar + 1);
    it->code = (uint8_t)code;
    return 1;
}

static int load_items(const char *text, uint32_t len)
{
    char line[PORTTOOL_LINE_MAX];
    uint32_t i = 0, n = 0;

    n_items = 0;
    while (i < len) {
        uint32_t k = 0;
        while (i < len && text[i] != '\n' && k + 1U < sizeof(line)) {
            line[k++] = text[i++];
        }
        line[k] = '\0';
        while (i < len && text[i] != '\n') { i++; }
        i++;
        if (k == 0U) { continue; }
        if (n >= AGE_ITEMS || !parse_item(line, &items[n])) {
            return 0;
        }
        n++;
    }
    n_items = n;
    return n > 0U;
}

/* ---- judging -------------------------------------------------------------- */

/* The value of field (with an optional [n] picking a /-separated part) in a
 * reply of space-separated key=value words; "_text" is the whole reply. */
static int field_value(const char *reply, const char *field, char *out, uint32_t len)
{
    char key[32];
    const char *br = strchr(field, '[');
    uint32_t idx = 0, klen;
    const char *p = reply;

    if (strcmp(field, "_text") == 0) {
        snprintf(out, len, "%s", reply);
        return 1;
    }
    klen = br != NULL ? (uint32_t)(br - field) : (uint32_t)strlen(field);
    if (klen == 0U || klen >= sizeof(key)) {
        return 0;
    }
    memcpy(key, field, klen);
    key[klen] = '\0';
    if (br != NULL) {
        idx = (uint32_t)strtoul(br + 1, NULL, 10);
    }
    while (*p != '\0') {
        while (*p == ' ') { p++; }
        if (strncmp(p, key, klen) == 0 && p[klen] == '=') {
            const char *v = p + klen + 1;
            for (uint32_t part = 0; part < idx; part++) {
                const char *slash = strchr(v, '/');
                const char *sp = strchr(v, ' ');
                if (slash == NULL || (sp != NULL && sp < slash)) {
                    return 0;
                }
                v = slash + 1;
            }
            uint32_t n = 0;
            while (v[n] != '\0' && v[n] != ' ' && (br == NULL || v[n] != '/') && n + 1U < len) {
                out[n] = v[n];
                n++;
            }
            out[n] = '\0';
            return 1;
        }
        while (*p != '\0' && *p != ' ') { p++; }
    }
    return 0;
}

/* Every check of one item against its reply; 1 when all hold. */
static int judge(const char *checks, const char *reply)
{
    char one[96], field[96], op[96], a[96], b[96], got[PORTTOOL_REPLY_MAX];
    const char *p = checks;

    while (*p != '\0') {
        uint32_t n = 0;
        while (*p != '\0' && *p != ';' && n + 1U < sizeof(one)) { one[n++] = *p++; }
        one[n] = '\0';
        if (*p == ';') { p++; }
        if (n == 0U) { continue; }

        /* field:op:a[:b] */
        char *s = one, *c1 = strchr(s, ':'), *c2, *c3;
        if (c1 == NULL) { return 0; }
        *c1 = '\0';
        c2 = strchr(c1 + 1, ':');
        if (c2 == NULL) { return 0; }
        *c2 = '\0';
        c3 = strchr(c2 + 1, ':');
        if (c3 != NULL) { *c3 = '\0'; }
        snprintf(field, sizeof(field), "%s", s);
        snprintf(op, sizeof(op), "%s", c1 + 1);
        snprintf(a, sizeof(a), "%s", c2 + 1);
        snprintf(b, sizeof(b), "%s", c3 != NULL ? c3 + 1 : "");

        if (!field_value(reply, field, got, sizeof(got))) {
            return 0;
        }
        if (strcmp(op, "eq") == 0) {
            if (strcmp(got, a) != 0) { return 0; }
        } else if (strcmp(op, "contains") == 0) {
            if (strstr(got, a) == NULL) { return 0; }
        } else if (strcmp(op, "range") == 0) {
            char *e;
            double v = strtod(got, &e);
            if (e == got || v < strtod(a, NULL) || v > strtod(b, NULL)) { return 0; }
        } else {
            return 0;
        }
    }
    return 1;
}

/* ---- the harness loops ------------------------------------------------------ */

static uint32_t rs232_seq;
static volatile int rs232_back;

void PortAge_LineSeen(const char *line)
{
    char want[16];

    snprintf(want, sizeof(want), "~age%lu", (unsigned long)rs232_seq);
    if (strcmp(line, want) == 0) {
        rs232_back = 1;
    }
}

/* C05 and C06 shorted: a line sent comes straight back. */
static void loop_rs232(char *out, uint32_t len)
{
#if PORTTOOL_HOST_TEST
    snprintf(out, len, "ok=1 sim=1");
#else
    uint32_t t0;

    rs232_seq++;
    rs232_back = 0;
    printf("~age%lu\r\n", (unsigned long)rs232_seq);
    t0 = HAL_GetTick();
    while (!rs232_back && HAL_GetTick() - t0 < 200U) {
        PortTool_PollCommand();
    }
    snprintf(out, len, "ok=%d", rs232_back);
#endif
}

/* DO1-6 through relays 1-6 into DI1-6, DO7 and DO8 straight into DI7, DI8. */
static void loop_dodi(char *out, uint32_t len)
{
#if PORTTOOL_HOST_TEST
    snprintf(out, len, "ok=1 sim=1");
#else
    const uint32_t full = (1UL << PortDout_Res()) - 1UL;

    PortDin_Init();
    Relay_Init();
    if (!PortDout_Init()) {
        snprintf(out, len, "ok=0 err=dout_init");
        return;
    }
    for (int k = 1; k <= PORT_DOUT_COUNT; k++) {
        PortDout_SetFreq(k, 1000U);
        PortDout_SetDuty(k, 0U);
    }
    for (int k = 1; k <= PORT_DOUT_COUNT && k <= PORT_DIN_COUNT; k++) {
        uint8_t on, off;
        if (k <= (int)RELAY_COUNT) { Relay_On((RELAY_Name)(k - 1)); }
        PortDout_SetDuty(k, full);
        HAL_Delay(30U);
        on = PortDin_ReadBits();
        PortDout_SetDuty(k, 0U);
        if (k <= (int)RELAY_COUNT) { Relay_Off((RELAY_Name)(k - 1)); }
        HAL_Delay(30U);
        off = PortDin_ReadBits();
        if (on != (uint8_t)(1U << (k - 1)) || off != 0U) {
            snprintf(out, len, "ok=0 ch=%d on=0x%02X off=0x%02X", k, on, off);
            return;
        }
    }
    snprintf(out, len, "ok=1");
#endif
}

/* AO1 -> AI1 and AO2 -> AI2: both DACs to mv1= and mv2=, read both AIs. */
static void loop_aoai(const char *args, char *out, uint32_t len)
{
#if PORTTOOL_HOST_TEST
    (void)args;
    snprintf(out, len, "ok=1 sim=1");
#else
    uint32_t mv[2] = { 0U, 0U }, raw[2] = { 0U, 0U }, pin;

    if (!PortCmd_GetU32(args, "mv1", &mv[0]) || !PortCmd_GetU32(args, "mv2", &mv[1])) {
        snprintf(out, len, "ok=0 err=args");
        return;
    }
    if (!PortVref_Enable() || !PortAdc_Init() || !PortDac_Init()) {
        snprintf(out, len, "ok=0 err=analog_init");
        return;
    }
    (void)PortDac_SetMv(1, mv[0]);
    (void)PortDac_SetMv(2, mv[1]);
    HAL_Delay(50U);
    for (int ch = 1; ch <= 2; ch++) {
        if (!PortAdc_ReadAin(ch, &raw[ch - 1], &pin)) {
            snprintf(out, len, "ok=0 err=ain%d", ch);
            return;
        }
    }
    snprintf(out, len, "ok=1 raw1=%lu raw2=%lu", (unsigned long)raw[0], (unsigned long)raw[1]);
#endif
}

static void run_item(const age_item_t *it, char *reply, uint32_t len)
{
    if (strcmp(it->target, "loop.rs232") == 0) {
        loop_rs232(reply, len);
    } else if (strcmp(it->target, "loop.dodi") == 0) {
        loop_dodi(reply, len);
    } else if (strncmp(it->target, "session.", 8) == 0) {
        if (!PortTool_SessionSample(it->target + 8, it->args, 1500U, reply, len)
            && reply[0] == '\0') {
            snprintf(reply, len, "ok=0 err=no_frame");
        }
    } else if (strcmp(it->target, "loop.aoai") == 0) {
        loop_aoai(it->args, reply, len);
    } else if (strcmp(it->target, "temp.read") == 0) {
        PortTemp_RunRead(it->args, reply, len);
    } else if (!PortTool_RunCapture(it->target, it->args, reply, len)) {
        snprintf(reply, len, "unknown target");
    }
}

/* ---- LED3 ----------------------------------------------------------------- */

static void led_tick(uint32_t now_ms)
{
    if (running) {
        PortLed_Set((now_ms / 1000U) % 2U == 0U);   /* aging: 1 s on, 1 s off */
    } else if (done && passed) {
        PortLed_Set(1);
    } else if (done) {
        /* N quick pulses for the first failing item's code, then 2 s dark. */
        uint32_t cycle = first_code * 400U + 2000U;
        uint32_t t = now_ms % cycle;
        PortLed_Set(t < first_code * 400U && (t % 400U) < 200U);
    }
}

/* ---- running -------------------------------------------------------------- */

static void finish(void)
{
    age_rec_t r;

    running = 0;
    done = 1;
    passed = fails == 0U;
    memset(&r, 0xFF, sizeof(r));
    r.kind = 'E';
    r.code = (uint8_t)passed;
    r.a = elapsed_ms / 1000U;
    r.b = rounds;
    r.c = fails;
    (void)age_append(&r);
}

static void save_progress(void)
{
    age_rec_t r;

    memset(&r, 0xFF, sizeof(r));
    r.kind = 'P';
    r.a = elapsed_ms / 1000U;
    r.b = rounds;
    r.c = fails;
    (void)age_append(&r);
    saved_ms = elapsed_ms;
}

static void record_fail(uint32_t i, const char *reply)
{
    age_rec_t r;

    fails++;
    if (failed_mask & (1UL << i)) {
        return;   /* only an item's first failure is written */
    }
    failed_mask |= 1UL << i;
    if (first_code == 0U) {
        first_code = items[i].code;
    }
    memset(&r, 0xFF, sizeof(r));
    r.kind = 'F';
    r.code = items[i].code;
    r.item = (uint8_t)i;
    r.a = rounds + 1U;
    r.b = elapsed_ms / 1000U;
    r.c = 0;
    strncpy(r.text, reply, sizeof(r.text));
    (void)age_append(&r);
    memset(&r, 0xFF, sizeof(r));
    r.kind = 'T';
    if (strlen(reply) > sizeof(r.text)) {
        /* The continuation word carries the next 31 characters. */
        uint8_t *t = &r.code;
        size_t n = strnlen(reply + sizeof(r.text), AGE_WORD - 1U);
        memcpy(t, reply + sizeof(r.text), n);
        if (n < AGE_WORD - 1U) { t[n] = 0U; }
    }
    (void)age_append(&r);
}

/* Reads the task header and replays the records; 1 when there is a task. */
static int load(void)
{
    age_head_t head;
    const age_head_t *h = &head;

    running = done = passed = 0;
    elapsed_ms = rounds = fails = failed_mask = first_code = 0;
    next_item = 0;
    n_items = 0;
    rec_off = AGE_REC_OFF;
    if (!age_copy(&head, AGE_BASE, sizeof(head)) || h->magic != AGE_MAGIC
        || h->text_len == 0U || h->text_len > AGE_TEXT_MAX
        || !age_copy(stage, AGE_BASE + AGE_WORD, h->text_len)
        || PortTool_Crc32Update(0, (const uint8_t *)stage, h->text_len) != h->text_crc
        || !load_items(stage, h->text_len)) {
        return 0;
    }
    task_secs = h->secs;
    for (; rec_off <= AGE_SIZE - AGE_WORD; rec_off += AGE_WORD) {
        age_rec_t word;
        const age_rec_t *r = &word;
        if (!age_copy(&word, AGE_BASE + rec_off, AGE_WORD)) {
            continue;   /* torn by a power cut: skipped, the next word is still free */
        }
        switch (r->kind) {
        case 'P':
            elapsed_ms = r->a * 1000U; rounds = r->b; fails = r->c;
            break;
        case 'F':
            if (r->item < AGE_ITEMS) { failed_mask |= 1UL << r->item; }
            if (first_code == 0U) { first_code = r->code; }
            break;
        case 'T':
            break;
        case 'E':
            elapsed_ms = r->a * 1000U; rounds = r->b; fails = r->c;
            done = 1;
            passed = r->code == 1U;
            break;
        default:
            return 1;   /* first erased word: the end of the records */
        }
    }
    return 1;
}

void PortAge_Boot(uint32_t now_ms)
{
    if (!load()) {
        return;
    }
    PortLed_Init();
    saved_ms = elapsed_ms;
    last_ms = now_ms;
    due_ms = now_ms;
    running = !done;
}

int PortAge_Running(void) { return running; }

void PortAge_Pause(void)
{
    if (running) {
        running = 0;
        save_progress();
    }
}

void PortAge_Tick(uint32_t now_ms)
{
    char reply[PORTTOOL_REPLY_MAX];

    if (running) {
        elapsed_ms += now_ms - last_ms;
    }
    last_ms = now_ms;
    if (running && (int32_t)(now_ms - due_ms) >= 0) {
        run_item(&items[next_item], reply, sizeof(reply));
        if (!judge(items[next_item].checks, reply)) {
            record_fail(next_item, reply);
        }
        if (++next_item >= n_items) {
            next_item = 0;
            rounds++;
        }
        now_ms = HAL_GetTick();
        elapsed_ms += now_ms - last_ms;   /* the item's own time counts too */
        last_ms = now_ms;
        due_ms = now_ms + AGE_GAP_MS;
        if (elapsed_ms - saved_ms >= AGE_SAVE_MS) {
            save_progress();
        }
        if (elapsed_ms >= task_secs * 1000U) {
            finish();
        }
    }
    if (running || done) {
        led_tick(now_ms);
    }
}

/* ---- commands --------------------------------------------------------------- */

static void cmd_commit(void)
{
    age_head_t h;
    uint32_t off;
    int ok;

    if (!staging || stage_len == 0U || !load_items(stage, stage_len)) {
        printf("ERR age commit: begin, then add at least one item\r\n");
        return;
    }
    memset(&h, 0xFF, sizeof(h));
    h.magic = AGE_MAGIC;
    h.secs = stage_secs;
    h.text_len = stage_len;
    h.text_crc = PortTool_Crc32Update(0, (const uint8_t *)stage, stage_len);

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);
    ok = age_erase() && age_program(0, &h);
    for (off = 0; ok && off < stage_len; off += AGE_WORD) {
        uint8_t w[AGE_WORD];
        uint32_t n = stage_len - off < AGE_WORD ? stage_len - off : AGE_WORD;
        memset(w, 0xFF, sizeof(w));
        memcpy(w, stage + off, n);
        ok = age_program(AGE_WORD + off, w);
    }
    HAL_FLASH_Lock();
    if (!ok || memcmp((const void *)(AGE_BASE + AGE_WORD), stage, stage_len) != 0) {
        printf("ERR age commit failed: flash err=0x%08lX\r\n", (unsigned long)HAL_FLASH_GetError());
        return;
    }
    staging = 0;
    PortAge_Boot(HAL_GetTick());
    running = 0;   /* starts at the next power-up, away from the PC */
    printf("OK age committed items=%lu secs=%lu\r\n", (unsigned long)n_items,
           (unsigned long)task_secs);
}

static void cmd_clear(void)
{
    int ok;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);
    ok = age_erase();
    HAL_FLASH_Lock();
    running = done = passed = 0;
    n_items = task_secs = 0;
    elapsed_ms = rounds = fails = failed_mask = first_code = 0;
    PortLed_Set(0);
    printf(ok ? "OK age cleared\r\n" : "ERR age clear failed\r\n");
}

static void cmd_read(void)
{
    const char *state = n_items == 0U ? "none" : running ? "run" : !done ? "paused"
                      : passed ? "pass" : "fail";

    printf("OK age state=%s secs=%lu elapsed=%lu rounds=%lu fails=%lu items=%lu code=%lu\r\n",
           state, (unsigned long)task_secs, (unsigned long)(elapsed_ms / 1000U),
           (unsigned long)rounds, (unsigned long)fails, (unsigned long)n_items,
           (unsigned long)first_code);
}

/* The k-th failure record (from 1), with its reading. */
static void cmd_fail(const char *rest)
{
    uint32_t want = 0, k = 0;

    if (!PortCmd_GetU32(rest, "i", &want) || want == 0U) {
        printf("ERR age fail takes i=<n> from 1\r\n");
        return;
    }
    for (uint32_t off = AGE_REC_OFF; off <= AGE_SIZE - 2U * AGE_WORD; off += AGE_WORD) {
        age_rec_t word, next;
        const age_rec_t *r = &word;
        if (!age_copy(&word, AGE_BASE + off, AGE_WORD)) {
            continue;
        }
        if (r->kind == 0xFFU) {
            break;
        }
        if (r->kind != 'F' || ++k != want) {
            continue;
        }
        char got[16 + AGE_WORD];
        const uint8_t *t = (const uint8_t *)&next + 1;
        if (!age_copy(&next, AGE_BASE + off + AGE_WORD, AGE_WORD)) {
            memset(&next, 0xFF, sizeof(next));
        }
        uint32_t n = 0;
        for (uint32_t j = 0; j < sizeof(r->text) && r->text[j] != '\0' && (uint8_t)r->text[j] != 0xFFU; j++) {
            got[n++] = r->text[j];
        }
        if (n == sizeof(r->text)) {
            for (uint32_t j = 0; j < AGE_WORD - 1U && t[j] != 0U && t[j] != 0xFFU; j++) {
                got[n++] = (char)t[j];
            }
        }
        got[n] = '\0';
        for (uint32_t c = 0; c < n; c++) {
            if (got[c] == '\r' || got[c] == '\n') { got[c] = ' '; }
        }
        printf("OK age i=%lu code=%u item=%s round=%lu at=%lu got=%s\r\n", (unsigned long)want,
               r->code, r->item < n_items ? items[r->item].target : "?",
               (unsigned long)r->a, (unsigned long)r->b, got);
        return;
    }
    printf("ERR age fail: there are %lu failure records\r\n", (unsigned long)k);
}

void PortAge_Command(const char *rest)
{
    char verb[16];
    const char *args = NULL;

    PortCmd_Word(rest != NULL ? rest : "", verb, sizeof(verb), &args);
    if (args == NULL) {
        args = "";
    }
    if (strcmp(verb, "begin") == 0) {
        if (!PortCmd_GetU32(args, "secs", &stage_secs) || stage_secs == 0U) {
            printf("ERR age begin takes secs=<seconds>\r\n");
            return;
        }
        staging = 1;
        stage_len = 0;
        printf("OK age begin secs=%lu\r\n", (unsigned long)stage_secs);
    } else if (strcmp(verb, "add") == 0) {
        age_item_t probe;
        uint32_t n = (uint32_t)strlen(args);
        if (!staging || !parse_item(args, &probe) || stage_len + n + 1U > AGE_TEXT_MAX) {
            printf("ERR age add: want begin first, then \"<code> <target> <args>|<checks>\"\r\n");
            return;
        }
        memcpy(stage + stage_len, args, n);
        stage_len += n;
        stage[stage_len++] = '\n';
        printf("OK age add %s\r\n", probe.target);
    } else if (strcmp(verb, "commit") == 0) {
        cmd_commit();
    } else if (strcmp(verb, "run") == 0) {
        if (n_items == 0U || done) {
            printf("ERR age run: no unfinished task\r\n");
            return;
        }
        last_ms = due_ms = HAL_GetTick();
        running = 1;
        PortLed_Init();
        printf("OK age running\r\n");
    } else if (strcmp(verb, "read") == 0) {
        cmd_read();
    } else if (strcmp(verb, "fail") == 0) {
        cmd_fail(args);
    } else if (strcmp(verb, "clear") == 0) {
        cmd_clear();
    } else {
        printf("ERR pt.age takes begin, add, commit, run, read, fail or clear\r\n");
    }
}

#endif /* PORTTOOL_ENABLE */
