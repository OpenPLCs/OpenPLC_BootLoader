// port_lse.c
//
// LSE drive capability - see port_lse.h.

#include "port_lse.h"
#include "main.h"

static const uint32_t lse_hal_drv[4] = {
    RCC_LSEDRIVE_LOW, RCC_LSEDRIVE_MEDIUMLOW, RCC_LSEDRIVE_MEDIUMHIGH, RCC_LSEDRIVE_HIGH,
};

int PortLse_Restart(int drv, uint32_t *start_ms)
{
    uint32_t start;

    *start_ms = 0U;
    if (drv < PORT_LSE_LOW || drv > PORT_LSE_HIGH) {
        return 0;
    }

    HAL_PWR_EnableBkUpAccess();

    /* LSEDRV is ignored while LSE runs. */
    CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);
    start = HAL_GetTick();
    while ((RCC->BDCR & RCC_BDCR_LSERDY) != 0U) {
        if ((HAL_GetTick() - start) > PORT_LSE_START_TIMEOUT_MS) {
            return 0;
        }
    }

    /* The HAL macro applies the rev Y encoding. */
    __HAL_RCC_LSEDRIVE_CONFIG(lse_hal_drv[drv]);
    SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);

    start = HAL_GetTick();
    while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0U) {
        if ((HAL_GetTick() - start) > PORT_LSE_START_TIMEOUT_MS) {
            *start_ms = PORT_LSE_START_TIMEOUT_MS;
            return 0;
        }
    }
    *start_ms = HAL_GetTick() - start;
    return 1;
}

int PortLse_Drive(void)
{
    uint32_t v = RCC->BDCR & RCC_BDCR_LSEDRV;

    /* Undo the swap __HAL_RCC_LSEDRIVE_CONFIG writes on rev Y. */
    if ((HAL_GetREVID() <= REV_ID_Y) &&
        ((v == RCC_LSEDRIVE_MEDIUMLOW) || (v == RCC_LSEDRIVE_MEDIUMHIGH))) {
        v = (~v) & RCC_BDCR_LSEDRV_Msk;
    }
    for (int i = PORT_LSE_LOW; i <= PORT_LSE_HIGH; i++) {
        if (v == lse_hal_drv[i]) {
            return i;
        }
    }
    return PORT_LSE_LOW;
}

int PortLse_Ready(void)
{
    return (RCC->BDCR & RCC_BDCR_LSERDY) != 0U;
}

/* Input capture prescaler: one capture every 8 LSE periods keeps the gap
 * (244 us) under the 16-bit counter's wrap at a 200 MHz timer clock. */
#define LSE_PER_CAPTURE 8U
#define LSE_CAPTURE_TIMEOUT_MS 5U
#define LSE_MEASURE_TRIES 3

static uint32_t tim16_clock_hz(void)
{
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

    /* TIMPRE = 0: timers run at twice an APB clock that is divided. */
    return ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) == RCC_APB2_DIV1) ? pclk2 : 2U * pclk2;
}

static int tim16_wait_capture(uint16_t *ccr)
{
    uint32_t start = HAL_GetTick();

    while ((TIM16->SR & TIM_SR_CC1IF) == 0U) {
        if ((HAL_GetTick() - start) > LSE_CAPTURE_TIMEOUT_MS) {
            return 0;
        }
    }
    if ((TIM16->SR & TIM_SR_CC1OF) != 0U) {
        return 0; /* a capture was overwritten before it was read */
    }
    *ccr = (uint16_t)TIM16->CCR1; /* reading clears CC1IF */
    return 1;
}

static int lse_measure_once(uint32_t window_ms, PortLseFreq *f)
{
    uint16_t prev, cur;
    uint32_t start;
    int ok = 0;

    TIM16->CR1 = 0U;
    TIM16->PSC = 0U;
    TIM16->ARR = 0xFFFFU;
    TIM16->TISEL = TIM_TIM16_TI1_RCC_LSE;
    TIM16->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1PSC; /* IC1 on TI1, every 8th edge */
    TIM16->CCER = TIM_CCER_CC1E;                         /* rising edge */
    TIM16->EGR = TIM_EGR_UG;
    TIM16->SR = 0U;
    TIM16->CR1 = TIM_CR1_CEN;

    f->ticks = 0U;
    f->periods = 0U;
    if (tim16_wait_capture(&prev)) {
        start = HAL_GetTick();
        ok = 1;
        while ((HAL_GetTick() - start) < window_ms) {
            if (!tim16_wait_capture(&cur)) {
                ok = 0;
                break;
            }
            f->ticks += (uint16_t)(cur - prev);
            f->periods += LSE_PER_CAPTURE;
            prev = cur;
        }
        f->ms = HAL_GetTick() - start;
    }

    TIM16->CR1 = 0U;
    TIM16->CCER = 0U;
    return ok && f->periods > 0U;
}

int PortLse_MeasureFreq(uint32_t window_ms, PortLseFreq *f)
{
    int ok = 0;

    if (!PortLse_Ready()) {
        return 0;
    }
    __HAL_RCC_TIM16_CLK_ENABLE();
    f->tclk_hz = tim16_clock_hz();
    for (int i = 0; i < LSE_MEASURE_TRIES && !ok; i++) {
        ok = lse_measure_once(window_ms, f);
    }
    __HAL_RCC_TIM16_CLK_DISABLE();
    return ok ? 1 : -1;
}
