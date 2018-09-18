/*
 * Copyright (C) 2018 Aalto University
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_nrf52
 * @{
 *
 * @file
 * @brief       nRF52 Watchdog implementation
 *
 * @author      Pekka Nikander <pekka.nikander@iki.fi>
 *
 * @}
 */

#define ENABLE_DEBUG (1)
#include "debug.h"

#include "cpu.h"
#include "div.h"
#include "nrf52.h"
#include "watchdog.h"
#if !defined(CPU_WDT_IRQ_PRIO)
#define CPU_WDT_IRQ_PRIO (CPU_DEFAULT_IRQ_PRIO-1)
#endif

static watchdog_callback_t callback;

void isr_wdt(void)
{
    if (callback)
        callback();
#ifdef DEVELHELP
    /* The bkpt instruction will signal to the debugger to break here. */
    __asm__("bkpt #0");
#endif
    puts("WD interrupt");
    // Just wait for the reset
    for (;;)
	;
}

void watchdog_silence(int idx)
{
    NRF_WDT->RR[idx] = WDT_RR_RR_Reload << WDT_RR_RR_Pos;
}

void watchdog_init(uint32_t barking_time_us, int silencer_count, watchdog_callback_t cb)
{
    callback = cb;

    /* Enable the wathdog interrupt */
    NVIC_SetPriority(WDT_IRQn, CPU_WDT_IRQ_PRIO);
    NVIC_EnableIRQ(WDT_IRQn);
    NRF_WDT->INTENSET = WDT_INTENSET_TIMEOUT_Enabled << WDT_INTENSET_TIMEOUT_Pos;

    /* Initialise the counter value, measured in LF clock cycles */
    const uint32_t barking_time_lf = div_u32_by_15625div512(barking_time_us);
    NRF_WDT->CRV = (barking_time_lf << WDT_CRV_CRV_Pos) & WDT_CRV_CRV_Msk;

    /* Initialise the reload request mask */
    const uint32_t silencers =
        ~ ((((~WDT_RREN_RR0_Enabled) << WDT_RREN_RR0_Pos) /* 0xFFFFFFFE */
            << (silencer_count-1))        /* e.g. 0xFFFFFFF0 for 4 */
          );
    NRF_WDT->RREN = silencers;

    /* TBD: Enable the default configuration to be changed */

    /* Start the watchdog */
    NRF_WDT->TASKS_START = 1;
}
