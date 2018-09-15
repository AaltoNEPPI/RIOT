/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_cortexm_common
 * @{
 *
 * @file
 * @brief       Cortex-M specific configuration and initialization options
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

/*
 * When compiling this file for nRF52 with the SoftDevice,
 * do not override NVIC_SetPriority as we do elsewhere, since
 * here we want to set also PendSV_IRQn and SVCall_IRQn.
 *
 * Not overriding is safe here, since the SoftDevice will be
 * initialised only later in cpu.c
 */
#if defined(CPU_NRF52) && defined(SOFTDEVICE_PRESENT) && defined(NRF_SDH_ENABLED)
#define DONT_OVERRIDE_NVIC 1
#endif

#include "cpu.h"

/**
 * Interrupt vector base address, defined by the linker
 */
extern const void *_isr_vectors;

#if defined(CPU_CORTEXM_INIT_SUBFUNCTIONS)
#define CORTEXM_STATIC_INLINE /*empty*/
#else
#define CORTEXM_STATIC_INLINE static inline
#endif

CORTEXM_STATIC_INLINE void cortexm_init_isr_priorities(void)
{
    /* initialize the interrupt priorities */
    /* set pendSV interrupt to same priority as the rest */
    NVIC_SetPriority(PendSV_IRQn, CPU_DEFAULT_IRQ_PRIO);
    /* set SVC interrupt to same priority as the rest */
    NVIC_SetPriority(SVCall_IRQn, CPU_DEFAULT_IRQ_PRIO);
    /* initialize all vendor specific interrupts with the same value */
    for (unsigned i = 0; i < CPU_IRQ_NUMOF; i++) {
        NVIC_SetPriority((IRQn_Type) i, CPU_DEFAULT_IRQ_PRIO);
    }
}

CORTEXM_STATIC_INLINE void cortexm_init_misc(void)
{
    /* enable wake up on events for __WFE CPU sleep */
    SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

    /* for Cortex-M3 r1p0 and up the STKALIGN option was added, but not automatically
     * enabled until revision r2p0. For 64bit function arguments to work properly this
     * needs to be enabled.
     */
#ifdef SCB_CCR_STKALIGN_Msk
    SCB->CCR |= SCB_CCR_STKALIGN_Msk;
#endif
}

void cortexm_init(void)
{
    cortexm_init_fpu();

    /* configure the vector table location to internal flash */
#if defined(CPU_ARCH_CORTEX_M3) || defined(CPU_ARCH_CORTEX_M4) || \
    defined(CPU_ARCH_CORTEX_M4F) || defined(CPU_ARCH_CORTEX_M7) || \
    (defined(CPU_ARCH_CORTEX_M0PLUS) && (__VTOR_PRESENT == 1))
    SCB->VTOR = (uint32_t)&_isr_vectors;
#endif

    cortexm_init_isr_priorities();
    cortexm_init_misc();
}
