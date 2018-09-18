/*
 * Copyright (C) 2017 HAW Hamburg
 *               2017 Freie Universität Berlin
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
 * @brief       Low-level ADC driver implementation
 *
 * @author      Dimitri Nahm <dimitri.nahm@haw-hamburg.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#define ENABLE_DEBUG (1)
#include "debug.h"

#include "cpu.h"
#include "mutex.h"
#include "periph/adc.h"
#include "periph_conf.h"

#define FOO

/**
 * @name    Default ADC reference, gain configuration and acquisition time
 *
 * Can be overridden by the board configuration if needed. The default
 * configuration uses the full VDD (typically 3V3) as reference and samples for
 * 10us.
 * @{
 */
#ifndef ADC_REF
#define ADC_REF             SAADC_CH_CONFIG_REFSEL_VDD1_4
#endif
#ifndef ADC_GAIN
#define ADC_GAIN            SAADC_CH_CONFIG_GAIN_Gain1_4
#endif
#ifndef ADC_TACQ
#define ADC_TACQ            SAADC_CH_CONFIG_TACQ_10us
#endif
#ifndef ADC_BURST
#define ADC_BURST           SAADC_CH_CONFIG_BURST_Enabled
#endif
#ifndef ADC_OVERSAMPLE
#define ADC_OVERSAMPLE      SAADC_OVERSAMPLE_OVERSAMPLE_Over256x
#endif
/** @} */

/**
 * @brief   Lock to prevent concurrency issues when used from different threads
 */
static mutex_t lock = MUTEX_INIT;

#ifdef FOO
/**
 * @brief   Lock to wait for an interrupt
 */
static mutex_t isr_lock = MUTEX_INIT_LOCKED;
#endif

/**
 * @brief   We use a static result buffer so we do not have to reprogram the
 *          result pointer register
 */
static int16_t result;

static inline void prep(void)
{
    mutex_lock(&lock);
    NRF_SAADC->ENABLE = 1;
}

static inline void done(void)
{
    NRF_SAADC->ENABLE = 0;
    mutex_unlock(&lock);
}

int adc_init(adc_t line)
{
    if (line >= ADC_NUMOF) {
        return -1;
    }

    prep();

    // NVIC_SetPriority(SAADC_IRQn, 6 /*XXX*/);
    NVIC_EnableIRQ(SAADC_IRQn);

    /* prevent multiple initialization by checking the result ptr register */
    if (NRF_SAADC->RESULT.PTR != (uint32_t)&result) {
        /* set data pointer and the single channel we want to convert */
        NRF_SAADC->RESULT.MAXCNT = 1;
        NRF_SAADC->RESULT.PTR = (uint32_t)&result;

        /* configure the first channel (the only one we use):
         * - bypass resistor ladder+
         * - acquisition time as defined by board (or 10us as default)
         * - reference and gain as defined by board (or VDD as default)
         * - allow burst mode oversampling
         */
        NRF_SAADC->CH[0].CONFIG =
            0
            | (ADC_GAIN  << SAADC_CH_CONFIG_GAIN_Pos)
            | (ADC_REF   << SAADC_CH_CONFIG_REFSEL_Pos)
            | (ADC_TACQ  << SAADC_CH_CONFIG_TACQ_Pos)
            | (ADC_BURST << SAADC_CH_CONFIG_BURST_Pos)
            ;
        NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;

        /* calibrate SAADC */
        NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
        NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
        while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0) {}
    }

    done();

    return 0;
}

#ifdef FOO
/**
 * XXX
 */
void isr_saadc(void) {
    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->INTENCLR = SAADC_INTEN_END_Msk;
    /* Wake up adc_sample */
    mutex_unlock(&isr_lock);
    cortexm_isr_end();
}
#endif

static uint8_t res2oversample[] = {
    SAADC_OVERSAMPLE_OVERSAMPLE_Bypass,   //<  8 bit resolution
    SAADC_OVERSAMPLE_OVERSAMPLE_Bypass,   //< 10 bit resolution
    SAADC_OVERSAMPLE_OVERSAMPLE_Bypass,   //< 12 bit resolution
    ADC_OVERSAMPLE,                       //< 14 bit resolution, oversampling
};

int adc_sample(adc_t line, adc_res_t res)
{
    assert(line < ADC_NUMOF);

    /* check if resolution is valid */
    if (res > ADC_RES_14BIT) {
        return -1;
    }

    assert(res < sizeof(res2oversample)/sizeof(res2oversample[0]));

    /* prepare device */
    prep();

    /* set oversample */
    NRF_SAADC->OVERSAMPLE = res2oversample[res];
    /* set resolution */
    NRF_SAADC->RESOLUTION = res;
    /* set line to sample */
    NRF_SAADC->CH[0].PSELP = (line + 1);

    /* start the SAADC and wait for the started event */
    NRF_SAADC->EVENTS_STARTED = 0;
    NRF_SAADC->TASKS_START = 1;
    while (NRF_SAADC->EVENTS_STARTED == 0) {}

#ifdef FOO
    /* prevent interrupts from taking place early */
    const uint32_t irqs = irq_disable();

    /* enable EVENTS_END interrupt */
    NRF_SAADC->INTENSET = SAADC_INTEN_END_Msk; /* Disabled at the handler */
#endif

    /* trigger the actual conversion */
    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->TASKS_SAMPLE = 1;

#ifdef FOO
    /* Wait for the sample to complete */
    irq_restore(irqs);
    // NOTE: There is a potential conflict window here.  FIXME.
    mutex_lock(&isr_lock);
    // We get here only when the ISR has unlocked the mutex.
    
    // Try to relock the mutex, making sure it won't get released
    if (mutex_trylock(&isr_lock)) {
        core_panic(PANIC_ASSERT_FAIL, "spurious interrupt: nRF52 ADC isr lock released");
    }
#else
    while (NRF_SAADC->EVENTS_END == 0) {
        // thread_yield_higher();
    }
#endif

    /* stop the SAADC */
    NRF_SAADC->EVENTS_STOPPED = 0;
    NRF_SAADC->TASKS_STOP = 1;
    while (NRF_SAADC->EVENTS_STOPPED == 0) {}

    /* free device */
    done();

    /* hack -> the result can be a small negative number when a AINx pin is
     * connected via jumper wire a the board's GND pin. There seems to be a
     * slight difference between the internal CPU GND and the board's GND
     * voltage levels?! (observed on nrf52dk and nrf52840dk) */
    return (result < 0) ? 0 : (int)result;
}
