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
 * The watchdog is clocked by the 32 kHz LF clock, and initialised
 * with a downcounter.  When the counter reaches zero, the watchdog
 * barks and raises the interrupt.  Then, two LF clock cycles or some
 * 60 us later it resets the system.  The system can run some 3500
 * clock cycles between the interrupt and the reset.
 *
 * To prevent the watchdog from barking, it must be silenced before
 * the programmed time period runs out.  It supports up to eight
 * independent silence registers.  All of the configured silence
 * registers must be loaded to reload the watchdog downcounter to its
 * initial value.
 *
 * @}
 */

#include <stdint.h>

/**
 * @brief   Watchdog callback type
 *
 * Called from the watchdog interrupt the interrupt context, with
 * interrupts disabled.  Can run for about 3500 instructions before
 * the watchdog resets the system.  There is no way to prevent the
 * system reset.
 */
typedef void (*watchdog_callback_t)(void);

/**
 * @brief   Silence the watchdog at the given index
 *
 * To silence the dog, this function must be called separately for
 * each index from zero to silencer_count before the watchdog timer
 * runs out.
 *
 * @param [in] idx  Index of the watchdog to feed
 */
void watchdog_silence(int idx);

/**
 * @brief   Initialise the watchdog
 *
 * @param [in] waiting_time_us  The maximum time WD will wait before
                                interrupting, in microseconds
 * @param [in] silencer_count   The number of silencers indices
 *                              configured
 * @param [in] cb               Function called at the WD interrupt.
 */
void watchdog_init(
    uint32_t waiting_time_us, int silencer_count, watchdog_callback_t cb);
