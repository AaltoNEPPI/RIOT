/*
 * Copyright (c) 2015, Nordic Semiconductor
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
/**
 * @addtogroup cpu
 * @{
 *
 * @addtogroup nrf52832
 * @{
 *
 * @addtogroup nrf52832-ble Bluetooth Low Energy drivers
 * @{
 *
 * @file
 *
 * @brief   Basic BLE functions.
 *
 * @author  Wojciech Bober <wojciech.bober@nordicsemi.no>
 */
#ifndef BLE_CORE_H
#define BLE_CORE_H

#include <stdint.h>

#include "ble_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * BLE observer priority used in ble-core.c connectivity observer.
 * See @ble_evt_handler() in `ble-core.c`
 *
 * Note that you can (and should) register additional
 * observers, e.g. 6lowpan registers an BLE IPSP observer.
 */
#define BLE_CONNECTION_MEDIUM_BLE_OBSERVER_PRIO 1

/**
 * BLE context, provided by the application
 */
typedef struct ble_context {
    const uint8_t conn_cfg_tag; /**< Connection configuration tag */
    const char *const name;     /**< Advertised device name */
    ble_uuid_t *adv_uuids;      /**< Table of advertised services */
    uint32_t adv_uuid_cnt;      /**< Number of advertised servies */
} ble_context_t;

/**
 * @brief Initialize and enable the BLE stack.
 */
void ble_init(const ble_context_t *p_ble_context);

/**
 * @brief Initialize BLE advertising data.
 *
 * @param name Human readable device name that will be advertised
 */
void ble_advertising_init(const ble_context_t *p_ble_context);

/**
 * @brief Start BLE advertising.
 */
void ble_advertising_start(const ble_context_t *p_ble_context);

/**
 * @brief Return device EUI64 MAC address
 *
 * @param addr pointer to a buffer to store the address
 */
void ble_get_mac(uint8_t addr[8]);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CORE_H */

/**
 * @}
 * @}
 * @}
 */
