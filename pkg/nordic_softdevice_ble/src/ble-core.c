/*
 * Copyright (c) 2016, Nordic Semiconductor
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
 *         Basic BLE functions.
 * @author
 *         Wojciech Bober <wojciech.bober@nordicsemi.no>
 *         Kaspar Schleiser <kaspar@schleiser.de>
 *
 */
#include <stdbool.h>
#include <stdint.h>

#include "nrf_sdm.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_nvic.h"

#include "app_error.h"
#include "ble_advdata.h"
#include "iot_defines.h"

#include "ble-core.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

/**
 * Time for which the device must be advertising in non-connectable
 * mode (in seconds). BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED (0)
 * disables timeout.
 */
#define APP_ADV_DURATION BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED

/**
 * Minimum acceptable connection interval (0.5 seconds).
 */
#define GAP_MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS)

/**
 * Maximum acceptable connection interval (1 second).
 */
#define GAP_MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS)

/**
 * Slave latency.
 */
#define GAP_SLAVE_LATENCY     0

/**
 * Connection supervisory time-out (4 seconds).
 */
#define GAP_CONN_SUP_TIMEOUT  MSEC_TO_UNITS(4000, UNIT_10_MS)

/**
 * Advertising handle used to identify an advertising set.
 */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;

static void
ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);

/*---------------------------------------------------------------------------*/

/**
 * Pointer to the BLE context passed by the upper layer to `ble_init`.
 *
 * `NRF_SDH_BLE_OBSERVER` macro expands to a compile-time assignment
 * of the event handler function and the context argument to a
 * structure, which is placed in a specific memory segment scanned by
 * the SD whenever there is an event to dispatch.  Since RIOT does not
 * currently support similar arrangement but typically relies on
 * context parameters passed by the application, we use an extra layer
 * of indirection here.
 */
/*
 * TODO: Should not need this kind of arrangement.
 */
const ble_context_t *m_p_ble_context;

// Register with the SoftDevice handler module for BLE events.
NRF_SDH_BLE_OBSERVER(m_ble_observer, BLE_CONNECTION_MEDIUM_BLE_OBSERVER_PRIO,
                     ble_evt_handler, &m_p_ble_context);

/**
 * @brief Initialize and enable the BLE stack.
 */
void
ble_init(const ble_context_t *p_ble_context)
{
  uint32_t err_code;
  uint32_t app_ram_start = 0;

  m_p_ble_context = p_ble_context;

  // Add configuration the BLE stack using the default settings for 6LowPan
  // Fetch the start address of the application RAM.
  err_code = nrf_sdh_ble_default_cfg_set(p_ble_context->conn_cfg_tag, &app_ram_start);
  APP_ERROR_CHECK(err_code);
  DEBUG("ble_stack_init: app_ram_start[%d]=%lx\n", p_ble_context->conn_cfg_tag, app_ram_start);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&app_ram_start); // Issues warnings/debug about RAM start
  if (NRF_ERROR_NO_MEM == err_code) {
    core_panic(PANIC_GENERAL_ERROR, "ble-core: BLE initialisation failed: SoftDevice RAM too small");
  }
  APP_ERROR_CHECK(err_code);

  // Setup address
  ble_gap_addr_t ble_addr;
  err_code = sd_ble_gap_addr_get(&ble_addr);
  APP_ERROR_CHECK(err_code);

  ble_addr.addr[5] = 0x00;
  ble_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;

  err_code = sd_ble_gap_addr_set(&ble_addr);
  APP_ERROR_CHECK(err_code);

  DEBUG("ble-core: GAP address: %02x:%02x:%02x:%02x:%02x:%02x\n",
        ble_addr.addr[5],
        ble_addr.addr[4],
        ble_addr.addr[3],
        ble_addr.addr[2],
        ble_addr.addr[1],
        ble_addr.addr[0]);
}

/*---------------------------------------------------------------------------*/

/**
 * @brief Return device EUI64 MAC address
 * @param addr pointer to a buffer to store the address
 */
void
ble_get_mac(uint8_t addr[8])
{
  uint32_t err_code;
  ble_gap_addr_t ble_addr;

  err_code = sd_ble_gap_addr_get(&ble_addr);
  APP_ERROR_CHECK(err_code);

  IPV6_EUI64_CREATE_FROM_EUI48(addr, ble_addr.addr, ble_addr.addr_type);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief Initialize BLE advertising data.
 * @param name Human readable device name that will be advertised
 */
void
ble_advertising_init(const ble_context_t *p_ble_context)
{
  const char *const name = p_ble_context->name;

  /**
   * Buffers for storing an encoded advertising set.
   * The ble_gap_adv_data_t buffer must be statically allocated for the SoftDevice to use.
   * See nRF5_SDK_15.0.0_a53641a/components/softdevice/s132/headers/ble_gap.h#L836
   */
  static uint8_t enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
  static ble_gap_adv_data_t adv_data = {
    .adv_data =      { .p_data = enc_advdata, .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX },
    .scan_rsp_data = { .p_data = NULL,        .len    = 0                             },
  };

  uint32_t err_code;
  ble_advdata_t advdata;
  ble_gap_adv_params_t adv_params;
  const uint8_t flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)name, strlen(name));
  APP_ERROR_CHECK(err_code);

  ble_gap_conn_params_t gap_conn_params;
  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = GAP_MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = GAP_MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = GAP_SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = GAP_CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);

  // Build and set advertising data.
  memset(&advdata, 0, sizeof(advdata));

  advdata.name_type = BLE_ADVDATA_FULL_NAME;
  advdata.flags = flags;
  advdata.uuids_complete.uuid_cnt = p_ble_context->adv_uuid_cnt;
  advdata.uuids_complete.p_uuids = p_ble_context->adv_uuids;

  err_code = ble_advdata_encode(&advdata, adv_data.adv_data.p_data, &adv_data.adv_data.len);
  APP_ERROR_CHECK(err_code);

  // Initialize advertising parameters (used when starting advertising).
  memset(&adv_params, 0, sizeof(adv_params));

  adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
  adv_params.duration        = APP_ADV_DURATION;
  adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
  adv_params.p_peer_addr     = NULL; // Undirected advertisement.
  adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
  adv_params.interval        = p_ble_context->app_adv_interval;

  err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &adv_data, &adv_params);
  APP_ERROR_CHECK(err_code);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief Start BLE advertising.
 */
void
ble_advertising_start(const ble_context_t *p_ble_context)
{
  uint32_t err_code;

  err_code = sd_ble_gap_adv_start(m_adv_handle, p_ble_context->conn_cfg_tag);
  APP_ERROR_CHECK(err_code);

  DEBUG("ble-core: advertising started\n");
}
/*---------------------------------------------------------------------------*/
/**
 * @brief Print GAP address.
 * @param addr a pointer to address
 */
void
ble_gap_addr_print(const ble_gap_addr_t *addr)
{
    unsigned int i;
    for(i = 0; i < sizeof(addr->addr); i++) {
        if (i > 0) {
            DEBUG(":");
        }
        DEBUG("%02x", addr->addr[i]);
    }
    DEBUG(" (%d)", addr->addr_type);
}

/**
 * @brief SoftDevice BLE event callback.
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void
ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
  static const ble_gap_phys_t phys = {
    .rx_phys = BLE_GAP_PHY_AUTO,
    .tx_phys = BLE_GAP_PHY_AUTO,
  };

  const ble_context_t *p_ble_context = *(ble_context_t **)p_context;
  ret_code_t err_code;

  switch(p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    DEBUG("ble-core: connected [handle:%d, peer: ", p_ble_evt->evt.gap_evt.conn_handle);
    ble_gap_addr_print(&(p_ble_evt->evt.gap_evt.params.connected.peer_addr));
    DEBUG("]\n");
    sd_ble_gap_rssi_start(p_ble_evt->evt.gap_evt.conn_handle,
                          BLE_GAP_RSSI_THRESHOLD_INVALID,
                          0);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    DEBUG("ble-core: disconnected [handle:%d]\n", p_ble_evt->evt.gap_evt.conn_handle);
    ble_advertising_start(p_ble_context);
    break;

  case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    DEBUG("ble-core: sec_param_request [handle:%d]\n", p_ble_evt->evt.gap_evt.conn_handle);
    err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
                                           BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                           NULL,
                                           NULL);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    DEBUG("ble-core: PHY update request\n");
    err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
    break;

  default:
    break;
  }
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
