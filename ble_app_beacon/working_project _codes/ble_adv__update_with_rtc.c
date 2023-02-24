/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Applicaticon main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(10240, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x0E                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x0C                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0xA154                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define TX_TIME_STAMP                   0x00, 0x00, 0x00, 0x00, \
                                        0x00, 0x00                         //< Tx Timestamp value. 
#define PACKET_COUNT                    0x00, 0x00, 0x00, 0x01
#define DEVICE_ID                       0x01
#define ADV_UPDATE_INTERVAL             APP_TIMER_TICKS(10000)
#define TS_UPDATE_INTERVAL              APP_TIMER_TICKS(1)  

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define COMPARE_COUNTERTIME  (3UL)                                        /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */

APP_TIMER_DEF(m_adv_update_timer_id); 
// APP_TIMER_DEF(time_stamp_update_id);
static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[2][BLE_GAP_ADV_SET_DATA_SIZE_MAX];      /**< Buffer for storing an encoded advertising set. */
uint32_t                    err_code;
ble_advdata_t               advdata;
static uint8_t              flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED; 
static uint64_t             tx_time_stamp = 0;                             // Variable to store time stamp of transmission
static uint16_t             packet_count = 0;                              // Variable to store packet count
static uint64_t             diff_time = 0;
static uint64_t             prev_time = 0;
static uint64_t             timer_offset = 1;

ble_advdata_manuf_data_t manuf_specific_data;

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata[0],
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};


static uint8_t m_beacon_info[APP_ADV_DATA_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    DEVICE_ID,
    PACKET_COUNT,
    TX_TIME_STAMP
};


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    
    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_ADV_DATA_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_SHORT_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = 0;       // Never time out.

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}

static void adv_update_timeout_handler(void * p_context)
{
   
    UNUSED_PARAMETER(p_context);

    uint32_t err_code;
    m_adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;

    /* Increment the Beacon's Major value as a test to demonstrate that we can update payload
       while advertising */
    // major_value++;
    static uint64_t current_time = 0;
    // int second_equalizer = 1000;
    // double curr_time = 0;
    uint64_t drift = 0;
    packet_count++;
    // char curr_time_string[15];
    current_time = tx_time_stamp + (70 * packet_count);
    diff_time = current_time - prev_time;
    prev_time = current_time;
    drift = (packet_count*10000)-current_time;
    // curr_time = (double)current_time/(double)second_equalizer;
    // sprintf(curr_time_string, "%f", curr_time);
    // tx_time_stamp = tx_time_stamp + 1000;
    m_beacon_info[2]  = (packet_count >> 24) & 0x000000FF;
    m_beacon_info[3]  = (packet_count >> 16) & 0x000000FF;
    m_beacon_info[4]  = (packet_count >> 8) & 0x000000FF;
    m_beacon_info[5]  =  packet_count & 0x000000FF;
    m_beacon_info[6]  = (current_time >> 48) & 0x0000000000FF;
    m_beacon_info[7]  = (current_time >> 32) & 0x0000000000FF;
    m_beacon_info[8]  = (current_time >> 24) & 0x0000000000FF;
    m_beacon_info[9]  = (current_time >> 16) & 0x0000000000FF;
    m_beacon_info[10] = (current_time >> 8) & 0x0000000000FF;
    m_beacon_info[11] =  current_time & 0x0000000000FF;

    /* Swap adv data buffer - from API doc: "In order to update advertising
    data while advertising,new advertising buffers must be provided" */
    m_adv_data.adv_data.p_data =
        (m_adv_data.adv_data.p_data == m_enc_advdata[0]) ? m_enc_advdata[1] : m_enc_advdata[0];


    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);
    
    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, NULL);
    APP_ERROR_CHECK(err_code);
    
    // NRF_LOG_INFO("Time value: 0x%x ms", tx_time_stamp);
    NRF_LOG_INFO("Expected Time value: %d ms", (packet_count*10000));
    NRF_LOG_INFO("Observed Time value: %d ms", current_time);
    NRF_LOG_INFO("Drift: %d ms", drift);
    NRF_LOG_INFO("Function Call Interval: %d ms", diff_time);
    // NRF_LOG_INFO("Time value in seconds: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(curr_time));
    NRF_LOG_INFO("Packet Count: 0x%x or %d", packet_count, packet_count);
}

//static void time_stamp_update_handler(void * p_context)
//{
   
//    UNUSED_PARAMETER(p_context);

//    uint32_t err_code;
//    tx_time_stamp = tx_time_stamp + 1;

//}

/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.*/
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        tx_time_stamp = tx_time_stamp + 1;
        // NRF_LOG_INFO("Timer Increment");
    }
}


/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_init(void)
{
    uint32_t err_code;
    const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */


    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 32;    //for 1ms tick
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc,true);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
    err_code = nrf_drv_rtc_cc_set(&rtc,0,COMPARE_COUNTERTIME * 8,true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}

/**@brief Function for starting application timers. */
static void application_timers_start(void)
{
    ret_code_t err_code;
    // Start application timers.
    err_code = app_timer_start(m_adv_update_timer_id, ADV_UPDATE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    // err_code1 = app_timer_start(time_stamp_update_id, TS_UPDATE_INTERVAL, NULL);
    // APP_ERROR_CHECK(err_code);
    
}

/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_adv_update_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                adv_update_timeout_handler);
    APP_ERROR_CHECK(err_code);
    //err_code = app_timer_create(&time_stamp_update_id, 
    //                            APP_TIMER_MODE_REPEATED,
    //                            time_stamp_update_handler);
    //APP_ERROR_CHECK(err_code);
}



/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    timers_init();  // for app_timer 
    leds_init();
    power_management_init();
    lfclk_init();
    rtc_init();
    ble_stack_init();
    advertising_init();

    // Start execution.
    NRF_LOG_INFO("Beacon example started.");
    advertising_start();
    application_timers_start();

    // Enter main loop.
     while(1)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
