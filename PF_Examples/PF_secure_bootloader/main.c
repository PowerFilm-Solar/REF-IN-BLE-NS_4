/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
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
 * @defgroup bootloader_secure_ble main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file for secure DFU.
 *
 */

#include <stdint.h>
#include "boards.h"
#include "nrf_mbr.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_bootloader_dfu_timers.h"
#include "nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nrf_bootloader_info.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "crc32.h"

APP_TIMER_DEF(m_led_interval_id);
APP_TIMER_DEF(m_led_on_id);
APP_TIMER_DEF(m_led_off_id);

static uint8_t m_num_blinks;

static void on_error(void)
{
    NRF_LOG_FINAL_FLUSH();

#if NRF_MODULE_ENABLED(NRF_LOG_BACKEND_RTT)
    // To allow the buffer to be flushed by the host.
    nrf_delay_ms(100);
#endif
#ifdef NRF_DFU_DEBUG_VERSION
    NRF_BREAKPOINT_COND;
#endif
    NVIC_SystemReset();
}


void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    NRF_LOG_ERROR("%s:%d", p_file_name, line_num);
    on_error();
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    NRF_LOG_ERROR("Received a fault! id: 0x%08x, pc: 0x%08x, info: 0x%08x", id, pc, info);
    on_error();
}


void app_error_handler_bare(uint32_t error_code)
{
    NRF_LOG_ERROR("Received an error: 0x%08x!", error_code);
    on_error();
}

static void pf_start_interval(void *p_context){
  bsp_board_led_on(BSP_BOARD_LED_0);
  (void)app_timer_start(m_led_on_id, APP_TIMER_TICKS(1),&m_num_blinks);
}

static void pf_power_off(void *p_context){
  bsp_board_led_off(BSP_BOARD_LED_0);
  if(p_context!=NULL){
    uint8_t num_blinks = (*(uint8_t *)p_context);
    if( num_blinks>1){
    (void)app_timer_start(m_led_off_id, APP_TIMER_TICKS(100),NULL);
    }
  }
}

static void pf_wait_blink(void *p_context){
  bsp_board_led_on(BSP_BOARD_LED_0);
  (void)app_timer_start(m_led_on_id, APP_TIMER_TICKS(1), NULL);
}

static void timers_init(void){
    ret_code_t err_code;
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    m_num_blinks = 1;

    err_code = app_timer_create(&m_led_interval_id,
                                APP_TIMER_MODE_REPEATED,
                                pf_start_interval);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_led_on_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                pf_power_off);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_led_off_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                pf_wait_blink);
    APP_ERROR_CHECK(err_code);
    
    (void)app_timer_start(m_led_interval_id, APP_TIMER_TICKS(3000),NULL);
}
/**
 * @brief Function notifies certain events in DFU process.
 */
static void dfu_observer(nrf_dfu_evt_type_t evt_type)
{
    switch (evt_type)
    {
        case NRF_DFU_EVT_DFU_FAILED:
        case NRF_DFU_EVT_DFU_ABORTED:
        case NRF_DFU_EVT_DFU_INITIALIZED:
            bsp_board_init(BSP_INIT_LEDS);
            m_num_blinks=1;
            //bsp_board_led_on(BSP_BOARD_LED_0);
            //bsp_board_led_on(BSP_BOARD_LED_1);
            //bsp_board_led_off(BSP_BOARD_LED_2);
            break;
        case NRF_DFU_EVT_TRANSPORT_ACTIVATED:
            m_num_blinks=2;
            //bsp_board_led_off(BSP_BOARD_LED_1);
            //bsp_board_led_on(BSP_BOARD_LED_2);
            break;
        case NRF_DFU_EVT_DFU_STARTED:
            break;
        default:
            break;
    }
}


/**@brief Function for application main entry. */
int main(void)
{
    uint32_t ret_val;
    //sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    // Must happen before flash protection is applied, since it edits a protected page.
    NRF_POWER->DCDCEN = 1;
    nrf_bootloader_mbr_addrs_populate();

    // Protect MBR and bootloader code from being overwritten.
    ret_val = nrf_bootloader_flash_protect(0, MBR_SIZE);
    APP_ERROR_CHECK(ret_val);
    ret_val = nrf_bootloader_flash_protect(BOOTLOADER_START_ADDR, BOOTLOADER_SIZE);
    APP_ERROR_CHECK(ret_val);

    (void) NRF_LOG_INIT(nrf_bootloader_dfu_timer_counter_get);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Inside main");
    timers_init();
    ret_val = nrf_bootloader_init(dfu_observer);
    APP_ERROR_CHECK(ret_val);

    NRF_LOG_FLUSH();

    NRF_LOG_ERROR("After main, should never be reached.");
    NRF_LOG_FLUSH();

    APP_ERROR_CHECK_BOOL(false);
}

/**
 * @}
 */
