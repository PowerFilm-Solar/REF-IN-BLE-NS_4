/**
 * Copyright (c) 2013 - 2020, Nordic Semiconductor ASA
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
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(CRC32)
#include "crc32.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_clock.h"
#include "nrf_rtc.h"
#include <stdlib.h>

#define RTC_INSTANCE   2
#define RTC_STRUCT     NRF_RTC2
#define RTC_IRQHandler RTC2_IRQHandler
#define RTC_IRQn       RTC2_IRQn
#define RTC_CC_COUNT   NRF_RTC_CC_CHANNEL_COUNT(2))

#define RTC_PRESCALER       (0)              //!< The value provided to the RTC as the prescaler. 0 corresponds to one tick per clock cycle of the LFCLK (32768 ticks/s).
#define RTC_WRAP_TICKS      ((1 << 24) - 1)  //!< The largest possible value in the RTC counter register.
#define MAX_TIMEOUT_TICKS   (RTC_WRAP_TICKS) //!< The longest fire timeout allowed. Longer timeouts are handled by multiple firings.


APP_TIMER_DEF(m_crc_timer);

static uint32_t crc;
static uint32_t size_remaining;
static uint8_t const *p_data_g;
static uint32_t i;

static void PF_crc32_timer_handler(void *p_context){

  for (  uint32_t current_i = i; size_remaining && (i < current_i+1024); i++)
    {
        crc = crc ^ p_data_g[i];
        for (uint32_t j = 8; j > 0; j--)
        {
            crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
        }
        size_remaining--;
    }
    if(size_remaining){
      app_timer_start(m_crc_timer,APP_TIMER_TICKS(1500),NULL);
    }
  return;
}

uint32_t crc32_compute(uint8_t const * p_data, uint32_t size, uint32_t const * p_crc)
{
#if NRF_MODULE_ENABLED(PF_CRC_DELAY)
    static uint8_t first_run = 1;
    if(first_run){
      if (!nrf_clock_lf_is_running())
        {
            nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);
        }
        nrf_rtc_event_clear(RTC_STRUCT, NRF_RTC_EVENT_TICK);
        nrf_rtc_event_clear(RTC_STRUCT, NRF_RTC_EVENT_COMPARE_0);
        nrf_rtc_event_clear(RTC_STRUCT, NRF_RTC_EVENT_COMPARE_1);
        NRFX_IRQ_PRIORITY_SET(RTC_IRQn, 5);
        NRFX_IRQ_ENABLE(RTC_IRQn);
        nrf_rtc_prescaler_set(RTC_STRUCT, RTC_PRESCALER);
        nrf_rtc_task_trigger(RTC_STRUCT, NRF_RTC_TASK_CLEAR);
        nrf_rtc_task_trigger(RTC_STRUCT, NRF_RTC_TASK_START);
        nrf_rtc_int_enable(RTC_STRUCT, RTC_INTENSET_OVRFLW_Msk);
      
      uint32_t err = app_timer_init();
      APP_ERROR_CHECK(err);
      err = nrf_pwr_mgmt_init();
      APP_ERROR_CHECK(err);
      app_timer_create( &m_crc_timer, APP_TIMER_MODE_SINGLE_SHOT, PF_crc32_timer_handler);
      first_run = 0;
    }
    crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
    size_remaining = size;
    p_data_g = p_data;
    //TODO check if a certain interval has passed before trying to check this shit again.
    for (i = 0; (i < size) && (i < 2048); i++)
    {
        crc = crc ^ p_data[i];
        for (uint32_t j = 8; j > 0; j--)
        {
            crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
        }
        size_remaining--;
    }
    if(size_remaining){
      uint32_t ret_code_t = app_timer_start(m_crc_timer, APP_TIMER_TICKS(400), NULL);
      APP_ERROR_CHECK(ret_code_t);
      while(size_remaining){
        nrf_pwr_mgmt_run();
      }
    }
    return ~crc;
#else
    uint32_t crc;

    crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
    for (uint32_t i = 0; i < size; i++)
    {
        crc = crc ^ p_data[i];
        for (uint32_t j = 8; j > 0; j--)
        {
            crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
        }
    }
    return ~crc;
#endif
}
#endif //NRF_MODULE_ENABLED(CRC32)
