/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include "RCInput.h"
#include "hal.h"
#include "hwdef/common/ppm.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#if HAL_WITH_IO_MCU
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

#include <AP_Math/AP_Math.h>

#ifndef HAL_NO_UARTDRIVER
#include <GCS_MAVLink/GCS.h>
#endif

#define SIG_DETECT_TIMEOUT_US 500000
using namespace ChibiOS;
extern const AP_HAL::HAL& hal;
void RCInput::init()
{
#ifndef HAL_BUILD_AP_PERIPH
    AP::RC().init();
#endif

//这里两个宏定义，是用来选择执行FMU或者IO的过程

#if HAL_USE_ICU == TRUE
    //attach timer channel on which the signal will be received
    sig_reader.attach_capture_timer(&RCIN_ICU_TIMER, RCIN_ICU_CHANNEL, STM32_RCIN_DMA_STREAM, STM32_RCIN_DMA_CHANNEL);
#endif

#if HAL_USE_EICU == TRUE
    sig_reader.init(&RCININT_EICU_TIMER, RCININT_EICU_CHANNEL);
#endif

    _init = true;
}

bool RCInput::new_input()
{
    if (!_init) {
        return false;
    }
    if (!rcin_mutex.take_nonblocking()) {
        return false;
    }
    bool valid = _rcin_timestamp_last_signal != _last_read;

    _last_read = _rcin_timestamp_last_signal;
    rcin_mutex.give();

#if HAL_RCINPUT_WITH_AP_RADIO
    if (!_radio_init) {
        _radio_init = true;
        radio = AP_Radio::get_singleton();
        if (radio) {
            radio->init();
        }
    }
#endif    
    return valid;
}

uint8_t RCInput::num_channels()
{
    if (!_init) {
        return 0;
    }
    return _num_channels;
}

uint16_t RCInput::read(uint8_t channel)
{
    if (!_init || (channel >= MIN(RC_INPUT_MAX_CHANNELS, _num_channels))) {
        return 0;
    }
    rcin_mutex.take(HAL_SEMAPHORE_BLOCK_FOREVER);
    uint16_t v = _rc_values[channel];
    rcin_mutex.give();
#if HAL_RCINPUT_WITH_AP_RADIO
    if (radio && channel == 0) {
        // hook to allow for update of radio on main thread, for mavlink sends
        radio->update();
    }
#endif
    return v;
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    if (!_init) {
        return false;
    }
 
    if (len > RC_INPUT_MAX_CHANNELS) {
        len = RC_INPUT_MAX_CHANNELS;
    }
    for (uint8_t i = 0; i < len; i++){
        periods[i] = read(i);
    }
    return len;
}

void RCInput::_timer_tick(void)
{
    if (!_init) {
        return;
    }

#ifndef HAL_NO_UARTDRIVER
    const char *rc_protocol = nullptr;
#endif

#ifndef HAL_BUILD_AP_PERIPH
#if HAL_USE_ICU == TRUE
    const uint32_t *p;
    uint32_t n;
    while ((p = (const uint32_t *)sig_reader.sigbuf.readptr(n)) != nullptr) {
        AP::RC().process_pulse_list(p, n*2, sig_reader.need_swap);
        sig_reader.sigbuf.advance(n);
    }
#endif

#if HAL_USE_EICU == TRUE
    uint32_t width_s0, width_s1;
    while(sig_reader.read(width_s0, width_s1)) {
        AP::RC().process_pulse(width_s0, width_s1);
    }
#endif

    if (AP::RC().new_input()) {
        rcin_mutex.take(HAL_SEMAPHORE_BLOCK_FOREVER);
        _rcin_timestamp_last_signal = AP_HAL::micros();
        _num_channels = AP::RC().num_channels();
        _num_channels = MIN(_num_channels, RC_INPUT_MAX_CHANNELS);
        for (uint8_t i=0; i<_num_channels; i++) {
            _rc_values[i] = AP::RC().read(i);
        }
        rcin_mutex.give();
#ifndef HAL_NO_UARTDRIVER
        rc_protocol = AP::RC().protocol_name();
#endif
    }
#endif // HAL_BUILD_AP_PERIPH

#if HAL_RCINPUT_WITH_AP_RADIO
    if (radio && radio->last_recv_us() != last_radio_us) {
        last_radio_us = radio->last_recv_us();
        rcin_mutex.take(HAL_SEMAPHORE_BLOCK_FOREVER);
        _rcin_timestamp_last_signal = last_radio_us;
        _num_channels = radio->num_channels();
        _num_channels = MIN(_num_channels, RC_INPUT_MAX_CHANNELS);
        for (uint8_t i=0; i<_num_channels; i++) {
            _rc_values[i] = radio->read(i);
        }
        rcin_mutex.give();
    }
#endif

#if HAL_WITH_IO_MCU
    rcin_mutex.take(HAL_SEMAPHORE_BLOCK_FOREVER);
    if (AP_BoardConfig::io_enabled() &&
        iomcu.check_rcinput(last_iomcu_us, _num_channels, _rc_values, RC_INPUT_MAX_CHANNELS)) {
        _rcin_timestamp_last_signal = last_iomcu_us;
#ifndef HAL_NO_UARTDRIVER
        rc_protocol = iomcu.get_rc_protocol();
#endif
    }
    rcin_mutex.give();
#endif

#ifndef HAL_NO_UARTDRIVER
    if (rc_protocol && rc_protocol != last_protocol) {
        last_protocol = rc_protocol;
        gcs().send_text(MAV_SEVERITY_DEBUG, "RCInput: decoding %s", last_protocol);
    }
#endif

    // note, we rely on the vehicle code checking new_input()
    // and a timeout for the last valid input to handle failsafe
}

/*
  start a bind operation, if supported
 */
bool RCInput::rc_bind(int dsmMode)
{
#if HAL_WITH_IO_MCU
    rcin_mutex.take(HAL_SEMAPHORE_BLOCK_FOREVER);
    if (AP_BoardConfig::io_enabled()) {
        iomcu.bind_dsm(dsmMode);
    }
    rcin_mutex.give();
#endif

#ifndef HAL_BUILD_AP_PERIPH
    // ask AP_RCProtocol to start a bind
    AP::RC().start_bind();
#endif

#if HAL_RCINPUT_WITH_AP_RADIO
    if (radio) {
        radio->start_recv_bind();
    }
#endif
    return true;
}
#endif //#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
