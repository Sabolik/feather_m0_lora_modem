/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "../lmic/lmic.h"
#include "../modem.h"
#include "hw.h"

// -----------------------------------------------------------------------------
// I/O

static void hal_dio0_activated(void);

static void hal_io_init () {
    hal_samd_io_init( hal_dio0_activated );
}

// val ==1  => tx 1, rx 0 ; val == 0 => tx 0, rx 1
void hal_pin_rxtx (u1_t val) {
    return;
}

// set radio NSS pin to given value
void hal_pin_nss (u1_t val) {
    if (val) {
        hal_samd_set_pin_level_rfm_cs();
    }
    else {
        hal_samd_clr_pin_level_rfm_cs();
    }
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if(val == 0 || val == 1) { // drive pin
        if ( val ){
            hal_samd_set_pin_level_rfm_rst();
        }
        else{
            hal_samd_clr_pin_level_rfm_rst();
        }
        hal_samd_set_pin_output_rfm_rst();
    } else { // keep pin floating
        hal_samd_set_pin_input_rfm_rst();
    }
}

void hal_pin_led (u1_t val) {
    if (val) {
        hal_samd_set_pin_level_red_led();
    }
    else {
        hal_samd_clr_pin_level_red_led();
    }
}

extern void radio_irq_handler(u1_t dio);

static void hal_dio0_activated(void) {
    radio_irq_handler(0);
}

// -----------------------------------------------------------------------------
// SPI

// for sx1272 and 1276

static void hal_spi_init () {
    return;
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    return (u1_t)hal_samd_spi(out);
}

// -----------------------------------------------------------------------------
// TIME

static void hal_rtc_isr_handler(void) {
    hal_samd_rtc_clear_isr_flag();
}

static void hal_time_init () {
    /* Enable RTC NVIC Interrupt Line */
    hal_samd_rtc_isr_cb( hal_rtc_isr_handler );
    hal_samd_rtc_init();
}

u4_t hal_ticks () {
    return (u4_t) hal_samd_rtc_get_time();
}

// return modified delta ticks from now to specified ticktime (0 for past, FFFF for far future)
static u2_t deltaticks (u4_t time) {
    u4_t t = hal_ticks();
    s4_t d = time - t;
    if( d<=0 ) return 0;    // in the past
    if( (d>>16)!=0 ) return 0xFFFF; // far ahead
    return (u2_t)d;
}

void hal_waitUntil (u4_t time) {
    while( deltaticks(time) != 0 ); // busy wait until timestamp is reached
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    u2_t dt;
    if((dt = deltaticks(time)) < 5) // event is now (a few ticks ahead)
    {
        return 1;
    }
    else // rewind timer (fully or to exact time))
    {
        hal_samd_rtc_set_wakeuptime( time );
        return 0;
    }
}

// -----------------------------------------------------------------------------
// IRQ

void hal_disableIRQs () {
    hal_samd_disable_irq();
}

void hal_enableIRQs () {
    hal_samd_enable_irq();
}

// -----------------------------------------------------------------------------
// I2C

void hal_i2c_ioInit(s2_t addr) {
    hal_samd_i2c_ioInit(addr);
}

void hal_i2c_writeByte(u1_t reg_addr, u1_t data) {
    hal_samd_i2c_writeByte( reg_addr, data );
}

void hal_i2c_readBlock(u1_t reg_addr, u1_t *read_buff, u1_t length) {
    hal_samd_i2c_readBlock(reg_addr, read_buff, length);
}

// -----------------------------------------------------------------------------
// USB
extern FRAME txframe;

static void usart_tx_done(void) {
    txframe.len = txframe.max;  // update txframe.len to free the buffer
    frame_tx(0);
}

static void usart_rx_done(uint8_t* data, uint16_t len) {
    u1_t receptionEnabled = 1;
    
    while ( receptionEnabled && len-- ) {
        receptionEnabled = frame_rx(*(data++));
    }
    // continue reading until enabled - not ready to reply
    if ( receptionEnabled ) {
        usart_startrx();
    }
}

static void usb_serial_init(void) {
    hal_samd_usb_serial_init();
}

static void usb_serial_start(void) {
    u4_t waitPeriod = os_getTime() + ms2osticks(1000);
    
    while (!hal_samd_usb_serial_initialized() && deltaticks(waitPeriod)) {
		// wait cdc acm to be installed
        // up to 1s, otherwise deinitialize acm
	};
    
    if ( !deltaticks(waitPeriod) ) {
        hal_samd_usb_serial_deinit();
    }
    else {
        hal_samd_usb_serial_start(usart_rx_done, usart_tx_done);
    } 
}

void usart_starttx () {
    if ( ! hal_samd_usb_serial_starttx(txframe.buf, txframe.max) )
    {
        // clear outcomming buffers etc.
        usart_tx_done();
    } 
}

void usart_startrx () {
    hal_samd_usb_serial_startrx();
}

// -----------------------------------------------------------------------------
// Modem Callbacks

void usart_init (void){
    usb_serial_init();
    usb_serial_start();
}

void leds_init (void) {
}

static void persistance_write(void* dst, const void* src, u2_t len, u2_t storage_size) {
    hal_samd_persistance_write(dst, src, len, storage_size);
}

void eeprom_write (u4_t* addr, u4_t val) {
    persistance_write(addr, &val, sizeof(val), sizeof(persist_t));
}

void eeprom_copy (void* dst, const void* src, u2_t len) {
    persistance_write(dst, src, len, sizeof(persist_t));
}

// -----------------------------------------------------------------------------
// Sensor Callbacks

void sensor_power (u1_t state) {
    if (state) {
        hal_samd_sensor_set_power();
    }
    else {
        hal_samd_sensor_clr_power();
    }
}

// -----------------------------------------------------------------------------

void hal_sleep () {
    if ( hal_samd_usb_serial_is_off() ) {
        hal_samd_sleep();
    }
    else {
        // powered from usb, do not sleep
    }
}

void hal_init () {
    hal_disableIRQs();
    hal_samd_system_init();
    hal_samd_persistance_init();
    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
    hal_enableIRQs();
}

void hal_failed () {
    // HALT...
    hal_disableIRQs();
    //hal_sleep();
    while(1)
    {
        hal_pin_led(1);
        hal_waitUntil( hal_ticks() + ms2osticks(900) );
        hal_pin_led(0);
        hal_waitUntil( hal_ticks() + ms2osticks(100) );
    }
}

   