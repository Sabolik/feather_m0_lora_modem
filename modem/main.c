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

#include "lmic/lmic.h"
#include "modem.h"
#include "hal/hw.h"

//////////////////////////////////////////////////
// UTILITY JOB
//////////////////////////////////////////////////

#define RX_TIMEOUT_PERIOD           100

#define BLINK_PERIOD_RUNNING_ms     5000
#define BLINK_PERIOD_NO_SESSION_ms  1000
#define BLINK_PERIOD_DEFAULT_ms     BLINK_PERIOD_NO_SESSION_ms

#define BLINK_DURATION_TX_ms        500 
#define BLINK_DURATION_DEFAULT_ms   5

static u2_t blinkPeriod_ms = BLINK_PERIOD_DEFAULT_ms;

static void rxtimeoutcheckfunc (osjob_t* job);
static void oneblinkfunc (osjob_t* job);
static void blinkfunc (osjob_t* job);

static osjob_t rxtimeoutcheckjob;
static osjob_t oneblinkjob;
static osjob_t blinkjob;

static void rxtimeoutcheckfunc (osjob_t* job) {
    if( radio_is_rx_timeout() ) radio_irq_handler(1);
    
    // reschedule blink job
    os_setTimedCallback(job, os_getTime()+ms2osticks(RX_TIMEOUT_PERIOD), rxtimeoutcheckfunc);
}

static void oneblinkfunc (osjob_t* job) {
    hal_pin_led(0);
    // cancel blink job
    os_clearCallback(job);
    // Blink every blinkPeriod_ms
    os_setTimedCallback(&blinkjob, os_getTime()+ms2osticks(blinkPeriod_ms), blinkfunc);
}

static void blinkfunc (osjob_t* job) {
    hal_pin_led(1);
    // reschedule blink job
    // switch off LED
    os_setTimedCallback(&oneblinkjob, os_getTime()+ms2osticks(BLINK_DURATION_DEFAULT_ms), oneblinkfunc);
}

// initial job
static void initfunc (osjob_t* job) {
    // reset MAC state
    LMIC_reset();
    // start modem
    modem_init();
}

int main () {
    osjob_t myjob;

    // initialize runtime env
    os_init();
    // setup initial job
    os_setCallback(&myjob, initfunc);
    //os_setCallback(&blinkjob, blinkfunc);
    // execute scheduled jobs and events
    os_runloop();
    // (not reached)
    return 0;
}

void onEventMainCallback (ev_t ev) {
    if ( ev == EV_TXSTART )
    {
        // check for RX timeout manually by polling the radio instead of waiting for interrupt
        // Required radio pin not available on MCU -> thus polling instead of needs of wire soldering.
        os_setTimedCallback(&rxtimeoutcheckjob, os_getTime()+ms2osticks(RX_TIMEOUT_PERIOD), rxtimeoutcheckfunc);
    }
    else
    {
        if ( ev == EV_TXCOMPLETE )
        {
            // longer LED On duration indicates packet to be sent
            hal_pin_led(1);
            os_setTimedCallback(&oneblinkjob, os_getTime()+ms2osticks(BLINK_DURATION_TX_ms), oneblinkfunc);
        }
        // stop timeout polling, e.g. TX_COMPLETE event
        os_clearCallback(&rxtimeoutcheckjob);
    }
}

void leds_set (u1_t id, u1_t state) {
    // Only One Red Led available on Adafruit board
    if ( id == LED_SESSION )
    {
        // If joining, Led is blinking in 100ms period
        // Blinking from modem routine directly, but since 
        // BLINK_PERIOD_RUNNING_ms and BLINK_PERIOD_NO_SESSION_ms
        // are longer then 100ms, it can switch between them
        blinkPeriod_ms = state ? BLINK_PERIOD_RUNNING_ms : BLINK_PERIOD_NO_SESSION_ms;
        os_clearCallback(&blinkjob);
        os_setCallback(&blinkjob, blinkfunc);
    }
}
