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


#include "../basicmac/lmic/lmic.h"

// modem version
#define VERSION_MAJOR 2
#define VERSION_MINOR 4
#define VERSION_STR   "VERSION 2.4 ("__DATE__" "__TIME__")"

// LED ids
#define LED_SESSION 1  // (IMST: yellow, LRSC: green)
#define LED_POWER   2  // (IMST: green,  LRSC: red)
#define LED_TX_DONE 3

// patch patterns
#define PATTERN_JOINCFG_STR "g0CMw49rRbav6HwQN0115g42OpmvTn7q" // (32 bytes)
#define PATTERN_JOINCFG_HEX "6730434d7734397252626176364877514e303131356734324f706d76546e3771"
#define PATTERN_JOINCFG_CRC 0x6B3D
#define PATTERN_SESSCFG_STR "Bmf3quaCJwVKURWWREeGKtm0pqLD0Yhr5cpPkP6s" // (40 bytes)
#define PATTERN_SESSCFG_HEX "426d6633717561434a77564b55525757524565474b746d3070714c4430596872356370506b503673"
#define PATTERN_SESSCFG_CRC 0xC9D5

#define CMD_BUF_LEN 200
#define RSP_BUF_LEN 100

// layout of join paramters
typedef struct {
    u1_t deveui[8];
    u1_t appeui[8];
    u1_t devkey[16];
} joinparam_t;

// layout of session parameters
typedef struct {
    u4_t netid;
    devaddr_t devaddr;
    u1_t nwkkey[16];
    u1_t artkey[16];
} sessparam_t;

// layout of periodic AT command execution
typedef struct {
    u4_t cmdperiod;
    u4_t cmdlen;
    u1_t cmdbuf[CMD_BUF_LEN];
} cmdparam_t;

// persistent state
typedef struct {
    u4_t cfghash;
    u4_t flags;
    joinparam_t joinpar;
    sessparam_t sesspar;
    u4_t seqnoDn;
    u4_t seqnoUp;
    u4_t eventmask;
    u1_t regcode;
    cmdparam_t cmdparam;
} persist_t;

#define FLAGS_JOINPAR 0x01
#define FLAGS_SESSPAR 0x02

#define PERSIST ((persist_t*)EEPROM_BASE)

// frame rx/tx state
#define FRAME_INIT   0x00
#define FRAME_A_A    0xA1
#define FRAME_A_T    0xA2
#define FRAME_A_OK   0xA3
#define FRAME_A_ERR  0xA4
#define FRAME_B_B    0xB1
#define FRAME_B_LEN  0xB2
#define FRAME_B_LRC  0xB3
#define FRAME_B_OK   0xB4
#define FRAME_B_ERR  0xB5

typedef struct {
    u1_t state;
    u1_t *buf;
    u2_t len;
    u2_t max;
    u1_t lrc;
 } FRAME;


void modem_init (void);
void modem_rxdone (osjob_t* j);
void modem_txdone (osjob_t* j);

void buffer_init (void);
u1_t* buffer_alloc (u2_t len);
void buffer_free (u1_t* buf, u2_t len);

void queue_init (void);
void queue_add (u1_t* buf, u2_t len);
u1_t queue_shift (FRAME* f);

void frame_init (FRAME* f, u1_t* buf, u2_t max);
u2_t frame_tx (u1_t next);
u1_t frame_rx (u1_t c);

void usart_init (void);
void usart_starttx (void);
void usart_startrx (void);
void usart_tx_done(void);

void leds_init (void);
void leds_set (u1_t id, u1_t state);

u1_t gethex (u1_t* dst, const u1_t* src, u2_t len);
u1_t puthex (u1_t* dst, const u1_t* src, u1_t len);
u1_t putshort (u1_t* dst, const u2_t* src, u1_t len, u1_t decimalplaces);
u1_t int2hex (u1_t* dst, u4_t v);
u1_t short2hex (u1_t* dst, u2_t v);
u1_t byte2hex (u1_t* dst, u1_t v);
u1_t hex2int (u4_t* n, const u1_t* src, u1_t len);
u1_t hex2short (u2_t* n, const u1_t* src, u1_t len);
u1_t hex2byte (u1_t* n, const u1_t* src, u1_t len);
u1_t dec2int (u4_t* n, const u1_t* src, u1_t len);
void reverse (u1_t* dst, const u1_t* src, u1_t len);
u1_t tolower (u1_t c);
u1_t toupper (u1_t c);

u1_t cpystr (u1_t* dst, const char* src);
u1_t cmpstr (u1_t* buf, u1_t len, char* str);

void modem_transmitdata (u1_t* buf, u2_t len);
