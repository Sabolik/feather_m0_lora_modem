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

#include "modem.h"
#include "hal/hw.h"
#include "sensor/sensor.h"

//////////////////////////////////////////////////
// CONFIGURATION (WILL BE PATCHED)
//////////////////////////////////////////////////

static void cmd_process (u1_t *cmdbuf, u1_t cmdlen);

_Static_assert(sizeof(persist_t) <= EEPROM_SIZE, "persist_t struct too large");

static const union {
    joinparam_t param;
    u1_t pattern[sizeof(joinparam_t)];
} joincfg = {
    .pattern = { PATTERN_JOINCFG_STR }
};

static const union {
    sessparam_t param;
    u1_t pattern[sizeof(sessparam_t)];
} sesscfg = {
    .pattern = { PATTERN_SESSCFG_STR }
};

// COMMAND TO SET DEVEUI=FF-FF-FF-FF-FF-FF-FF-00, APPEUI=DE-DE-AA-AA-00-00-00-1A (PORT=25501):
//   ATJ=FFFFFFFFFFFFFF00,DEDEAAAA0000001A,AA5555555555555555AAAAAAAAAAAAAA
//   OK

static const char* evnames[] = {
    [EV_SCAN_TIMEOUT]   = "SCAN_TIMEOUT",
    [EV_BEACON_FOUND]   = "BEACON_FOUND",
    [EV_BEACON_MISSED]  = "BEACON_MISSED",
    [EV_BEACON_TRACKED] = "BEACON_TRACKED",
    [EV_JOINING]        = "JOINING",
    [EV_JOINED]         = "JOINED",
    [EV_RFU1]           = "RFU1",
    [EV_JOIN_FAILED]    = "JOIN_FAILED",
    [EV_REJOIN_FAILED]  = "REJOIN_FAILED",
    [EV_TXCOMPLETE]     = "TXCOMPLETE",
    [EV_LOST_TSYNC]     = "LOST_TSYNC",
    [EV_RESET]          = "RESET",
    [EV_RXCOMPLETE]     = "RXCOMPLETE",
    [EV_LINK_DEAD]      = "LINK_DEAD",
    [EV_LINK_ALIVE]     = "LINK_ALIVE",
    [EV_SCAN_FOUND]     = "SCAN_FOUND",
    [EV_TXSTART]        = "TXSTART",
    [EV_TXDONE]         = "TXDONE",
    [EV_DATARATE]       = "DATARATE",
    [EV_START_SCAN]     = "START_SCAN",
    [EV_ADR_BACKOFF]    = "ADR_BACKOFF",
};

// return mask of all events matching given string prefix
static u4_t evmatch (u1_t* name, u1_t len) {
    u4_t mask = 0;
    for(u1_t e=0; e < sizeof(evnames)/sizeof(evnames[0]); e++) {
	if(evnames[e]) {
	    u1_t l;
	    for(l=0; l<len && toupper(name[l]) == evnames[e][l]; l++); // compare
	    if(l == len) mask |= (1 << e);
	}
    }
    return mask;
}


// transient state
static struct {
    u1_t cmdbuf[CMD_BUF_LEN];
    u1_t rspbuf[RSP_BUF_LEN];
    u2_t rsplen;
    u1_t txpending;
    osjob_t alarmjob;    
    osjob_t blinkjob;
    osjob_t ledjob;
    osjob_t cmdrepeatjob;
} MODEM;

// provide region code
u1_t os_getRegion (void) {
    return PERSIST->regcode;
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, PERSIST->joinpar.deveui, 8);
}

// provide join ID (8 bytes, LSBF)
void os_getJoinEui (u1_t* buf) {
    memcpy(buf, PERSIST->joinpar.appeui, 8);
}

// provide device network key (16 bytes)
void os_getNwkKey (u1_t* buf) {
    memcpy(buf, PERSIST->joinpar.devkey, 16);
}

// provide device application key (16 bytes)
void os_getAppKey (u1_t* buf) {
    memcpy(buf, PERSIST->sesspar.artkey, 16);
}

extern FRAME rxframe;
extern FRAME txframe;


// blink session led
static void blinkfunc (osjob_t* j) {
    static u1_t ledstate;
    // toggle LED
    ledstate = !ledstate;
    leds_set(LED_SESSION, ledstate);
    // reschedule blink job
    os_setTimedCallback(j, os_getTime()+ms2osticks(100), blinkfunc);
}

// switch on power led again
static void ledfunc (osjob_t* j) {
    leds_set(LED_POWER, 1);
}

// start transmission of prepared response or queued event message
static void modem_starttx () {
    hal_disableIRQs();
    if( !MODEM.txpending ) {
	// check if we have something to send
	if(queue_shift(&txframe)) { // send next queued event
	    usart_starttx();
	    MODEM.txpending = 1;
	} else if(MODEM.rsplen) { // send response
	    frame_init(&txframe, MODEM.rspbuf, MODEM.rsplen);
	    MODEM.rsplen = 0;
	    usart_starttx();
	    MODEM.txpending = 1;
	}
    }
    hal_enableIRQs();
}

// LRSC MAC event handler
// encode and queue event for output
void onLmicEvent (ev_t ev) {
    // turn LED off for a short moment
    leds_set(LED_POWER, 0);
    os_setTimedCallback(&MODEM.ledjob, os_getTime()+ms2osticks(200), ledfunc);

    // update sequence counters for session
    if(PERSIST->flags & FLAGS_SESSPAR) {
	eeprom_write(&PERSIST->seqnoUp, LMIC.seqnoUp);
	eeprom_write(&PERSIST->seqnoDn, LMIC.seqnoDn);
    }

    // take action on specific events
    switch(ev) {
    case EV_JOINING:
	// start blinking
	blinkfunc(&MODEM.blinkjob);
	break;
    case EV_JOINED: {
	// cancel blink job
	os_clearCallback(&MODEM.blinkjob);
	// switch on LED
	leds_set(LED_SESSION, 1);
	// save newly established session
	sessparam_t newsession;
	newsession.netid = LMIC.netid;
	newsession.devaddr = LMIC.devaddr;
	memcpy(newsession.nwkkey, LMIC.lceCtx.nwkSKey, 16);
	memcpy(newsession.artkey, LMIC.lceCtx.appSKey, 16);
	eeprom_copy(&PERSIST->sesspar, &newsession, sizeof(newsession));
	eeprom_write(&PERSIST->seqnoDn, LMIC.seqnoDn);
	eeprom_write(&PERSIST->seqnoUp, LMIC.seqnoUp);
	eeprom_write(&PERSIST->flags, PERSIST->flags | FLAGS_SESSPAR);
      }
    case EV_TXCOMPLETE:
    leds_set(LED_TX_DONE, 1);
    break;
    default: break;
    }

    // report events (selected by eventmask)
    if(PERSIST->eventmask & (1 << ev)) {
	u1_t len = strlen(evnames[ev]);
	u1_t* buf;
	switch(ev) {
	case EV_TXCOMPLETE:
	case EV_RXCOMPLETE: { // report EV_XXCOMPLETE,<flags>[,<port>[,<downstream data>]]
	    u1_t f = LMIC.txrxFlags; // EV_XXCOMPLETE,FF[,PP[,DDDDDDDD...]]
	    buf = buffer_alloc(3 + len + 1 + 2 + ((f & TXRX_PORT) ? 3 : 0) + (LMIC.dataLen ? 1+2*LMIC.dataLen:0) + 2);
	    memcpy(buf, "EV_", 3);
	    memcpy(buf+3, evnames[ev], len);
	    len += 3;
	    buf[len++] = ',';
	    // flags
	    buf[len++] = (f & TXRX_ACK) ? 'A' : (f & TXRX_NACK) ? 'N' : '0';
	    buf[len++] = (f & TXRX_DNW1) ? '1' : (f & TXRX_DNW2) ? '2' : (f & TXRX_PING) ? 'P' : '0';
	    if(f & TXRX_PORT) { // port
		buf[len++] = ',';
		len += puthex(buf+len, &LMIC.frame[LMIC.dataBeg-1], 1);
	    }
	    if(LMIC.dataLen) { // downstream data
		buf[len++] = ',';
		len += puthex(buf+len, LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
	    }
	    break;
	}
	default: // just report event name
	    buf = buffer_alloc(3+len+2);
	    memcpy(buf, "EV_", 3);
	    memcpy(buf+3, evnames[ev], len);
	    len += 3;
	    break;
	}
	buf[len++] = '\r';
	buf[len++] = '\n';
	queue_add(buf, len);
	modem_starttx();
    }
}

static void onAlarm (osjob_t* j) {
    queue_add((u1_t*)"EV_ALARM\r\n", 10);
    modem_starttx();
}

static void onCmdRepeat (osjob_t* j) {
    cmd_process(PERSIST->cmdparam.cmdbuf, PERSIST->cmdparam.cmdlen);
    
    if (PERSIST->cmdparam.cmdperiod) {  // repeat
        os_setTimedCallback(j, os_getTime()+sec2osticks(PERSIST->cmdparam.cmdperiod), onCmdRepeat);
    }
}

static void modem_reset () {
    os_clearCallback(&MODEM.alarmjob);      // cancel alarm job
    os_clearCallback(&MODEM.blinkjob);      // cancel blink job
    os_clearCallback(&MODEM.ledjob);        // cancel LED job
    os_clearCallback(&MODEM.cmdrepeatjob);  // cancel repeat job


    leds_set(LED_POWER, 1); // LED on
    leds_set(LED_SESSION, 0); // LED off

    LMIC_reset();

    // check for existing session
    if(PERSIST->flags & FLAGS_SESSPAR) {
	// configure session
	LMIC_setSession(PERSIST->sesspar.netid, PERSIST->sesspar.devaddr, PERSIST->sesspar.nwkkey, PERSIST->sesspar.artkey);
	LMIC.seqnoDn = PERSIST->seqnoDn;
	LMIC.seqnoUp = PERSIST->seqnoUp + 2; // avoid reuse of seq numbers
	leds_set(LED_SESSION, 1); // LED on
    }
    
    if (PERSIST->cmdparam.cmdperiod) {  // repeat stored command periodically
        os_setTimedCallback(&MODEM.cmdrepeatjob, os_getTime()+sec2osticks(PERSIST->cmdparam.cmdperiod), onCmdRepeat);
    }
}

// initialize persistent state (factory reset)
static void persist_init (u1_t factory) {
    // check if stored state matches firmware config
    u2_t joincfgcrc = os_crc16((u1_t*)&joincfg, sizeof(joincfg));
    u2_t sesscfgcrc = os_crc16((u1_t*)&sesscfg, sizeof(sesscfg));
    u4_t cfghash = (joincfgcrc << 16) | sesscfgcrc;
    if(PERSIST->cfghash != cfghash || factory) {
	u4_t flags = 0;
	if(joincfgcrc != PATTERN_JOINCFG_CRC) { // patched
	    eeprom_copy(&PERSIST->joinpar, &joincfg, sizeof(joincfg));
	    flags |= FLAGS_JOINPAR;
	}
	if(sesscfgcrc != PATTERN_SESSCFG_CRC) { // patched
	    eeprom_copy(&PERSIST->sesspar, &sesscfg, sizeof(sesscfg));
	    eeprom_write(&PERSIST->seqnoDn, 0);
	    eeprom_write(&PERSIST->seqnoUp, 0);
	    flags |= FLAGS_SESSPAR;
	}
	eeprom_write(&PERSIST->flags, flags);
	eeprom_write(&PERSIST->eventmask, ~0); // report ALL events
	eeprom_write(&PERSIST->cfghash, cfghash);
    
    u1_t regcode_default = REGCODE_EU868; // Default region code
    eeprom_copy(&PERSIST->regcode, &regcode_default, sizeof(regcode_default));
    
    eeprom_write(&PERSIST->cmdparam.cmdperiod, 0);  // Defaults for AT CMD repeat
    eeprom_write(&PERSIST->cmdparam.cmdlen, 0);
    }
}   

// process command and prepare response in MODEM.cmdbuf[]
static void cmd_process (u1_t *cmdbuf, u1_t cmdlen) {    
    u1_t ok = 0;
    u1_t cmd = tolower(cmdbuf[0]);
    u1_t* rspbuf = MODEM.rspbuf;
    u1_t keeplastcommand = 1;
    static u1_t cmdbuflast[CMD_BUF_LEN];
    static u1_t cmdlenlast = 0;
    if(cmdlen == 0) { // AT
	ok = 1;
    } else if(cmd == 'v' && cmdlen == 2 && cmdbuf[1] == '?') { // ATV? query version
	rspbuf += cpystr(rspbuf, "OK,");
	rspbuf += cpystr(rspbuf, VERSION_STR);
	ok = 1;
    } else if(cmd == 'z' && cmdlen == 1) { // ATZ reset
	modem_reset();
	ok = 1;
    } else if(cmd == '&' && cmdlen == 2 && tolower(cmdbuf[1]) == 'f') { // AT&F factory reset
	persist_init(1);
	modem_reset();
	ok = 1;
    } else if(cmd == 'e' && cmdlen >= 2) { // event mask (query/set/add/remove)
	u1_t mode = cmdbuf[1];
	if(mode == '?' && cmdlen == 2) { // ATE? query
	    rspbuf += cpystr(rspbuf, "OK,");
	    if(PERSIST->eventmask == 0)       rspbuf += cpystr(rspbuf, "NONE");
	    else if(PERSIST->eventmask == ~0) rspbuf += cpystr(rspbuf, "ALL");
	    else {
		for(u1_t e=0; e<32; e++) {
		    if(e < sizeof(evnames)/sizeof(evnames[0]) && evnames[e] && (PERSIST->eventmask & (1 << e))) {
			if(rspbuf - cmdbuf != 3) *rspbuf++ = '|';
			rspbuf += cpystr(rspbuf, evnames[e]);
		    }
		}
	    }
	    ok = 1;
	} else if(mode == '=' || mode == '+' || mode == '-') { // ATE= ATE+ ATE- set/add/remove
	    u1_t i, j, clear = 0;
	    u4_t mask = 0;
	    for(i=2; i < cmdlen; i = j+1) {
		for(j=i; j < cmdlen && cmdbuf[j]!='|'; j++);
		mask |= evmatch(cmdbuf+i, j-i);
	    }
	    if(mask == 0 && mode == '=') {
		if(cmpstr(cmdbuf+2, cmdlen-2, "ALL")) mask = ~0;
		else if(cmpstr(cmdbuf+2, cmdlen-2, "NONE")) clear = 1;
	    }
	    if(mask || clear) {
		if(mode == '+')      mask = PERSIST->eventmask | mask;
		else if(mode == '-') mask = PERSIST->eventmask & ~mask;
		eeprom_write(&PERSIST->eventmask, mask);
		ok = 1;
	    }
	}
    } else if(cmd == 's' && cmdlen >= 2) { // SESSION parameters
	if(cmdbuf[1] == '?' && cmdlen == 2) { // ATS? query (netid,devaddr,seqnoup,seqnodn)
	    if(PERSIST->flags & FLAGS_SESSPAR) {
		rspbuf += cpystr(rspbuf, "OK,");
		rspbuf += int2hex(rspbuf, PERSIST->sesspar.netid);
		*rspbuf++ = ',';
		rspbuf += int2hex(rspbuf, PERSIST->sesspar.devaddr);
		*rspbuf++ = ',';
		rspbuf += int2hex(rspbuf, PERSIST->seqnoUp);
		*rspbuf++ = ',';
		rspbuf += int2hex(rspbuf, PERSIST->seqnoDn);
		ok = 1;
	    }
	} else if(cmdbuf[1] == '=' && cmdlen == 2+8+1+8+1+32+1+32) { // ATS= set (netid,devaddr,nwkkey,artkey)
	    sessparam_t par;
	    if( hex2int(&par.netid, cmdbuf+2, 8) &&
		cmdbuf[2+8] == ',' &&
		hex2int(&par.devaddr, cmdbuf+2+8+1, 8) &&
		cmdbuf[2+8+1+8] == ',' &&
		gethex(par.nwkkey, cmdbuf+2+8+1+8+1, 32) == 16 &&
		cmdbuf[2+8+1+8+1+32] == ',' &&
		gethex(par.artkey, cmdbuf+2+8+1+8+1+32+1, 32) == 16 ) {
		// use new parameters
		LMIC_reset();
		LMIC_setSession(par.netid, par.devaddr, par.nwkkey, par.artkey);
		// switch on LED
		leds_set(LED_SESSION, 1);
		// save parameters
		eeprom_copy(&PERSIST->sesspar, &par, sizeof(par));
		eeprom_write(&PERSIST->seqnoUp, LMIC.seqnoUp);
		eeprom_write(&PERSIST->seqnoDn, LMIC.seqnoDn);
		eeprom_write(&PERSIST->flags, PERSIST->flags | FLAGS_SESSPAR);
		ok = 1;
	    }
	}
    } else if(cmd == 'j' && cmdlen >= 2) { // JOIN parameters
	if(cmdbuf[1] == '?' && cmdlen == 2) { // ATJ? query (deveui,appeui)
	    if(PERSIST->flags & FLAGS_JOINPAR) {
		u1_t tmp[8];
		rspbuf += cpystr(rspbuf, "OK,");
		reverse(tmp, PERSIST->joinpar.deveui, 8);
		rspbuf += puthex(rspbuf, tmp, 8);
		*rspbuf++ = ',';
		reverse(tmp, PERSIST->joinpar.appeui, 8);
		rspbuf += puthex(rspbuf, tmp, 8);
		ok = 1;
	    }
	} else if(cmdbuf[1] == '=' && cmdlen == 2+16+1+16+1+32) { // ATJ= set (deveui,appeui,devkey)
	    joinparam_t par;
	    if( gethex(par.deveui, cmdbuf+2,           16) == 8 &&
		cmdbuf[2+16] == ',' &&
		gethex(par.appeui, cmdbuf+2+16+1,      16) == 8 &&
		cmdbuf[2+16+1+16] == ',' &&
		gethex(par.devkey, cmdbuf+2+16+1+16+1, 32) == 16 ) {
		reverse(par.deveui, par.deveui, 8);
		reverse(par.appeui, par.appeui, 8);
		eeprom_copy(&PERSIST->joinpar, &par, sizeof(par));
		eeprom_write(&PERSIST->flags, PERSIST->flags | FLAGS_JOINPAR);
		ok = 1;
	    }
	}
    } else if(cmd == 'j' && cmdlen == 1) { // ATJ join network
	if(PERSIST->flags & FLAGS_JOINPAR) {
	    LMIC_reset(); // force join
	    LMIC_startJoining();
	    ok = 1;
	}
    } else if(cmd == 't' && cmdlen >= 1) { // ATT transmit
	if(cmdlen == 1) { // no conf, no port, no data
	    if(LMIC.devaddr) { // valid session required!
		LMIC_sendAlive(); // send empty frame
		ok = 1;
	    }
	} else if(cmdlen >= 5) { // confirm,port[,data]  (0,FF[,112233...])
	    if((cmdbuf[1]=='0' || cmdbuf[1]=='1') && cmdbuf[2]==',' && // conf
	       gethex(&LMIC.pendTxPort, cmdbuf+1+1+1, 2) == 1 && LMIC.pendTxPort) { // port
		LMIC.pendTxConf = cmdbuf[1] - '0';
		LMIC.pendTxLen = 0;
		if(cmdlen > 5 && cmdbuf[5]==',') { // data
		    LMIC.pendTxLen = gethex(LMIC.pendTxData, cmdbuf+6, cmdlen-6);
		}
		if(cmdlen == 5 || LMIC.pendTxLen) {
		    if(LMIC.devaddr || (PERSIST->flags & FLAGS_JOINPAR)) { // implicitely join!
			LMIC_setTxData();
			ok = 1;
		    }
		}
	    }
	}
    } else if(cmd == 'p' && cmdlen == 2) { // ATP set ping mode
	if(LMIC.devaddr) { // requires a session
	    u1_t n = cmdbuf[1];
	    if(n>='0' && n<='7') {
		LMIC_setPingable(n-'0');
		ok = 1;
	    }
	}
    } else if(cmd == 'a' && cmdlen >= 2) { // ATA set alarm timer
	u4_t secs;
	if(hex2int(&secs, cmdbuf+1, cmdlen-1)) {
	    os_setTimedCallback(&MODEM.alarmjob, os_getTime()+sec2osticks(secs), onAlarm);
	    ok = 1;
	}
    } else if(cmd == 'r' && cmdlen >= 2) { // ATR region code
    if(cmdbuf[1] == '?' && cmdlen == 2) { // ATR? query (region code)
        rspbuf += cpystr(rspbuf, "OK,");
        rspbuf += byte2hex(rspbuf, (u1_t)PERSIST->regcode);
        ok = 1;
    }
    else if(cmdlen == 4 && cmdbuf[1]=='=') { // ATR= set region code (01)
        u1_t regcode_set;
        if( hex2byte(&regcode_set, cmdbuf+2, 2) &&
            regcode_set != REGCODE_UNDEF && // valid regcode
            regcode_set <= REGCODE_CN470 )  // would be better to add some size indicating last enum member
        {
            eeprom_copy(&PERSIST->regcode, &regcode_set, sizeof(PERSIST->regcode));
            modem_reset();
            ok = 1;
        }
    }
    } else if(cmd == 'l' && cmdlen >= 2) { // ATL last command repeat
    u4_t secs;
    keeplastcommand = 0;    // do not keep ATL itself
    if(cmdlen == 2 && cmdbuf[1]=='0') { // stop repeat
        eeprom_write(&PERSIST->cmdparam.cmdperiod, 0);
        os_clearCallback(&MODEM.cmdrepeatjob);  // clear pending job
        ok = 1;
    }
    else if(cmdlen >= 3 &&  // repeat,period
            cmdbuf[2]==',' &&
            hex2int(&secs, cmdbuf+3, cmdlen-3) &&
            (cmdbuf[1]=='0' || (cmdbuf[1]=='1' && secs)) &&
            cmdlenlast) {
        eeprom_write(&PERSIST->cmdparam.cmdperiod, cmdbuf[1]=='1' ? secs : 0);
        eeprom_copy(&PERSIST->cmdparam.cmdbuf, cmdbuflast, cmdlenlast);  // keep last AT command
        eeprom_write(&PERSIST->cmdparam.cmdlen, cmdlenlast);
        os_setTimedCallback(&MODEM.cmdrepeatjob, os_getTime()+sec2osticks(secs), onCmdRepeat);
        ok = 1; 
    }
    } else if(cmd == 'w' && cmdlen >= 2) { // ATW weather data
    if(cmdbuf[1] == '?' && cmdlen == 2 && sensor_txterminal()) { // ATW? query (temperature,humidity,pressure)
        ok = 1;
    }
    else if(cmdlen == 8 && // ATW set confirm,port  (0,FF,FF)
            (LMIC.devaddr || (PERSIST->flags & FLAGS_JOINPAR))) { // implicitely join!
        u1_t pendTxPort;
        u1_t dataChannel;
        if((cmdbuf[1]=='0' || cmdbuf[1]=='1') && cmdbuf[2]==',' && // conf
        gethex(&pendTxPort, cmdbuf+3, 2) == 1 && pendTxPort && // port
        gethex(&dataChannel, cmdbuf+6, 2) == 1 && dataChannel) { // lpp data channel
            if (sensor_txradio(pendTxPort, cmdbuf[1] - '0', dataChannel)) {
                ok = 1;
            }
        }
    }
}

    // send response
    if(ok) {
    if(keeplastcommand){
        cmdlenlast = cmdlen;
        memcpy(cmdbuflast, cmdbuf, cmdlenlast);
    }
	if(rspbuf == MODEM.rspbuf) {
	    rspbuf += cpystr(rspbuf, "OK");
	}
    } else {
	rspbuf += cpystr(rspbuf, "ERROR");
    }
    *rspbuf++ = '\r';
    *rspbuf++ = '\n';
    MODEM.rsplen = rspbuf - MODEM.rspbuf;
    modem_starttx();
}

// called by initial job
void modem_init () {
    // clear modem state
    memset(&MODEM, 0, sizeof(MODEM));

    persist_init(0);

    leds_init();

    buffer_init();
    queue_init();

    // initialize USART
    usart_init();

    // start reception of command
    frame_init(&rxframe, MODEM.cmdbuf, sizeof(MODEM.cmdbuf));
    
    modem_reset();
    
    // query version, start Rx afterward 
    cmd_process ((u1_t*)"V?", 2);
}

// called by frame job
void modem_rxdone (osjob_t* j) {
    cmd_process(MODEM.cmdbuf, rxframe.len);
}    

// called by frame job
void modem_txdone (osjob_t* j) {
    MODEM.txpending = 0;
    // free events (static buffers ignored)
    buffer_free(txframe.buf, txframe.len);
    if(txframe.buf == MODEM.rspbuf) { // response transmitted
	// restart reception for new command
	frame_init(&rxframe, MODEM.cmdbuf, sizeof(MODEM.cmdbuf));
	usart_startrx();
    }
    // start transmission of next output message
    modem_starttx();
}

void modem_transmitdata (u1_t* buf, u2_t len)
{
    queue_add(buf, len);
    modem_starttx();
}