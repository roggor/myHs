/*
 * stanowiska.cpp
 *
 *  Created on: Oct 5, 2015
 *      Author: maciek
 */

#include <stdio.h>
#include <string.h>

#include "configGlobal.h"
#include "protoCol.h"
#include "protoDevR05.h"


void protoDevR05_t::setDigOutputOn(DIG_OUT_t wy, bool wrHw, bool hiPrio)
{
	digOut_[wy/8] |= 1 << (wy%8);

	if(wrHw)
		hwWriteAll(hiPrio);
}

void protoDevR05_t::setDigOutputOff(DIG_OUT_t wy, bool wrHw, bool hiPrio)
{
	digOut_[wy/8] &= ~(1 << (wy%8));

	if(wrHw)
		hwWriteAll(hiPrio);
}

void protoDevR05_t::setDispString(char *s, bool wrHw, bool hiPrio)
{
	//tutaj tez w stringu trzeba znalezc kropki i updatowac dispDot_
	unsigned char k;
	for (k=0; k<DISP_SIZE; k++)
		disp_[k] = s[k];

	if (wrHw)
		hwWriteAll(hiPrio);
}

void protoDevR05_t::setDispStringOpts(char *s, DISP_OPTS_t blinkOpts, bool wrHw, bool hiPrio)
{
	setDispString(s, false, hiPrio);
	dispOpts_ = blinkOpts;

	if (wrHw)
		hwWriteAll(hiPrio);
}

void protoDevR05_t::hwWriteAll(bool hiPrio)
{
	//to interfejs do protokolu, dodanie adresu, demultipleksacja z obiektw na interfejs
	//do dodania jeszcze interfejs tutaj ... /dev/ttySx
	protoFuncWriteAll(rs485Addr, digOut_, disp_, dispDot_, dispOpts_, hiPrio);
}

void protoDevR05_t::hwReadAll(unsigned char *we, unsigned char wrzutnik, bool err, unsigned char *pCzytnik, unsigned char czytLen)
{
	//to interfejs z protokolu
	//readDigInput(we);
	//readWrzutnik(wrzutnik);
	//readErr(err);
	//readCzytnik(pCzytnik, czytLen);
}

void protoDevR05_t::hwReadBz(void)
{ }


#if 0
void protoDevR05_t::PeriodicPoll(void)
{
	WriteBz();
}

void protoDevR05_t::UpdWejscia(unsigned char *we)
{
	printf ("Stan %2d (addr 0x%2x): UpdWejscia\n", stanMapAddr2Idx(addr),addr);
}

void protoDevR05_t::WriteBz()
{
	protoFuncSendRaw(addr, FUNC_WRITE_BZ, NULL, 0, true);
}

void protoDevR05_t::WriteAll(void)
{
	unsigned char k;
	unsigned char buff[WRITE_ALL_MAX_DATA]={0, };

	//wyjscia
	//wy1      - offBits = 0, sizeBits=1
	//wy2      - offBits = 1, sizeBits=1
	// ...
	//wy9      - offBits = 9, sizeBits=1

	//grzalka  - offBits = 10, sizeBits=1
	//silownik - offBits = 11, sizeBits=1
	//wrzDis   - offBits = 12, sizeBits=1

	//disp     - offbits = 16, sizeBits=32
	//dispDots - offBits = 48, sizeBits=4
	//dispBlink- offBits = 52, sizeBits=2

	for (k=0; k<WRITE_ALL_WY_NR; k++)
		buff[wy[k].offBits / 8] |= ((wy[k].data ? 1 : 0) << (7-(wy[k].offBits % 8)));

	//grzalka
	buff[grzalka.offBits / 8] |= ((grzalka.data ? 1 : 0) << (7-(grzalka.offBits % 8)));
	//silownik
	buff[silownik.offBits / 8] |= ((silownik.data ? 1 : 0) << (7-(silownik.offBits % 8)));
	//wrztDis
	buff[wrztDis.offBits / 8] |= ((wrztDis.data ? 1 : 0) << (7-(wrztDis.offBits % 8)));

	//disp
	memcpy((buff+(disp.offBits/8)), disp.data, (disp.sizeBits / 8));

	//dispOpt
	for (k=0; k<dispDots.sizeBits; k++)
		buff[dispDots.offBits / 8] |= ((dispDots.data[k] ? 1 : 0) << (7 - (dispDots.offBits % 8) - k));

	//dispBlink
	for (k=0; k<dispBlink.sizeBits; k++)
		buff[dispBlink.offBits / 8] |= (dispBlink.data << (7-(dispBlink.offBits % 8)));

	protoFuncSendRaw(addr,FUNC_WRITE_ALL,buff,WRITE_ALL_MAX_DATA,false);
}

#endif


