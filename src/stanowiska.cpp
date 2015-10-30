/*
 * stanowiska.cpp
 *
 *  Created on: Oct 5, 2015
 *      Author: maciek
 */
#include <stdio.h>
#include <string.h>

#include "globalConfig.h"
#include "protocol.h"
#include "stanowiska.h"

Stanowiska_t Stanowiska[MAX_STAN+1];

/*
 * mapIdx2Addr[idx] - zwraca adres stanowiska, defaultowo idx=0->addr=0x0, idx=1->addr=0x1 itd
 * dla idx=STAN_IDX_DUMMY zwraca wartosc 0xff
 */
static unsigned char mapIdx2Addr[MAX_STAN]=DEF_MAP_IDX2ADDR;

/*
 * mapAddr2Idx[addr] - zwraca index stanowiska, lub jesli adres nie przypisany do zadnego stanowiska
 * to zwraca index STAN_IDX_DUMMY
 */
static int mapAddr2Idx[255];


static void stanMapAddr2IdxDef(void)
{
	unsigned char k;
	for (k=0; k<255; k++)
	{
		if (k<MAX_STAN)
			mapAddr2Idx[k]=k;
		else
			mapAddr2Idx[k]=DUMMY_STAN_IDX;
	}
}

//bez sprawdzania argumentu - sprawdzanie robi stanSetAddr
unsigned char stanMapAddr2Idx(unsigned char rs485Addr){
	return mapAddr2Idx[rs485Addr];
}

//bez sprawdzania argumentu - sprawdzanie robi stanSetAddr
unsigned char stanMapIdx2Addr(unsigned char idx){
	return mapIdx2Addr[idx];
}

int stanSetAddr(unsigned char idx, unsigned char rs485Addr)
{
	if (idx<MAX_STAN)
	{
		mapAddr2Idx[mapIdx2Addr[idx]] = DUMMY_STAN_IDX;
		mapIdx2Addr[idx] = rs485Addr;
		mapAddr2Idx[rs485Addr] = idx;
		return 0;
	}
	return (-1);
}

unsigned char stanGetNumberOfConfigured(void)
{
	unsigned char k;
	unsigned char ret=0;
	Stanowiska_t *pStan;

	for (k=0; k<MAX_STAN; k++)
	{
		pStan=&Stanowiska[k];
		if(pStan->configured)
			ret++;
	}
	return ret;
}

void stanInit(void)
{
	stanMapAddr2IdxDef();

	/* for now configure only Stan 0 */
	Stanowiska[0].configured = true;
	Stanowiska[0].periodicPollEnabled = true;
}

void Stanowiska_t::setDigOutputOn(DIG_OUT_t wy, bool wrHw, bool hiPrio)
{
	digOut_[wy/8] |= 1 << (wy%8);

	if(wrHw)
		hwWriteAll(hiPrio);
}

void Stanowiska_t::setDigOutputOff(DIG_OUT_t wy, bool wrHw, bool hiPrio)
{
	digOut_[wy/8] &= ~(1 << (wy%8));

	if(wrHw)
		hwWriteAll(hiPrio);
}

void Stanowiska_t::setDispString(char *s, bool wrHw, bool hiPrio)
{
	//tutaj tez w stringu trzeba znalezc kropki i updatowac dispDot_
	unsigned char k;
	for (k=0; k<DISP_SIZE; k++)
		disp_[k] = s[k];

	if (wrHw)
		hwWriteAll(hiPrio);
}

void Stanowiska_t::setDispStringOpts(char *s, DISP_OPTS_t blinkOpts, bool wrHw, bool hiPrio)
{
	setDispString(s, false, hiPrio);
	dispOpts_ = blinkOpts;

	if (wrHw)
		hwWriteAll(hiPrio);
}

void Stanowiska_t::hwWriteAll(bool hiPrio)
{
	//to interfejs do protokolu, dodanie adresu, demultipleksacja z obiektw na interfejs
	//do dodania jeszcze interfejs tutaj ... /dev/ttySx
	protoFuncWriteAll(rs485Addr, digOut_, disp_, dispDot_, dispOpts_, hiPrio);
}

void Stanowiska_t::hwReadAll(unsigned char *we, unsigned char wrzutnik, bool err, unsigned char *pCzytnik, unsigned char czytLen)
{
	//to interfejs z protokolu
	//readDigInput(we);
	//readWrzutnik(wrzutnik);
	//readErr(err);
	//readCzytnik(pCzytnik, czytLen);
}

void Stanowiska_t::hwReadBz(void)
{ }





#if 0
void Stanowiska_t::PeriodicPoll(void)
{
	WriteBz();
}


void Stanowiska_t::TimeoutFromProtocol(void)
{
	//printf ("Stan %2d (addr 0x%2x): TimeoutFromProtoco\n", stanMapAddr2Idx(addr),addr);
}

void Stanowiska_t::RcvErrorFromProtocol(void)
{
	printf ("Stan %2d (addr 0x%2x): RcvErrorFromProtocol\n", stanMapAddr2Idx(addr),addr);
}


void Stanowiska_t::UpdWejscia(unsigned char *we)
{
	printf ("Stan %2d (addr 0x%2x): UpdWejscia\n", stanMapAddr2Idx(addr),addr);
}

void Stanowiska_t::WriteBz()
{
	protoFuncSendRaw(addr, FUNC_WRITE_BZ, NULL, 0, true);
}

void Stanowiska_t::WriteAll(void)
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


