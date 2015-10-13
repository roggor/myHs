/*
 * stanowiska.h
 *
 *  Created on: Oct 5, 2015
 *      Author: maciek
 */

#ifndef STANOWISKA_H_
#define STANOWISKA_H_

#define WRITE_ALL_WY_NR 9
#define WRITE_ALL_MAX_DATA 7

typedef enum
{
	DISP_BLINK_NO = 0,
	DISP_BLINK_SLOW,
	DISP_BLINK_FAST,
	DISP_BLINK_VFAST,
} DISP_OPTS_t;

class Param_t {
public:
	USART_COM_RX_FUNC_t protoFunc;
	unsigned int offBits;
	unsigned int sizeBits;
};

//tutaj beda wyjsia 1..9, grzalka, silownik, wrzutnik dissable
class Out_t : public Param_t {
public:
	bool data;
};

//tutaj wyswietlacz
class Disp_t : public Param_t {
public:
	unsigned char data[4];
};

//tutaj kropki wyswietlacza
class DispDots_t : public Param_t {
public:
	bool data[4];
};

//tutaj opcje wyswietlacza
class DispBlink_t : public Param_t {
public:
	DISP_OPTS_t data;
};

class Stanowiska_t {
public:
	bool enabled;
	unsigned char addr;

	class Out_t wy[WRITE_ALL_WY_NR];

	class Out_t grzalka;
	class Out_t silownik;
	class Out_t wrztDis;

	class Disp_t disp;
	class DispDots_t dispDots;
	class DispBlink_t dispBlink;

public:
	void UpdWejscia(unsigned char *we);
	void WriteAll(void);
	void TimeoutFromProtocol(void);
};

extern Stanowiska_t Stanowiska[MAX_STAN+1];

void stanInit(void);
unsigned char stanMapAddr2Idx(unsigned char addr);
unsigned char stanMapIdx2Addr(unsigned char idx);
int stanSetAddr(unsigned char idx, unsigned char addr);

unsigned char stanGetNumberOfEnabled(void);

#endif /* STANOWISKA_H_ */
