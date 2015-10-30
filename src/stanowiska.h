/*
 * stanowiska.h
 *
 *  Created on: Oct 5, 2015
 *      Author: maciek
 */

#ifndef STANOWISKA_H_
#define STANOWISKA_H_

#define DIG_OUTS_MAX	16 //podzielne przez 8
#define DISP_SIZE		4

typedef enum
{
	DIG_OUT_L1 = 0,
	DIG_OUT_L2,
	DIG_OUT_L3,
	DIG_OUT_L4,
	DIG_OUT_L5,
	DIG_OUT_L6,
	DIG_OUT_L7,
	DIG_OUT_L8,
	DIG_OUT_L9,
	DIG_OUT_G,
	DIG_OUT_S,
	DIG_OUT_D,
	DIG_OUT_MAX
} DIG_OUT_t;

typedef enum
{
	DISP_BLINK_NO = 0,
	DISP_BLINK_SLOW,
	DISP_BLINK_FAST,
	DISP_BLINK_VFAST,
} DISP_OPTS_t;


class Stanowiska_t {
public:
	bool configured;
	bool periodicPollEnabled;
	unsigned char rs485Addr;
	char *dev; // /dev/ttySx

private:
	unsigned char digOut_[DIG_OUTS_MAX/8];
	unsigned char disp_[DISP_SIZE];
	unsigned char dispDot_;
	DISP_OPTS_t dispOpts_;

public:
	void setDigOutputOn(DIG_OUT_t digOut_, bool wrHw, bool hiPrio);
	void setDigOutputOff(DIG_OUT_t digOut_, bool wrHw, bool hiPrio);

	void setDispString(char *s, bool wrHw, bool hiPrio);
	void setDispStringOpts(char *s, DISP_OPTS_t blinkOpts, bool wrHw, bool hiPrio);


/*********** Interfejs stanowiska <-> protokol **********/
private:
	void hwWriteAll(bool hiPrio);
public:
	//interfejs z protokolu
	void hwReadAll(unsigned char *we, unsigned char wrzutnik, bool err, unsigned char *pCzytnik, unsigned char czytLen);
	void hwReadBz(void);
/********************************************************/



	/*
	void PeriodicPoll(void);
	void WriteDisp(char *display, bool writeHw);
	void UpdWejscia(unsigned char *we);
	void WriteBz(void);
	void WriteAll(void);
	void TimeoutFromProtocol(void);
	void RcvErrorFromProtocol(void);
*/

};



extern Stanowiska_t Stanowiska[MAX_STAN+1];

void stanInit(void);
unsigned char stanMapAddr2Idx(unsigned char rs485Addr);
unsigned char stanMapIdx2Addr(unsigned char idx);
int stanSetAddr(unsigned char idx, unsigned char rs485Addr);

unsigned char stanGetNumberOfConfigured(void);

#endif /* STANOWISKA_H_ */
