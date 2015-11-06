/*
 * protocol.cpp
 *
 *  Created on: Oct 1, 2015
 *      Author: maciek
 */

/*
 * protocol.cpp
 *
 * Created: 2014-05-14 10:46:24
 *  Author: MaciejR
 */
#include <linux/types.h>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <termios.h> //for serial configuration
#include <linux/serial.h> //for rs485 configuration
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <time.h>

#include "configGlobDemo.h"
#include "configKartDemo.h"

#include "protoDev.h"
#include "protoDevR05.h"
#include "protoDevR06.h"
#include "protoCol.h"
#include "myjnia.h"

/* TODO:
 * dodac FIFO dla USART_COM_TX_LOPRIO_IDX
 * zglaszanie bledow przy jakimkolwiek bledzie apropo komunikacji tzn timeoutow
 * dobrac timeouty i sprawdzic na rs485
 * na demo zostawiam to co jest i dodaje tylko threada rx na innym interfejsie
 */

static protoThreadArgs_t protThreadArgs[MAX_TTY_DEVS] = { PROTO_THREAD_ARGS };
static const char *devNames[MAX_TTY_DEVS] = { DEV_NAMES };
static int fd[MAX_TTY_DEVS];

static usartCom_tx_t usartCom_tx[MAX_TTY_DEVS]={{0, }, };
static usartCom_rx_t usartCom_rx[MAX_TTY_DEVS]={{0, }, };

static usartComCritStats_t usartComCritStats[MAX_TTY_DEVS] = {{0, }, };

static void protoFuncSendRaw(unsigned int ttyNr, unsigned char addr,unsigned char func,unsigned char *buf,unsigned int len,bool hiPrio);
static uint8_t inline usartComCRC (volatile uint8_t* frame, uint16_t len);
static int parseRxProtocol(unsigned int ttyNr, unsigned char *buff, unsigned int length, bool flush);
static void *rxRs485Thread(void *arg);
static void *txRs485Thread(void *arg);
//static void *sendCyclicRs485Thread(void *arg);

funOnTty_t protoGetFuncOnTty(unsigned int ttyNr)
{
	return protThreadArgs[ttyNr].funOnBus;
}

unsigned char protoGetRxAddr (unsigned int ttyNr)
{
	protoDev_t *pDev;
	funOnTty_t funcOnTty = protoGetFuncOnTty(ttyNr);
	switch (funcOnTty)
	{
	case MASTER:
		return USART_COM_MASTER_ADDR;

	case SLAVE:
		pDev=mapDevAddr2Dev(ttyNr, 0, SLAVE);
		return(pDev->getDevRxAdd());

	default:
		printf("protoGetRxAddr-Should not happen\n");
		return (-1);
		break;
	}
}

void protoInit (void)
{
	int res;
	pthread_t rx_thread, tx_thread;
	struct termios newtio;
	unsigned int k;

	for (k=0; k<MAX_TTY_DEVS; k++)
	{
		if (protThreadArgs[k].funOnBus==NONE)
			continue;

		fd[k] = open(devNames[k], O_RDWR | O_NOCTTY );
		if (fd[k] < 0)
		{
			perror(devNames[k]);
			exit(-1);
		}

		bzero(&newtio, sizeof(newtio));
		newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
		newtio.c_iflag = IGNPAR;
		newtio.c_oflag = 0;

		/* set input mode (non-canonical, no echo,...) */
		newtio.c_lflag = 0;

		newtio.c_cc[VTIME]    = 1;     /* inter-character timer unused */
		newtio.c_cc[VMIN]     = 0; //255;   /* blocking read until 255 chars received */

		tcflush(fd[k], TCIFLUSH);
		tcsetattr(fd[k],TCSANOW,&newtio);

#if 0
	/* RS485 ioctls: */
#define TIOCGRS485      0x542E
#define TIOCSRS485      0x542F

		struct serial_rs485 rs485conf;

		/* Enable RS485 mode: */
		rs485conf.flags |= SER_RS485_ENABLED;

		/* Set logical level for RTS pin equal to 1 when sending: */
		rs485conf.flags |= SER_RS485_RTS_ON_SEND;

		/* Set logical level for RTS pin equal to 0 after sending: */
		rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);

		/* Set rts delay before send, if needed: */
		rs485conf.delay_rts_before_send = 10;

		/* Set rts delay after send, if needed: */
		rs485conf.delay_rts_after_send = 10;

		/* Set this flag if you want to receive data even whilst sending data */
		//rs485conf.flags |= SER_RS485_RX_DURING_TX;

		if (ioctl (fd, TIOCSRS485, &rs485conf) < 0) {
			/* Error handling. See errno. */
		}
#endif

		res = pthread_mutex_init(&usartCom_tx[k].mutexUsart, NULL);
		pthread_cond_init(&usartCom_tx[k].condUsartEmpty, NULL);
		pthread_cond_init(&usartCom_tx[k].condUsartFull, NULL);

		res = pthread_mutex_init(&usartCom_tx[k].mutexBuffer, NULL);
		pthread_cond_init(&usartCom_tx[k].condBufferEmpty, NULL);
		pthread_cond_init(&usartCom_tx[k].condBufferFull, NULL);

		if (res != 0) {
			perror("Mutex initialization failed");
			exit(EXIT_FAILURE);
		}

		res = pthread_create(&rx_thread, NULL, rxRs485Thread, (void *)(&protThreadArgs[k]));
		if (res != 0) {
			perror("Thread rx_thread creation failed");
			exit(EXIT_FAILURE);
		}

		res = pthread_create(&tx_thread, NULL, txRs485Thread, (void *)(&protThreadArgs[k]));
		if (res != 0) {
			perror("Thread tx_thread creation failed");
			exit(EXIT_FAILURE);
		}
	}

#if 0
	//hiPrio task for sendnig

	pthread_t tx_hiPrioThread;
	pthread_attr_t     tAttr;
	struct sched_param sSchedParam;

	res  = pthread_attr_init(&tAttr);
	res |= pthread_attr_setstacksize(&tAttr,DEF_THREAD_STACKSIZE);

	/*set additional thread attributes: run thread with system wide priority DEF_TIME_SYNC_PRIO, FIFO+RR scheduling*/
	res |= pthread_attr_setschedpolicy(&tAttr, SCHED_RR);
	res |= pthread_attr_setscope(&tAttr,       PTHREAD_SCOPE_SYSTEM);
	res |= pthread_attr_getschedparam(&tAttr,  &sSchedParam);

	sSchedParam.sched_priority = DEF_TIME_SYNC_PRIO;
	res |= pthread_attr_setschedparam(&tAttr,  &sSchedParam);
	res |= pthread_attr_setinheritsched(&tAttr,PTHREAD_EXPLICIT_SCHED);

	if (res)
	{
		perror("Unable to set thread attributes");
		exit(EXIT_FAILURE);
	}

	res = pthread_create(&tx_hiPrioThread, &tAttr, sendCyclicRs485Thread, NULL);
	if (res != 0) {
		perror("Thread tx_hiPrioThread creation failed");
		exit(EXIT_FAILURE);
	}
#endif

}

void protoFuncWriteAll(unsigned int ttyNr, unsigned char addr, unsigned char *wy, unsigned char *disp, unsigned char dots, unsigned char opts, bool hiPrio)
{
	unsigned char buff[FUNC_WRITE_ALL_DATA_LENGTH];

	buff[0]=wy[0];
	buff[1]=wy[1];
	memcpy(&buff[2], disp, DISP_SIZE);
	buff[6]= (dots | (opts<<4));

	protoFuncSendRaw(ttyNr, addr, FUNC_WRITE_ALL, buff, FUNC_WRITE_ALL_DATA_LENGTH, hiPrio);
}

void protoFuncWriteSSP(unsigned int ttyNr, unsigned char addr, unsigned char *hopData, unsigned char hopLen, unsigned char *nvData, unsigned char nvLen, bool hiPrio)
{
	unsigned char buff[FUNC_WRITE_SSP_DATA_LEN];
	unsigned char *pBuff=buff;

	*pBuff=hopLen; pBuff++;
	if (hopLen)
	{
		memcpy(pBuff, hopData, hopLen);
		pBuff+=hopLen;
	}

	*pBuff=nvLen; pBuff++;
	if (nvLen)
	{
		memcpy(pBuff, nvData, nvLen);
		pBuff+=nvLen;
	}

	protoFuncSendRaw(ttyNr, addr, FUNC_WRITE_SSP, buff, (pBuff-buff), hiPrio);
}

static void protoFuncSendRaw(unsigned int ttyNr, unsigned char addr,unsigned char func,unsigned char *buf,unsigned int len,bool hiPrio)
{
	unsigned char* pTxFrame;
	unsigned int idx=USART_COM_TX_LOPRIO_IDX;

	if (hiPrio)
		idx=USART_COM_TX_HIPRIO_IDX;

	/* producer */
	pthread_mutex_lock(&usartCom_tx[ttyNr].mutexBuffer);
	if(hiPrio)
	{
		while (usartCom_tx[ttyNr].countBufHiPrio == 1)
			pthread_cond_wait(&usartCom_tx[ttyNr].condBufferEmpty, &usartCom_tx[ttyNr].mutexBuffer);
		usartCom_tx[ttyNr].countBufHiPrio=1;
	}
	else
	{
		while (usartCom_tx[ttyNr].countBufNormal == 1)
			pthread_cond_wait(&usartCom_tx[ttyNr].condBufferEmpty, &usartCom_tx[ttyNr].mutexBuffer);
		usartCom_tx[ttyNr].countBufNormal=1;
	}

	pTxFrame = usartCom_tx[ttyNr].usartComTxUnionFrame[idx].usartComTxFrame;

	pTxFrame[0]=USART_COM_STARTBYTE;
	pTxFrame[1]=addr;
	pTxFrame[2]=func;
	if(len)
		memcpy((void*)&pTxFrame[3], buf, len);

	pTxFrame[3+len]=usartComCRC(pTxFrame, (len + 3));
	pTxFrame[4+len]=USART_COM_STOPTBYTE;
	usartCom_tx[ttyNr].usartComTxDataLength[idx]=(len + 5);

	pthread_cond_signal(&usartCom_tx[ttyNr].condBufferFull);
	pthread_mutex_unlock(&usartCom_tx[ttyNr].mutexBuffer);
}

static void protoSignalTimeout(unsigned int ttyNr)
{
	protoDev_t *pDev=mapDevAddr2Dev(ttyNr, usartCom_tx[ttyNr].addrReq, protThreadArgs[ttyNr].funOnBus);
	pDev->TimeoutFromProtocol();
}

static void protoSignalReceiveError(unsigned int ttyNr)
{
	protoDev_t *pDev=mapDevAddr2Dev(ttyNr, usartCom_tx[ttyNr].addrReq, protThreadArgs[ttyNr].funOnBus);
	pDev->RcvErrorFromProtocol();
}

static void protoFuncReadAll(unsigned int ttyNr, usartComRxUnionFrame_t readAllStr)
{
	protoDev_t *pDev=mapDevAddr2Dev(ttyNr, usartCom_tx[ttyNr].addrReq, protThreadArgs[ttyNr].funOnBus);
	unsigned char we[2], wrzutnik, czytLen;
	bool err;
	unsigned char *pCzytnik;

	we[0]=readAllStr.usartComReadAllStr.we[0];
	we[1]=readAllStr.usartComReadAllStr.we[1] & 0x1;

	wrzutnik=(readAllStr.usartComReadAllStr.we[1] & 0x7e) >> 1;

	err=((readAllStr.usartComReadAllStr.we[1]&0x1)?true:false);

	pCzytnik=readAllStr.usartComReadAllStr.czytData;
	czytLen=readAllStr.usartComReadAllStr.czytLen;

	pDev->hwReadAll(we, wrzutnik, err, pCzytnik, czytLen);
}

static void protoFuncReadBz(unsigned int ttyNr, usartComRxUnionFrame_t readAllStr)
{
	protoDev_t *pDev=mapDevAddr2Dev(ttyNr, usartCom_tx[ttyNr].addrReq, protThreadArgs[ttyNr].funOnBus);
	pDev->hwReadBz();
}

static void protoStatsIncr(unsigned int ttyNr, protoStatsType_t statsType)
{
	protoDev_t *pDev=mapDevAddr2Dev(ttyNr, usartCom_tx[ttyNr].addrReq, protThreadArgs[ttyNr].funOnBus);
	pDev->protoStats[statsType]++;
}

int protoGetGlobalStats(char* str)
{
#define STATS(__stat__,__str__) ret=sprintf(p, "%s %u\n",__str__,__stat__); p=p+ret;

	char *p=str;
	unsigned int ret, k;

	for (k=0; k<MAX_TTY_DEVS; k++)
	{
		if (protThreadArgs[k].funOnBus==NONE)
			continue;

		ret=sprintf(p, "Protocol global stats [dev=%s]:\n", devNames[protThreadArgs[k].ttyNr]); p=p+ret;
		STATS(usartComCritStats[k].usartComPartialFrames,         "PartialFrames                    :")
		STATS(usartComCritStats[k].usartComFramesCompleted,       "FramesCompleted                  :")
		STATS(usartComCritStats[k].usartComFramesToMe,            "FramesToMe                       :")
		STATS(usartComCritStats[k].usartComGoodFramesToMe,        "GoodFramesToMe                   :")
		STATS(usartComCritStats[k].usartComFlushesNr,             "FlushesNr                        :")
		STATS(usartComCritStats[k].usartComFramesBroadcast,       "FramesBroadcast(default=0)       :")
		STATS(usartComCritStats[k].usartComFramesToOthers,        "FramesToOthers(default=0)        :")
		STATS(usartComCritStats[k].usartComFramesFuncNotSupp,     "FramesFuncNotSupp(default=0)     :")
		STATS(usartComCritStats[k].usartComFramesWaitingForStart, "FramesWaitingForStart(default=0) :")
		STATS(usartComCritStats[k].usartComFramesWaitingForStop,  "FramesWaitingForStop(default=0)  :")
		STATS(usartComCritStats[k].usartComFramesBedCrc,          "FramesBedCrc(default=0)          :")
		STATS(usartComCritStats[k].usartComBufsWithoutContext,    "FramesWithoutContext(default=0)  :")
		STATS(usartComCritStats[k].usartComBytesAfterFinish,      "BytesAfterFinish(default=0)      :")
		STATS(usartComCritStats[k].usartComShouldNotHappen,       "ShouldNotHappen(default=0)       :")
		ret=sprintf(p, "\n"); p=p+ret;
		ret=sprintf(p, "Linux errors:\n"); p=p+ret;
		STATS(usartComCritStats[k].readLinuxErr,                  "readLinuxErr(default=0)          :")
		STATS(usartComCritStats[k].selectLinuxErr,                "selectLinuxErr(default=0)        :")
		STATS(usartComCritStats[k].writeLinuxErr,                 "writeLinuxErr(default=0)         :")
		STATS(usartComCritStats[k].txRs485ThreadErr,              "txRs485ThreadErr(default=0)      :")
	}
	return (p - str);
#undef STATS
}

static uint8_t inline usartComCRC (volatile uint8_t* frame, uint16_t len)
{
	uint8_t ret = 0xff;
	uint16_t k=0;

	for(k=0; k<len; k++)
		ret ^= frame[k];

	return ret;
}

/*
 * return:
 * 0  - completed frame received/flush
 * 1  - partial frame received
 * -1 - error during receiving
 * -2 - frames without context
 *
 * if(flush==true) to zwraca rxState
 *
 */
static int parseRxProtocol(unsigned int ttyNr, unsigned char *buff, unsigned int length, bool flush)
{
	unsigned int l;
	uint8_t usartComRxByte;
	USART_COM_RX_STATE_t flushRxState;

	if (flush)
	{
		flushRxState=usartCom_rx[ttyNr].usartCom_rx_state;
		usartComCritStats[ttyNr].usartComFlushesNr++;
		usartCom_rx[ttyNr].usartCom_rx_state = WAITING_FOR_START_BYTE;
		usartCom_rx[ttyNr].usartComDataLength=0;
		usartCom_rx[ttyNr].usartComFunction = FUNC_INVALID;
		usartCom_rx[ttyNr].czytLen=0;
		return ((int) flushRxState);
	}

	for (l=0; l<length; l++)
	{
		usartComRxByte = (unsigned char) buff[l];

		switch (usartCom_rx[ttyNr].usartCom_rx_state)
		{
		case WAITING_FOR_START_BYTE:
			if (usartComRxByte == USART_COM_STARTBYTE)
				usartCom_rx[ttyNr].usartCom_rx_state = WAITING_FOR_ADDR;
			else
				usartComCritStats[ttyNr].usartComFramesWaitingForStart++;
			break;

		case WAITING_FOR_ADDR:
			if (usartComRxByte == protoGetRxAddr (ttyNr))
			{
				usartComCritStats[ttyNr].usartComFramesToMe++;
				usartCom_rx[ttyNr].usartCom_rx_state = WAITING_FOR_FUNC_CODE;
			}
			else if (usartComRxByte == USART_COM_BROADCAST_ADDR)
			{
				usartComCritStats[ttyNr].usartComFramesBroadcast++;
				goto ERROR;
			}
			else
			{
				usartComCritStats[ttyNr].usartComFramesToOthers++;
				goto ERROR;
			}
			break;

		case WAITING_FOR_FUNC_CODE:
			switch (usartComRxByte)
			{
			case FUNC_READ_ALL:
				usartCom_rx[ttyNr].usartCom_rx_state = WAITING_FOR_READ_DATA;
				usartCom_rx[ttyNr].usartComFunction = FUNC_READ_ALL;
				break;

			case FUNC_READ_BZ:
				usartCom_rx[ttyNr].usartCom_rx_state = WAITING_FOR_CRC;
				usartCom_rx[ttyNr].usartComFunction = FUNC_READ_BZ;
				break;

				// write from master... should not be visible if I'm not seeing what I'm transmitting
			case FUNC_WRITE_ALL:
			case FUNC_WRITE_BZ:
			default:
				usartComCritStats[ttyNr].usartComFramesFuncNotSupp++;
				goto ERROR;
			}
			break;

			case WAITING_FOR_READ_DATA: // START(1)|ADD(1)|FUNC(1)|WE(2)|CZYT_LEN(1)|CZYT_DATA(0..256)|CRC(1)|STOP(1)
				switch (usartCom_rx[ttyNr].usartComFunction)
				{
				case FUNC_READ_ALL:
					if (usartCom_rx[ttyNr].usartComDataLength < 5)
						break;

					if (usartCom_rx[ttyNr].usartComDataLength == 5)
						usartCom_rx[ttyNr].czytLen=usartComRxByte;

					if (usartCom_rx[ttyNr].usartComDataLength >= (FUNC_READ_ALL_MIN_LENGTH+usartCom_rx[ttyNr].czytLen-3))
						usartCom_rx[ttyNr].usartCom_rx_state=WAITING_FOR_CRC;
					break;
				default:
					usartComCritStats[ttyNr].usartComShouldNotHappen++;
					goto ERROR;
				}
				break;

				case WAITING_FOR_CRC:
					if (usartComRxByte==usartComCRC(usartCom_rx[ttyNr].usartComRxUnionFrame.usartComRxFrame, usartCom_rx[ttyNr].usartComDataLength))
						usartCom_rx[ttyNr].usartCom_rx_state=WAITING_FOR_STOP_BYTE;
					else
					{
						usartComCritStats[ttyNr].usartComFramesBedCrc++;
						usartCom_rx[ttyNr].usartCom_rx_state=WAITING_FOR_STOP_BYTE_ERROR_CRC;
					}
					break;

				case WAITING_FOR_STOP_BYTE_ERROR_CRC:
					if (usartComRxByte == USART_COM_STOPTBYTE)
						protoStatsIncr(ttyNr, RECV_CRC);
					else
						usartComCritStats[ttyNr].usartComFramesWaitingForStop++;

					goto ERROR;

				case WAITING_FOR_STOP_BYTE:
					if (usartComRxByte == USART_COM_STOPTBYTE)
					{
						usartCom_rx[ttyNr].usartComRxUnionFrame.usartComRxFrame[usartCom_rx[ttyNr].usartComDataLength]=usartComRxByte;
						usartCom_rx[ttyNr].usartComDataLength++;
						usartComCritStats[ttyNr].usartComGoodFramesToMe++;

#if 0
						printf ("RCVED: ");
						int k;
						for (k=0; k< usartCom_rx[ttyNr].usartComDataLength; k++)
							printf("%2x ", usartCom_rx[ttyNr].usartComRxUnionFrame.usartComRxFrame[k]);
						printf ("\n");
#endif

						switch (usartCom_rx[ttyNr].usartComFunction)
						{
						case FUNC_READ_ALL:
							if (usartCom_rx[ttyNr].usartComRxUnionFrame.usartComReadAllStr.we[1] & 0x1)
								protoStatsIncr(ttyNr, RECV_STANTIMEOUT);

							protoFuncReadAll(ttyNr, usartCom_rx[ttyNr].usartComRxUnionFrame);
							break;

						case FUNC_READ_BZ:
							protoFuncReadBz(ttyNr, usartCom_rx[ttyNr].usartComRxUnionFrame);
							break;

						default:
							usartComCritStats[ttyNr].usartComShouldNotHappen++;
							goto ERROR;
						}
						usartCom_rx[ttyNr].usartCom_rx_state = FINISHED;
					}
					else
					{
						usartComCritStats[ttyNr].usartComFramesWaitingForStop++;
						goto ERROR;
					}
					break;

				case FINISHED:
					//bajty za bajtem STOPu sa ignorowane, konczymy return 0
					usartComCritStats[ttyNr].usartComBytesAfterFinish++;
					break;

				default:
					usartComCritStats[ttyNr].usartComShouldNotHappen++;
					goto ERROR;

		} //END of switch (usartCom_rx.usartCom_rx_state)

		if ((usartCom_rx[ttyNr].usartCom_rx_state != WAITING_FOR_START_BYTE) &&
				(usartCom_rx[ttyNr].usartCom_rx_state != FINISHED))
		{
			usartCom_rx[ttyNr].usartComRxUnionFrame.usartComRxFrame[usartCom_rx[ttyNr].usartComDataLength]=usartComRxByte;
			usartCom_rx[ttyNr].usartComDataLength++;
		}

	} //END of for (l=0; l<length; l++)

	//przychodza bajty, a my czekamy na bajt startu, bez tego po odebraniu 1 jakiegos bajtu zwracalibysmy "partial frame"
	if (usartCom_rx[ttyNr].usartCom_rx_state == WAITING_FOR_START_BYTE)
	{
		usartComCritStats[ttyNr].usartComBufsWithoutContext++;
		return -2;
	}

	//niedokonczone ramki
	if (usartCom_rx[ttyNr].usartCom_rx_state != FINISHED)
	{
		usartComCritStats[ttyNr].usartComPartialFrames++;
		return 1;
	}
	else
	{
		usartComCritStats[ttyNr].usartComFramesCompleted++;
		usartCom_rx[ttyNr].usartCom_rx_state = WAITING_FOR_START_BYTE;
		usartCom_rx[ttyNr].usartComDataLength=0;
		usartCom_rx[ttyNr].usartComFunction = FUNC_INVALID;
		usartCom_rx[ttyNr].czytLen=0;
		return 0;
	}

	ERROR:
	usartCom_rx[ttyNr].usartCom_rx_state = WAITING_FOR_START_BYTE;
	usartCom_rx[ttyNr].usartComDataLength=0;
	usartCom_rx[ttyNr].usartComFunction = FUNC_INVALID;
	usartCom_rx[ttyNr].czytLen=0;
	return -1;
}

#if 0
static void *sendCyclicRs485Thread(void *arg)
{
	unsigned char nrOfConfiguredStan;
	struct timeval timeout;
	unsigned char idx=0;
	protoDev_t *pStan;
	unsigned char addr;

	while (1)
	{
		nrOfConfiguredStan=getNumberOfConfigured();

		for(idx=0; idx<=GOOD_R0X_IDX; idx++)
		{
			pStan=mapDevAddr2Dev(0, idx);

			//kazde stanowisko dostanie co MAX_POLL_INTERVAL_US, niezaleznie od ilosci stanowisk
			timeout.tv_sec = 0;
			timeout.tv_usec = MAX_POLL_INTERVAL_US/nrOfConfiguredStan;

			select(0,NULL,NULL,NULL,&timeout);

			//pStan->PeriodicPoll();
		}
	}
	return NULL;
}
#endif

static void *rxRs485Thread(void *arg)
{
	int retSelect, retRead, parseRet;
	unsigned char buff[255];
	fd_set set;
	struct timeval timeout;
	int retFlush;
	protoThreadArgs_t *pArgs = (protoThreadArgs_t *)(arg);

	while(1)
	{
		pthread_mutex_lock(&usartCom_tx[pArgs->ttyNr].mutexUsart);
		while (usartCom_tx[pArgs->ttyNr].countUart == 0)
			pthread_cond_wait(&usartCom_tx[pArgs->ttyNr].condUsartFull, &usartCom_tx[pArgs->ttyNr].mutexUsart);

		//read
		while (1)
		{
			FD_ZERO(&set);
			FD_SET(fd[pArgs->ttyNr], &set);
			timeout.tv_sec = 0;
			timeout.tv_usec = 500000;

			retSelect=select(fd[pArgs->ttyNr] + 1, &set, NULL, NULL, &timeout);
			if(retSelect < 0)
			{
				usartComCritStats[pArgs->ttyNr].selectLinuxErr++;
				perror("select");
				continue;
			}
			else if(retSelect == 0)
			{
				//Timeout
				protoSignalTimeout(pArgs->ttyNr);

				protoStatsIncr(pArgs->ttyNr, RECV_TIMEOUTS);

				//only clean global variables in parseRxProtocol, and return rx_state
				retFlush=parseRxProtocol(pArgs->ttyNr, NULL, 0, true);
				printf("Select timeout: addr=0x%x,rx_state=%u\n", usartCom_tx[pArgs->ttyNr].addrReq, retFlush);
				break;
			}
			else
			{
				//sprawdzic czy read nie zwroci bledu
				retRead=read(fd[pArgs->ttyNr], buff, sizeof(buff));
				if(retRead <= 0)
				{
					perror("read");
					usartComCritStats[pArgs->ttyNr].readLinuxErr++;
					continue;
				}

				parseRet=parseRxProtocol(pArgs->ttyNr, buff, retRead, false);
				if (!parseRet)
				{
					protoStatsIncr(pArgs->ttyNr, RECV_OK);
					break;
				}
				else if (parseRet<0)
				{
					//ReceiveError
					protoSignalReceiveError(pArgs->ttyNr);

					protoStatsIncr(pArgs->ttyNr, RECV_ERR);

					usleep(20000); //17ms to jest 255 bajtow@115200
					tcflush(fd[pArgs->ttyNr], TCIFLUSH);

					//only clean global variables in parseRxProtocol, and return rx_state
					retFlush=parseRxProtocol(pArgs->ttyNr, NULL, 0, true);
					printf("parseRxError: addr=0x%x,rx_state=%u\n", usartCom_tx[pArgs->ttyNr].addrReq, retFlush);
					break;
				}
				else
					protoStatsIncr(pArgs->ttyNr, RECV_PARTIAL);
			}
		}
		usartCom_tx[pArgs->ttyNr].countUart=0;
		pthread_cond_signal(&usartCom_tx[pArgs->ttyNr].condUsartEmpty);
		pthread_mutex_unlock(&usartCom_tx[pArgs->ttyNr].mutexUsart);
	}
	return NULL;
}

static void *txRs485Thread(void *arg)
{
	int ret;
	unsigned int written;
	protoThreadArgs_t *pArgs = (protoThreadArgs_t *)(arg);

	while(1)
	{
		written=0;

		/* czekamy na wolna szyne rs485 (producer na szyne 485) */
		pthread_mutex_lock(&usartCom_tx[pArgs->ttyNr].mutexUsart);
		while (usartCom_tx[pArgs->ttyNr].countUart == 1)
			pthread_cond_wait(&usartCom_tx[pArgs->ttyNr].condUsartEmpty, &usartCom_tx[pArgs->ttyNr].mutexUsart);

		/* czekamy az sie pojawia dane (consumer z bufora) */
		pthread_mutex_lock(&usartCom_tx[pArgs->ttyNr].mutexBuffer);
		while ((usartCom_tx[pArgs->ttyNr].countBufNormal == 0) && (usartCom_tx[pArgs->ttyNr].countBufHiPrio == 0))
			pthread_cond_wait(&usartCom_tx[pArgs->ttyNr].condBufferFull, &usartCom_tx[pArgs->ttyNr].mutexBuffer);

		if (usartCom_tx[pArgs->ttyNr].countBufHiPrio == 1)
		{
			usartCom_tx[pArgs->ttyNr].addrReq=usartCom_tx[pArgs->ttyNr].usartComTxUnionFrame[USART_COM_TX_HIPRIO_IDX].usartComTxFrame[1];
			usartCom_tx[pArgs->ttyNr].sendHiPrio=true;
			do
			{
				ret=write(fd[pArgs->ttyNr], (void*)(((unsigned char*)(usartCom_tx[pArgs->ttyNr].usartComTxUnionFrame[USART_COM_TX_HIPRIO_IDX].usartComTxFrame)) + written), (usartCom_tx[pArgs->ttyNr].usartComTxDataLength[USART_COM_TX_HIPRIO_IDX]-written));
				if (ret <= 0)
				{
					perror("write");
					usartComCritStats[pArgs->ttyNr].writeLinuxErr++;
					break;
				}
				else
					written=written+ret;

			}
			while (written!=usartCom_tx[pArgs->ttyNr].usartComTxDataLength[USART_COM_TX_HIPRIO_IDX]);

			if (ret > 0)
				protoStatsIncr(pArgs->ttyNr, SEND_HIPRIO_FRAMES);

			usartCom_tx[pArgs->ttyNr].countBufHiPrio=0;
		}
		else if(usartCom_tx[pArgs->ttyNr].countBufNormal == 1)
		{
			usartCom_tx[pArgs->ttyNr].addrReq=usartCom_tx[pArgs->ttyNr].usartComTxUnionFrame[USART_COM_TX_LOPRIO_IDX].usartComTxFrame[1];
			usartCom_tx[pArgs->ttyNr].sendHiPrio=false;
			do
			{
				ret=write(fd[pArgs->ttyNr], (void*)(((unsigned char*)(usartCom_tx[pArgs->ttyNr].usartComTxUnionFrame[USART_COM_TX_LOPRIO_IDX].usartComTxFrame)) + written), (usartCom_tx[pArgs->ttyNr].usartComTxDataLength[USART_COM_TX_LOPRIO_IDX]-written));
				if (ret <= 0)
				{
					perror("write");
					usartComCritStats[pArgs->ttyNr].writeLinuxErr++;
					break;
				}
				else
					written=written+ret;
			}
			while (written!=usartCom_tx[pArgs->ttyNr].usartComTxDataLength[USART_COM_TX_LOPRIO_IDX]);

			if (ret > 0)
				protoStatsIncr(pArgs->ttyNr, SEND_LOPRIO_FRAMES);

			usartCom_tx[pArgs->ttyNr].countBufNormal=0;
		}
		else
		{
			usartComCritStats[pArgs->ttyNr].txRs485ThreadErr++;
			printf("Should not happen I think\n");
		}

		tcdrain(fd[pArgs->ttyNr]);

#if 0
		unsigned int k;
		for (k=0; k<usartComTxLength; k++)
			printf ("x%x ", usartComTxUnionFrame[pArgs->ttyNr].usartComTxFrame[k]);
		printf("\n");
#endif

		pthread_cond_signal(&usartCom_tx[pArgs->ttyNr].condBufferEmpty);
		pthread_mutex_unlock(&usartCom_tx[pArgs->ttyNr].mutexBuffer);

		usartCom_tx[pArgs->ttyNr].countUart=1;
		pthread_cond_signal(&usartCom_tx[pArgs->ttyNr].condUsartFull);
		pthread_mutex_unlock(&usartCom_tx[pArgs->ttyNr].mutexUsart);
	}
	return NULL;
}


