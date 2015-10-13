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

#include "globalConfig.h"
#include "protocol.h"
#include "stanowiska.h"

/* TODO:
 * dodac FIFO dla USART_COM_TX_LOPRIO_IDX
 * zglaszanie bledow przy jakimkolwiek bledzie apropo komunikacji tzn timeoutow
 *
 */

int fd;
usartCom_tx_t usartCom_tx={0, };
usartComCritStats_t usartComCritStats = {0, };
static protoStatsPerStan_t protoStatsPerStan[MAX_STAN] = {0, };

static uint8_t inline usartComCRC (volatile uint8_t* frame, uint16_t len);
static int parseRxProtocol(unsigned char *buff, unsigned int length, bool flush);
static void *rxRs485Thread(void *arg);
static void *txRs485Thread(void *arg);
static void *txHiPrioRs485Thread(void *arg);

void protoInit (void)
{
	int res;
	pthread_t rx_thread, tx_thread, tx_hiPrioThread;
	pthread_attr_t     tAttr;
	struct sched_param sSchedParam;

#define BAUDRATE B9600
#define MODEMDEVICE "/dev/ttyS0"
//#define MODEMDEVICE "/dev/ttyUSB0"

	struct termios newtio;

	fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );
	if (fd <0)
	{
		perror(MODEMDEVICE);
		exit(-1);
	}

	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;

	/* set input mode (non-canonical, no echo,...) */
	newtio.c_lflag = 0;

	newtio.c_cc[VTIME]    = 1;     /* inter-character timer unused */
	newtio.c_cc[VMIN]     = 0; //255;   /* blocking read until 255 chars received */

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd,TCSANOW,&newtio);


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

	res = pthread_mutex_init(&usartCom_tx.mutexUartReadWrite, NULL);
	pthread_cond_init(&usartCom_tx.emptyUart, NULL);
	pthread_cond_init(&usartCom_tx.fillUart, NULL);

	res = pthread_mutex_init(&usartCom_tx.mutexBufReadWrite, NULL);
	pthread_cond_init(&usartCom_tx.emptyBuf, NULL);
	pthread_cond_init(&usartCom_tx.fillBuf, NULL);

	if (res != 0) {
		perror("Mutex initialization failed");
		exit(EXIT_FAILURE);
	}

	res = pthread_create(&rx_thread, NULL, rxRs485Thread, NULL);
	if (res != 0) {
		perror("Thread rx_thread creation failed");
		exit(EXIT_FAILURE);
	}

	res = pthread_create(&tx_thread, NULL, txRs485Thread, NULL);
	if (res != 0) {
		perror("Thread tx_thread creation failed");
		exit(EXIT_FAILURE);
	}

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

	res = pthread_create(&tx_hiPrioThread, &tAttr, txHiPrioRs485Thread, NULL);
	if (res != 0) {
		perror("Thread tx_hiPrioThread creation failed");
		exit(EXIT_FAILURE);
	}
}

void protoFuncSendRaw(unsigned char addr,unsigned char func,unsigned char *buf,unsigned int len,bool hiPrio)
{
	unsigned char* pTxFrame;
	unsigned int idx=USART_COM_TX_LOPRIO_IDX;

	if (hiPrio)
		idx=USART_COM_TX_HIPRIO_IDX;

	/* producer */
	pthread_mutex_lock(&usartCom_tx.mutexBufReadWrite);
	if(hiPrio)
	{
		while (usartCom_tx.countBufHiPrio == 1)
			pthread_cond_wait(&usartCom_tx.emptyBuf, &usartCom_tx.mutexBufReadWrite);
		usartCom_tx.countBufHiPrio=1;
	}
	else
	{
		while (usartCom_tx.countBufNormal == 1)
			pthread_cond_wait(&usartCom_tx.emptyBuf, &usartCom_tx.mutexBufReadWrite);
		usartCom_tx.countBufNormal=1;
	}

	pTxFrame = usartCom_tx.usartComTxUnionFrame[idx].usartComTxFrame;

	pTxFrame[0]=USART_COM_STARTBYTE;
	pTxFrame[1]=addr;
	pTxFrame[2]=func;
	if(len)
		memcpy((void*)&pTxFrame[3], buf, len);

	pTxFrame[3+len]=usartComCRC(pTxFrame, (len + 3));
	pTxFrame[4+len]=USART_COM_STOPTBYTE;
	usartCom_tx.usartComTxDataLength[idx]=(len + 5);

	pthread_cond_signal(&usartCom_tx.fillBuf);
	pthread_mutex_unlock(&usartCom_tx.mutexBufReadWrite);
}

#if 0
void protoFuncWriteAll(unsigned char addr,unsigned char wy[2],unsigned char disp[4],unsigned char opts)
{
	unsigned char* pTxFrame;

	/* producer */
	pthread_mutex_lock(&usartCom_tx.mutexBufReadWrite);
	while (usartCom_tx.countBufNormal == 1)
		pthread_cond_wait(&usartCom_tx.emptyBuf, &usartCom_tx.mutexBufReadWrite);

	pTxFrame = usartCom_tx.usartComTxUnionFrame[USART_COM_TX_LOPRIO_IDX].usartComTxFrame;
	pTxFrame[0]=USART_COM_STARTBYTE;
	pTxFrame[1]=addr;
	pTxFrame[2]=FUNC_WRITE_ALL;
	memcpy(&pTxFrame[3], wy, 2);
	memcpy(&pTxFrame[5], disp, 4);
	pTxFrame[9]=opts;

	pTxFrame[10]=usartComCRC(pTxFrame, (10));
	pTxFrame[11]=USART_COM_STOPTBYTE;
	usartCom_tx.usartComTxDataLength[USART_COM_TX_LOPRIO_IDX]=12;

	//buffer filled
	usartCom_tx.countBufNormal=1;

	pthread_cond_signal(&usartCom_tx.fillBuf);
	pthread_mutex_unlock(&usartCom_tx.mutexBufReadWrite);
}
#endif

int protoGetGlobalStats(char* str)
{
#define STATS(__stat__,__str__) ret=sprintf(p, "%s %u\n",__str__,__stat__); p=p+ret;

	char *p=str;
	unsigned int ret;
	ret=sprintf(p, "Protocol global stats:\n"); p=p+ret;
	STATS(usartComCritStats.usartComPartialFrames,         "PartialFrames                    :")
	STATS(usartComCritStats.usartComFramesCompleted,       "FramesCompleted                  :")
	STATS(usartComCritStats.usartComFramesToMe,            "FramesToMe                       :")
	STATS(usartComCritStats.usartComGoodFramesToMe,        "GoodFramesToMe                   :")
	STATS(usartComCritStats.usartComFlushesNr,             "FlushesNr                        :")
	STATS(usartComCritStats.usartComFramesBroadcast,       "FramesBroadcast(default=0)       :")
	STATS(usartComCritStats.usartComFramesToOthers,        "FramesToOthers(default=0)        :")
	STATS(usartComCritStats.usartComFramesFuncNotSupp,     "FramesFuncNotSupp(default=0)     :")
	STATS(usartComCritStats.usartComFramesWaitingForStart, "FramesWaitingForStart(default=0) :")
	STATS(usartComCritStats.usartComFramesWaitingForStop,  "FramesWaitingForStop(default=0)  :")
	STATS(usartComCritStats.usartComFramesBedCrc,          "FramesBedCrc(default=0)          :")
	STATS(usartComCritStats.usartComBufsWithoutContext,    "FramesWithoutContext(default=0)  :")
	STATS(usartComCritStats.usartComBytesAfterFinish,      "BytesAfterFinish(default=0)      :")
	STATS(usartComCritStats.usartComShouldNotHappen,       "ShouldNotHappen(default=0)       :")
	ret=sprintf(p, "\n"); p=p+ret;
	ret=sprintf(p, "Linux errors:\n"); p=p+ret;
	STATS(usartComCritStats.readLinuxErr,                  "readLinuxErr(default=0)          :")
	STATS(usartComCritStats.selectLinuxErr,                "selectLinuxErr(default=0)        :")
	STATS(usartComCritStats.writeLinuxErr,                 "writeLinuxErr(default=0)         :")
	STATS(usartComCritStats.txRs485ThreadErr,              "txRs485ThreadErr(default=0)      :")

	return (p - str);

#undef STATS(__stat__,__str__)
}

int protoGetPerStanStats(char* str, unsigned int stan)
{
#define STATS(__stat__,__str__) ret=sprintf(p, "%s %u\n",__str__,__stat__); p=p+ret;

	char *p=str;
	unsigned int ret;
	ret=sprintf(p, "Stan nr %d stats:\n", stan); p=p+ret;
	STATS(protoStatsPerStan[stan].sendHiPrioFrames,        "sendHiPrioFrames                 :")
	STATS(protoStatsPerStan[stan].sendLoPrioFrames,        "sendLoPrioFrames                 :")
	STATS(protoStatsPerStan[stan].recvHiPrioPartial,       "recvHiPrioPartial                :")
	STATS(protoStatsPerStan[stan].recvLoPrioPartial,       "recvLoPrioPartial                :")
	STATS(protoStatsPerStan[stan].recvHiPrioOk,            "recvHiPrioOk                     :")
	STATS(protoStatsPerStan[stan].recvLoPrioOk,            "recvLoPrioOk                     :")
	STATS(protoStatsPerStan[stan].recvHiPrioTimeouts,      "recvHiPrioTimeouts               :")
	STATS(protoStatsPerStan[stan].recvLoPrioTimeouts,      "recvLoPrioTimeouts               :")
	STATS(protoStatsPerStan[stan].recvHiPrioErrCRC,        "recvHiPrioErrCRC                 :")
	STATS(protoStatsPerStan[stan].recvLoPrioErrCRC,        "recvLoPrioErrCRC                 :")
	STATS(protoStatsPerStan[stan].recvHiPrioErr,           "recvHiPrioErr                    :")
	STATS(protoStatsPerStan[stan].recvLoPrioErr,           "recvLoPrioErr                    :")
	ret=sprintf(p, "\n"); p=p+ret;

	return (p - str);

#undef STATS(__stat__,__str__)
}

static uint8_t inline usartComCRC (volatile uint8_t* frame, uint16_t len)
{
	uint8_t ret = 0xff;
	uint16_t k=0;

	for(k=0; k<len; k++)
		ret ^= frame[k];

	return ret;
}

static void protoSignalTimeout(void)
{
	int idx = stanMapAddr2Idx(usartCom_tx.addrReq);
	Stanowiska_t *pStan;
	if (idx >= 0)
	{
		pStan = &Stanowiska[idx];
		pStan->TimeoutFromProtocol();
	}
}

static void protoFuncReadAll(usartComRxUnionFrame_t readAllStr, uint16_t dataLength)
{
	int idx = stanMapAddr2Idx(usartCom_tx.addrReq);
	Stanowiska_t *pStan;
	if (idx >= 0)
	{
		pStan = &Stanowiska[idx];
		pStan->UpdWejscia(readAllStr.usartComReadAllStr.we);
		//		pStan->UpdWrzutnik(readAllStr.usartComReadAllStr.we);
		//		pStan->UpdErr(readAllStr.usartComReadAllStr.we);
		//		pStan->UpdCzytnik(readAllStr.usartComReadAllStr.czytLen, readAllStr.usartComReadAllStr.czytData);
	}
	//printf ("rx: addr x%x, protoFuncReadAll\n", usartCom_tx.addrReq);
}

static void protoFuncReadBz(usartComRxUnionFrame_t readAllStr, uint16_t dataLength)
{
	//int idx = stanMapAddr2Idx(usartCom_tx.addrReq);
	//printf ("rx: addr x%x, protoFuncReadBz\n", usartCom_tx.addrReq);
}

/*
 * return:
 * 0  - completed frame received/flush
 * 1  - partial frame received
 * -1 - error during receiving
 * -2 - frames without context
 */
static int parseRxProtocol(unsigned char *buff, unsigned int length, bool flush)
{
	unsigned int l;
	uint8_t usartComRxByte;
	static usartCom_rx_t usartCom_rx = {{{0, }}, WAITING_FOR_START_BYTE, FUNC_INVALID, 0, false, 0};

	if (flush)
	{
		usartComCritStats.usartComFlushesNr++;
		usartCom_rx.usartCom_rx_state = WAITING_FOR_START_BYTE;
		usartCom_rx.usartComDataLength=0;
		usartCom_rx.usartComFunction = FUNC_INVALID;
		usartCom_rx.czytLen=0;
		return 0;
	}

	for (l=0; l<length; l++)
	{
		usartComRxByte = (unsigned char) buff[l];

		switch (usartCom_rx.usartCom_rx_state)
		{
		case WAITING_FOR_START_BYTE:
			if (usartComRxByte == USART_COM_STARTBYTE)
				usartCom_rx.usartCom_rx_state = WAITING_FOR_ADDR;
			else
				usartComCritStats.usartComFramesWaitingForStart++;
			break;

		case WAITING_FOR_ADDR:
			if (usartComRxByte == USART_COM_MASTER_ADDR)
			{
				usartComCritStats.usartComFramesToMe++;
				usartCom_rx.usartCom_rx_state = WAITING_FOR_FUNC_CODE;
			}
			else if (usartComRxByte == USART_COM_BROADCAST_ADDR)
			{
				usartComCritStats.usartComFramesBroadcast++;
				goto ERROR;
			}
			else
			{
				usartComCritStats.usartComFramesToOthers++;
				goto ERROR;
			}
			break;

		case WAITING_FOR_FUNC_CODE:
			switch (usartComRxByte)
			{
			case FUNC_READ_ALL:
				usartCom_rx.usartCom_rx_state = WAITING_FOR_READ_DATA;
				usartCom_rx.usartComFunction = FUNC_READ_ALL;
				break;

			case FUNC_READ_BZ:
				usartCom_rx.usartCom_rx_state = WAITING_FOR_CRC;
				usartCom_rx.usartComFunction = FUNC_READ_BZ;
				break;

				// write from master... should not be visible if I'm not seeing what I'm transmitting
			case FUNC_WRITE_ALL:
			case FUNC_WRITE_BZ:
			default:
				usartComCritStats.usartComFramesFuncNotSupp++;
				goto ERROR;
			}
			break;

			case WAITING_FOR_READ_DATA: // START(1)|ADD(1)|FUNC(1)|WE(2)|CZYT_LEN(1)|CZYT_DATA(0..256)|CRC(1)|STOP(1)
				switch (usartCom_rx.usartComFunction)
				{
				case FUNC_READ_ALL:
					if (usartCom_rx.usartComDataLength < 5)
						break;

					if (usartCom_rx.usartComDataLength == 5)
						usartCom_rx.czytLen=usartComRxByte;

					if (usartCom_rx.usartComDataLength >= (FUNC_READ_ALL_MIN_LENGTH+usartCom_rx.czytLen-3))
						usartCom_rx.usartCom_rx_state=WAITING_FOR_CRC;
					break;
				default:
					usartComCritStats.usartComShouldNotHappen++;
					goto ERROR;
				}
				break;

				case WAITING_FOR_CRC:
					if (usartComRxByte==usartComCRC(usartCom_rx.usartComRxUnionFrame.usartComRxFrame, usartCom_rx.usartComDataLength))
						usartCom_rx.usartCom_rx_state=WAITING_FOR_STOP_BYTE;
					else
					{
						usartComCritStats.usartComFramesBedCrc++;
						usartCom_rx.usartCom_rx_state=WAITING_FOR_STOP_BYTE_ERROR_CRC;
					}
					break;

				case WAITING_FOR_STOP_BYTE_ERROR_CRC:
					if (usartComRxByte == USART_COM_STOPTBYTE)
					{
						if (usartCom_tx.sendHiPrio)
							protoStatsPerStan[stanMapAddr2Idx(usartCom_tx.addrReq)].recvHiPrioErrCRC++;
						else
							protoStatsPerStan[stanMapAddr2Idx(usartCom_tx.addrReq)].recvLoPrioErrCRC++;
					}
					else
						usartComCritStats.usartComFramesWaitingForStop++;

					goto ERROR;

				case WAITING_FOR_STOP_BYTE:
					if (usartComRxByte == USART_COM_STOPTBYTE)
					{
						usartCom_rx.usartComRxUnionFrame.usartComRxFrame[usartCom_rx.usartComDataLength]=usartComRxByte;
						usartCom_rx.usartComDataLength++;
						usartComCritStats.usartComGoodFramesToMe++;

#if 0
						printf ("RCVED: ");
						int k;
						for (k=0; k< usartCom_rx.usartComDataLength; k++)
							printf("%2x ", usartCom_rx.usartComRxUnionFrame.usartComRxFrame[k]);
						printf ("\n");
#endif

						switch (usartCom_rx.usartComFunction)
						{
						case FUNC_READ_ALL:
							protoFuncReadAll(usartCom_rx.usartComRxUnionFrame, usartCom_rx.usartComDataLength);
							break;

						case FUNC_READ_BZ:
							protoFuncReadBz(usartCom_rx.usartComRxUnionFrame, usartCom_rx.usartComDataLength);
							break;

						default:
							usartComCritStats.usartComShouldNotHappen++;
							goto ERROR;
						}
						usartCom_rx.usartCom_rx_state = FINISHED;
					}
					else
					{
						usartComCritStats.usartComFramesWaitingForStop++;
						goto ERROR;
					}
					break;

				case FINISHED:
					//bajty za bajtem STOPu sa ignorowane, konczymy return 0
					usartComCritStats.usartComBytesAfterFinish++;
					break;

				default:
					usartComCritStats.usartComShouldNotHappen++;
					goto ERROR;

		} //END of switch (usartCom_rx.usartCom_rx_state)

		if ((usartCom_rx.usartCom_rx_state != WAITING_FOR_START_BYTE) &&
				(usartCom_rx.usartCom_rx_state != FINISHED))
		{
			usartCom_rx.usartComRxUnionFrame.usartComRxFrame[usartCom_rx.usartComDataLength]=usartComRxByte;
			usartCom_rx.usartComDataLength++;
		}

	} //END of for (l=0; l<length; l++)

	//przychodza bajty, a my czekamy na bajt startu, bez tego po odebraniu 1 jakiegos bajtu zwracalibysmy "partial frame"
	if (usartCom_rx.usartCom_rx_state == WAITING_FOR_START_BYTE)
	{
		usartComCritStats.usartComBufsWithoutContext++;
		return -2;
	}

	//niedokonczone ramki
	if (usartCom_rx.usartCom_rx_state != FINISHED)
	{
		usartComCritStats.usartComPartialFrames++;
		return 1;
	}
	else
	{
		usartComCritStats.usartComFramesCompleted++;
		usartCom_rx.usartCom_rx_state = WAITING_FOR_START_BYTE;
		usartCom_rx.usartComDataLength=0;
		usartCom_rx.usartComFunction = FUNC_INVALID;
		usartCom_rx.czytLen=0;
		return 0;
	}

	ERROR:
	usartCom_rx.usartCom_rx_state = WAITING_FOR_START_BYTE;
	usartCom_rx.usartComDataLength=0;
	usartCom_rx.usartComFunction = FUNC_INVALID;
	usartCom_rx.czytLen=0;
	return -1;
}

static void *rxRs485Thread(void *arg)
{
	int retSelect, retRead, parseRet;
	unsigned char buff[255];
	fd_set set;
	struct timeval timeout;

	while(1)
	{
		pthread_mutex_lock(&usartCom_tx.mutexUartReadWrite);
		while (usartCom_tx.countUart == 0)
			pthread_cond_wait(&usartCom_tx.fillUart, &usartCom_tx.mutexUartReadWrite);

		//read
		while (1)
		{
			FD_ZERO(&set);
			FD_SET(fd, &set);
			timeout.tv_sec = 0;
			timeout.tv_usec = 300000;

			retSelect=select(fd + 1, &set, NULL, NULL, &timeout);
			if(retSelect < 0)
			{
				usartComCritStats.selectLinuxErr++;
				perror("select");
				continue;
			}
			else if(retSelect == 0)
			{
				//Timeout
				printf("T-addr 0x%x \n", usartCom_tx.addrReq);
				protoSignalTimeout();

				tcflush(fd, TCIFLUSH);

				if (usartCom_tx.sendHiPrio)
					protoStatsPerStan[stanMapAddr2Idx(usartCom_tx.addrReq)].recvHiPrioTimeouts++;
				else
					protoStatsPerStan[stanMapAddr2Idx(usartCom_tx.addrReq)].recvLoPrioTimeouts++;

				//only clean static variables in parseRxProtocol
				parseRxProtocol(NULL, 0, true);
				break;
			}
			else
			{
				//sprawdzic czy read nie zwroci bledu
				retRead=read(fd, buff, sizeof(buff));
				if(retRead <= 0)
				{
					perror("read");
					usartComCritStats.readLinuxErr++;
				}

				parseRet=parseRxProtocol(buff, retRead, false);
				if (!parseRet)
				{
					if (usartCom_tx.sendHiPrio)
						protoStatsPerStan[stanMapAddr2Idx(usartCom_tx.addrReq)].recvHiPrioOk++;
					else
						protoStatsPerStan[stanMapAddr2Idx(usartCom_tx.addrReq)].recvLoPrioOk++;
					break;
				}
				else if (parseRet<0)
				{
					if (usartCom_tx.sendHiPrio)
						protoStatsPerStan[stanMapAddr2Idx(usartCom_tx.addrReq)].recvHiPrioErr++;
					else
						protoStatsPerStan[stanMapAddr2Idx(usartCom_tx.addrReq)].recvLoPrioErr++;

					usleep(50000);
					tcflush(fd, TCIFLUSH);
				}
				else
				{
					if (usartCom_tx.sendHiPrio)
						protoStatsPerStan[stanMapAddr2Idx(usartCom_tx.addrReq)].recvHiPrioPartial++;
					else
						protoStatsPerStan[stanMapAddr2Idx(usartCom_tx.addrReq)].recvLoPrioPartial++;
				}
			}
		}
		usartCom_tx.countUart=0;
		pthread_cond_signal(&usartCom_tx.emptyUart);
		pthread_mutex_unlock(&usartCom_tx.mutexUartReadWrite);
	}
	return NULL;
}

static void *txHiPrioRs485Thread(void *arg)
{
#if 1
	unsigned char nrOfEnabledStan;
	struct timeval timeout;
	unsigned char idx=0;
	Stanowiska_t *pStan;
	unsigned char addr;

	while (1)
	{
		nrOfEnabledStan=stanGetNumberOfEnabled();

		for(idx=0; idx<MAX_STAN; idx++)
		{
			pStan=&Stanowiska[idx];
			if(pStan->enabled)
			{
				timeout.tv_sec = 0;
				timeout.tv_usec = MAX_POLL_INTERVAL_US/nrOfEnabledStan;

				select(0,NULL,NULL,NULL,&timeout);

				addr=pStan->addr;
				protoFuncSendRaw(addr,FUNC_WRITE_BZ,NULL,0,true);
			}
		}
	}

#else
	struct timeval timeout;
	static unsigned int k=0;
	unsigned char buf[255];

	while (1)
	{
		timeout.tv_sec = 0;
		timeout.tv_usec = 200000;

		select(0,NULL,NULL,NULL,&timeout);

		if(k)
		{
			//faked response from R05 ReadBz
			protoFuncSendRaw(0x0, 0x03, NULL, 0, true);
			k=0;
		}
		else
		{
			//faked response from R05 ReadAll
			protoFuncSendRaw(0x0, 0x03, NULL, 0, true);
			k=1;
		}
	}
	return NULL;
#endif
}

static void *txRs485Thread(void *arg)
{
	int ret;
	unsigned int written;

	while(1)
	{
		written=0;

		/* czekamy na wolna szyne rs485 (producer na szyne 485) */
		pthread_mutex_lock(&usartCom_tx.mutexUartReadWrite);
		while (usartCom_tx.countUart == 1)
			pthread_cond_wait(&usartCom_tx.emptyUart, &usartCom_tx.mutexUartReadWrite);

		/* czekamy az sie pojawia dane (consumer z bufora) */
		pthread_mutex_lock(&usartCom_tx.mutexBufReadWrite);
		while ((usartCom_tx.countBufNormal == 0) && (usartCom_tx.countBufHiPrio == 0))
			pthread_cond_wait(&usartCom_tx.fillBuf, &usartCom_tx.mutexBufReadWrite);

		if (usartCom_tx.countBufHiPrio == 1)
		{
			usartCom_tx.addrReq=usartCom_tx.usartComTxUnionFrame[USART_COM_TX_HIPRIO_IDX].usartComTxFrame[1];
			usartCom_tx.sendHiPrio=true;
			do
			{
				ret=write(fd, (void*)(((unsigned char*)(usartCom_tx.usartComTxUnionFrame[USART_COM_TX_HIPRIO_IDX].usartComTxFrame)) + written), (usartCom_tx.usartComTxDataLength[USART_COM_TX_HIPRIO_IDX]-written));
				if (ret <= 0)
				{
					perror("write");
					usartComCritStats.writeLinuxErr++;
					break;
				}
				else
					written=written+ret;

			}
			while (written!=usartCom_tx.usartComTxDataLength[USART_COM_TX_HIPRIO_IDX]);

			if (ret > 0)
				protoStatsPerStan[stanMapAddr2Idx(usartCom_tx.addrReq)].sendHiPrioFrames++;

			usartCom_tx.countBufHiPrio=0;
		}
		else if(usartCom_tx.countBufNormal == 1)
		{
			usartCom_tx.addrReq=usartCom_tx.usartComTxUnionFrame[USART_COM_TX_LOPRIO_IDX].usartComTxFrame[1];
			usartCom_tx.sendHiPrio=false;
			do
			{
				ret=write(fd, (void*)(((unsigned char*)(usartCom_tx.usartComTxUnionFrame[USART_COM_TX_LOPRIO_IDX].usartComTxFrame)) + written), (usartCom_tx.usartComTxDataLength[USART_COM_TX_LOPRIO_IDX]-written));
				if (ret <= 0)
				{
					perror("write");
					usartComCritStats.writeLinuxErr++;
					break;
				}
				else
					written=written+ret;
			}
			while (written!=usartCom_tx.usartComTxDataLength[USART_COM_TX_LOPRIO_IDX]);

			if (ret > 0)
				protoStatsPerStan[stanMapAddr2Idx(usartCom_tx.addrReq)].sendLoPrioFrames++;

			usartCom_tx.countBufNormal=0;
		}
		else
		{
			usartComCritStats.txRs485ThreadErr++;
			printf("Should not happen I think\n");
		}

		tcdrain(fd);

#if 0
		unsigned int k;
		for (k=0; k<usartComTxLength; k++)
			printf ("x%x ", usartComTxUnionFrame.usartComTxFrame[k]);
		printf("\n");
#endif

		pthread_cond_signal(&usartCom_tx.emptyBuf);
		pthread_mutex_unlock(&usartCom_tx.mutexBufReadWrite);

		usartCom_tx.countUart=1;
		pthread_cond_signal(&usartCom_tx.fillUart);
		pthread_mutex_unlock(&usartCom_tx.mutexUartReadWrite);
	}
	return NULL;
}


