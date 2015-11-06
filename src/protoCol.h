/*
 * protocol.h
 *
 *  Created on: Oct 1, 2015
 *      Author: maciek
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <pthread.h>

#define MAX_DATA_LENGTH		256  /* 252 data bytes + 2 start bytes + 2 stop bytes */
#define USART_COM_MAXFRAME 8+MAX_DATA_LENGTH // START(1)|ADDR(1)|FUNC(1)|WE(2)|CZYT_LEN(1)|CZYT_DATA(256)|CRC(1)|STOP(1)

#define USART_COM_STARTBYTE	  0xA1
#define USART_COM_STOPTBYTE	  0xA2

#define FUNC_READ_ALL_MIN_LENGTH 8
#define FUNC_WRITE_ALL_DATA_LENGTH 7
#define FUNC_WRITE_SSP_DATA_LEN 66

//czas co ile bedzie odpytywany kazdy modul
#define MAX_POLL_INTERVAL_US 300000

typedef enum
{
	MASTER,
	SLAVE,
	NONE
} funOnTty_t;

typedef struct
{
	uint8_t ttyNr;
	funOnTty_t funOnBus;
} protoThreadArgs_t;

typedef union
{
	struct _usartComReadAllStr
	{
		// START(1)|ADD(1)|FUNC(1)|WE(2)|CZYT_LEN(1)|CZYT_DATA(1..255)|CRC(1)|STOP(1)
		uint8_t start;
		uint8_t addr;
		uint8_t func;
		uint8_t we[2];
		uint8_t czytLen;
		uint8_t czytData[255];
		uint8_t crc;
		uint8_t stop;
	} usartComReadAllStr;

	struct _usartComReadBzStr
	{
		//START(1)|ADDR(1)|FUNC(1)|CRC(1)|STOP(1)
		uint8_t start;
		uint8_t addr;
		uint8_t func;
		uint8_t crc;
		uint8_t stop;
	} usartComReadBzStr;

	uint8_t usartComRxFrame[USART_COM_MAXFRAME];
} usartComRxUnionFrame_t;

typedef union
{
	struct _usartComWriteAllStr
	{
		//START(1)|ADDR(1)|FUNC(1)|WY(2)|DISP(4)|DISP_OPTIONS(1)|CRC(1)|STOP(1)
		uint8_t start;
		uint8_t addr;
		uint8_t func;
		uint8_t wy[2];
		uint8_t disp[4];
		uint8_t disp_options;
		uint8_t crc;
		uint8_t stop;
	} usartComWriteAllStr;

	struct _usartComWriteBzStr
	{
		//START(1)|ADDR(1)|FUNC(1)|CRC(1)|STOP(1)
		uint8_t start;
		uint8_t addr;
		uint8_t func;
		uint8_t crc;
		uint8_t stop;
	} usartComWriteBzStr;

	uint8_t usartComTxFrame[USART_COM_MAXFRAME];
} usartComTxUnionFrame_t;

typedef enum
{
	FUNC_WRITE_ALL=0,
	FUNC_WRITE_BZ,
	FUNC_READ_ALL,
	FUNC_READ_BZ,
	FUNC_WRITE_SSP,
	FUNC_READ_SSP,
	FUNC_WRITE_PRT,
	FUNC_READ_PRT,
	FUNC_READ_ERROR1,
	FUNC_INVALID
} USART_COM_RX_FUNC_t;

typedef enum
{
	WAITING_FOR_START_BYTE = 0,
	WAITING_FOR_ADDR,
	WAITING_FOR_FUNC_CODE,
//	WAITING_FOR_WRITE_DATA,
	WAITING_FOR_READ_DATA, //data comming from other slaves, state until data length field
	WAITING_FOR_CRC,
	WAITING_FOR_STOP_BYTE,
	WAITING_FOR_STOP_BYTE_ERROR_CRC,
	FINISHED,
} USART_COM_RX_STATE_t;

typedef struct
{
	uint32_t usartComPartialFrames;
	uint32_t usartComFramesCompleted;
	uint32_t usartComFramesToMe;
	uint32_t usartComGoodFramesToMe;
	uint32_t usartComFlushesNr;
	uint32_t usartComFramesBroadcast;
	uint32_t usartComFramesToOthers;
	uint32_t usartComFramesFuncNotSupp;
	uint32_t usartComFramesWaitingForStart;
	uint32_t usartComFramesWaitingForStop;
	uint32_t usartComFramesBedCrc;
	uint32_t usartComBufsWithoutContext;
	uint32_t usartComBytesAfterFinish;
	uint32_t usartComShouldNotHappen;

	uint32_t readLinuxErr;
	uint32_t selectLinuxErr;
	uint32_t writeLinuxErr;
	uint32_t txRs485ThreadErr;
} usartComCritStats_t;

typedef struct _usartCom_rx
{
	usartComRxUnionFrame_t usartComRxUnionFrame;
	USART_COM_RX_STATE_t usartCom_rx_state;
	USART_COM_RX_FUNC_t usartComFunction;

	uint8_t czytLen;
	bool usartComToMe;
	uint16_t usartComDataLength;

} usartCom_rx_t;

typedef struct _usartCom_tx
{
	pthread_mutex_t mutexUsart;
	unsigned int countUart;
	pthread_cond_t  condUsartEmpty, condUsartFull;

	pthread_mutex_t mutexBuffer;
	unsigned int countBufNormal;
	unsigned int countBufHiPrio;
	pthread_cond_t  condBufferEmpty, condBufferFull;

	unsigned char addrReq;
	bool sendHiPrio;

#define USART_COM_TX_LOPRIO_IDX 0
#define USART_COM_TX_HIPRIO_IDX 1

	usartComTxUnionFrame_t usartComTxUnionFrame[USART_COM_TX_HIPRIO_IDX+1];
	uint16_t usartComTxDataLength[USART_COM_TX_HIPRIO_IDX+1];
} usartCom_tx_t;

void protoInit (void);
void protoFuncWriteAll(unsigned int ttyNr, unsigned char addr, unsigned char *wy, unsigned char *disp, unsigned char dots, unsigned char opts, bool hiPrio);
void protoFuncWriteSSP(unsigned int ttyNr, unsigned char addr, unsigned char *hopData, unsigned char hopLen, unsigned char *nvData, unsigned char nvLen, bool hiPrio);

int protoGetGlobalStats(char* str);

#endif /* PROTOCOL_H_ */
