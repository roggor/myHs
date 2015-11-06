/*
 * protoDevBase.h
 *
 *  Created on: Oct 30, 2015
 *      Author: maciek
 */

#ifndef PROTODEV_H_
#define PROTODEV_H_

typedef enum PROTO_DEV_TYPE {
	DEV_BASE,
	DEV_R05,
	DEV_R06,
	DEV_IM_SLAVE //ja jestem slavem
} protoDevType_t;

typedef enum PROTO_STATS_TYPE {
	SEND_HIPRIO_FRAMES,
	SEND_LOPRIO_FRAMES,
	RECV_PARTIAL,
	RECV_OK,
	RECV_TIMEOUTS,
	RECV_CRC,
	RECV_ERR,
	RECV_STANTIMEOUT,
	PROTO_STATS_TYPE_MAX
} protoStatsType_t;

class protoDev_t {
public:
	bool configured;
	bool preiodicPollEnabled;
	uint32_t protoStats[PROTO_STATS_TYPE_MAX];

protected:
	unsigned char txAddr; //adres do ktorego wysylam
	unsigned char rxAddr; //adres na jakim odbieram
	unsigned char ttyNr;

public:
	protoDev_t() {	}
	protoDev_t(bool conf, bool poll, unsigned char addr, unsigned char nr)
	{
		configured=conf;
		preiodicPollEnabled=poll;
		txAddr=addr;
		ttyNr=nr;
	}

	virtual protoDevType_t type() const { return DEV_BASE; }
	unsigned int getDevTxAdd(void) { return ((ttyNr<<8) | txAddr); }
	unsigned int getDevRxAdd(void) { return (rxAddr); }


	//interfejs z protokolu ... wirtualne, do przedefiniowannia w klasach pochodnych
	virtual void hwReadAll(unsigned char *we, unsigned char wrzutnik, bool err, unsigned char *pCzytnik, unsigned char czytLen)	{ }
	virtual void hwReadBz(void)	{ }

	void TimeoutFromProtocol(void);
	void RcvErrorFromProtocol(void);
};

#endif /* PROTODEV_H_ */
