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
	DEV_R06
} protoDevType_t;

typedef enum PROTO_STATS_TYPE {
	RECV_HIPRIO_PARTIAL,
	RECV_LOPRIO_PARTIAL,
	SEND_HIPRIO_FRAMES,
	SEND_LOPRIO_FRAMES,
	RECV_HIPRIO_OK,
	RECV_LOPRIO_OK,
	RECV_HIPRIO_TIMEOUTS,
	RECV_LOPRIO_TIMEOUTS,
	RECV_HIPRIO_CRC,
	RECV_LOPRIO_CRC,
	RECV_HIPRIO_ERR,
	RECV_LOPRIO_ERR,
	RECV_HIPRIO_STANTIMEOUT,
	RECV_LOPRIO_STANTIMEOUT,
	PROTO_STATS_TYPE_MAX
} protoStatsType_t;

class protoDev_t {
public:
	bool configured;
	bool preiodicPollEnabled;
	uint32_t protoStats[PROTO_STATS_TYPE_MAX];

protected:
	unsigned char rs485Addr;
	unsigned char devNr;

public:
	protoDev_t() {	}
	protoDev_t(bool conf, bool poll, unsigned char addr, unsigned char nr)
	{
		configured=conf;
		preiodicPollEnabled=poll;
		rs485Addr=addr;
		devNr=nr;
	}

	virtual protoDevType_t type() const { return DEV_BASE; }
	unsigned int getRs485devAdd(void) { return ((devNr<<8) | rs485Addr); }

	//interfejs z protokolu ... wirtualne, do przedefiniowannia w klasach pochodnych
	virtual void hwReadAll(unsigned char *we, unsigned char wrzutnik, bool err, unsigned char *pCzytnik, unsigned char czytLen)	{ }
	virtual void hwReadBz(void)	{ }

	void TimeoutFromProtocol(void);
	void RcvErrorFromProtocol(void);
};

#endif /* PROTODEV_H_ */
