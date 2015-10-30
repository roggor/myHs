/*
 * hopper.h
 *
 *  Created on: Oct 29, 2015
 *      Author: maciek
 */

#ifndef SRC_HOPPER_H_
#define SRC_HOPPER_H_

#define PROTO_WRITE_SSP_HOP_MAX_DATA	0x20 //ilosc danych do hoppera w protokole (w R06 jest 32)
#define PROTO_WRITE_SSP_HOP_MAX (PROTO_WRITE_SSP_HOP_MAX_DATA + 7)


#define HOP_CC2_VALUE_LEN		4
#define HOP_CC2_CURENCY_LEN		3

#define HOP_CC2_VALUE_CURR_LEN	(HOP_CC2_VALUE_LEN + HOP_CC2_CURENCY_LEN)

#define HOP_CC2_OVERHEAD		5 //dstAddr, len, srcAddr, cmd, crc
#define HOP_CC2_MAX_DATA		(PROTO_WRITE_SSP_HOP_MAX_DATA - HOP_CC2_OVERHEAD) //ilosc danych w protokole CC2

//commands codes
#define HOP_FLOAT_AMOUNT_CMD	0x17
#define HOP_FLOAT_AMOUNT_LEN	(HOP_CC2_OVERHEAD + HOP_CC2_VALUE_CURR_LEN)
//addresses
#define HOP_ADDR_HOST			0x01
#define HOP_ADDR_DEVICE			0x03

//faked crc for now
#define CRC(__buff__,__len__) 0xaa

typedef enum
{
	PLN=(('P'<<16) + ('L'<<8) + ('N'<<0)),
	EUR=(('E'<<16) + ('U'<<8) + ('R'<<0))
} currency_t;

typedef union
{
	struct _cc2Frame
	{
		unsigned char dstAddr;
		unsigned char len;
		unsigned char srcAddr;
		unsigned char cmd;
		unsigned char data[HOP_CC2_MAX_DATA];
		unsigned char crc;
	} cc2Frame;

	unsigned char cc2RawBytes[PROTO_WRITE_SSP_HOP_MAX_DATA];
} cc2Frame_t;



class Rozmieniarka_t {
public:
	unsigned char rs485Addr; //R06 rs485 address
	char *dev; // /dev/ttySx

private:
	cc2Frame_t hopTxBuff;
	unsigned int hopTxDataLen;

	cc2Frame_t nvTxBuff; //faked for now
	unsigned int nvTxDataLen; //faked for now

public:
	void setHopReqFloat(uint32_t amount, currency_t currency, bool wrHw, bool hiPrio);

/********** Interfejs rozmieniarka <-> protokol *********/
private:
	void hwWriteSSP(bool hiPrio);

/********************************************************/
};













#endif /* SRC_HOPPER_H_ */
