/*
 * hopper.cpp
 *
 *  Created on: Oct 29, 2015
 *      Author: maciek
 */

/*
hopPayout
hopFloat(unsigned int amount)
hopEmpty
hopReqStatus
*/
#include <string.h>

#include "configGlobDemo.h"
#include "configKartDemo.h"

#include "protoCol.h"
#include "protoDevR06.h"


void protoDevR06_t::hwWriteSSP(bool hiPrio)
{
	protoFuncWriteSSP(ttyNr, txAddr, hopTxBuff.cc2RawBytes, hopTxDataLen, nvTxBuff.cc2RawBytes, nvTxDataLen, hiPrio);
}

void protoDevR06_t::setHopReqFloat(uint32_t amount, currency_t currency, bool wrHw, bool hiPrio)
{
	hopTxBuff.cc2Frame.dstAddr=HOP_ADDR_DEVICE;
	hopTxBuff.cc2Frame.len=(HOP_FLOAT_AMOUNT_LEN-HOP_CC2_OVERHEAD);
	hopTxBuff.cc2Frame.srcAddr=HOP_ADDR_HOST;
	hopTxBuff.cc2Frame.cmd=HOP_FLOAT_AMOUNT_CMD;

	hopTxBuff.cc2Frame.data[0]=((amount >> 0 ) & 0xff);
	hopTxBuff.cc2Frame.data[1]=((amount >> 8 ) & 0xff);
	hopTxBuff.cc2Frame.data[2]=((amount >> 16) & 0xff);
	hopTxBuff.cc2Frame.data[3]=((amount >> 24) & 0xff);

	hopTxBuff.cc2Frame.data[4]=((currency >> 0 ) & 0xff);
	hopTxBuff.cc2Frame.data[5]=((currency >> 8 ) & 0xff);
	hopTxBuff.cc2Frame.data[6]=((currency >> 16) & 0xff);

	hopTxBuff.cc2Frame.crc=CRC(buff,HOP_FLOAT_AMOUNT_LEN);

	hopTxDataLen=HOP_FLOAT_AMOUNT_LEN;

	if(wrHw)
		hwWriteSSP(hiPrio);
}
