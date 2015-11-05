/*
 * myjnia.cpp
 *
 *  Created on: Nov 2, 2015
 *      Author: maciek
 */
#include "cli.h"
#include "configGlobal.h"
#include "protoCol.h"
#include "protoDev.h"
#include "protoDevR05.h"
#include "protoDevR06.h"
#include "myjnia.h"


static protoDevR05_t protoDevR05[MAX_R05];
static protoDevR06_t protoDevR06[MAX_R06];
static protoDev_t protoDevDummy;

protoDev_t *protoDev[MAX_R0X];

unsigned char getNumberOfConfigured(void)
{
	unsigned char k;
	unsigned char ret=0;
	protoDevR05_t *pStan;

	for (k=0; k<=GOOD_R0X_IDX; k++)
	{
		pStan=&protoDevR05[k];
		if(pStan->configured)
			ret++;
	}
	return ret;
}

protoDev_t *mapDevAddr2Dev(unsigned char dev, unsigned char addr)
{
	unsigned int k;
	for (k=0; k<=(GOOD_R0X_IDX); k++)
		if (protoDev[k]->getRs485devAdd() == (unsigned int)((dev<<8) | addr))
			return protoDev[k];

	return protoDev[DUMMY_R0X_IDX];
}

static void devPointerArrayInit(void)
{
	unsigned int k;

	for (k=0; k<MAX_R05; k++)
		protoDev[k]=&protoDevR05[k];

	for (k=0; k<MAX_R06; k++)
		protoDev[k+MAX_R05]=&protoDevR06[k];

	protoDev[DUMMY_R0X_IDX]=&protoDevDummy;
}

void devArrayInit(void)
{
	devPointerArrayInit();

}
