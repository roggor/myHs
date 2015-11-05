/*
 * protoDevBase.cpp
 *
 *  Created on: Oct 30, 2015
 *      Author: maciek
 */
#include <stdio.h>

#include "configGlobal.h"
#include "protoDev.h"

void protoDev_t::TimeoutFromProtocol(void)
{
	printf ("Stan (devNr %2d, addr 0x%2x): TimeoutFromProtoco\n", devNr,rs485Addr);
}

void protoDev_t::RcvErrorFromProtocol(void)
{
	printf ("Stan (devNr %2d, addr 0x%2x): RcvErrorFromProtocol\n", devNr,rs485Addr);
}
