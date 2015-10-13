/*
 * globalConfig.h
 *
 *  Created on: Oct 5, 2015
 *      Author: maciek
 */

#ifndef GLOBALCONFIG_H_
#define GLOBALCONFIG_H_

#define MAX_STAN 10
#define DUMMY_STAN_IDX 10

#define USART_COM_BROADCAST_ADDR 0xFF
#define USART_COM_MASTER_ADDR    0xFE

/*
 * indeksy od 0...MAX_STAN
 */
#define DEF_MAP_IDX2ADDR {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9}

#define DEF_THREAD_STACKSIZE    (128 * 1024)
#define DEF_TIME_SYNC_PRIO       80


#endif /* GLOBALCONFIG_H_ */
