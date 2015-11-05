/*
 * globalConfig.h
 *
 *  Created on: Oct 5, 2015
 *      Author: maciek
 */

#ifndef CONFIGGLOBAL_H_
#define CONFIGGLOBAL_H_

#define MAX_R05 10
#define MAX_R06 2
#define MAX_R0X (MAX_R05+MAX_R06+1)

#define GOOD_R0X_IDX (MAX_R05+MAX_R06-1)
#define DUMMY_R0X_IDX (MAX_R05+MAX_R06)

#define USART_COM_BROADCAST_ADDR 0xFF
#define USART_COM_MASTER_ADDR    0xFE

/*
 * indeksy od 0...MAX_STAN
 */
#define DEF_MAP_IDX2ADDR {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9}

#define DEF_THREAD_STACKSIZE    (128 * 1024)
#define DEF_TIME_SYNC_PRIO       80

/*TODO verify */
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;


#endif /* CONFIGGLOBAL_H_ */
