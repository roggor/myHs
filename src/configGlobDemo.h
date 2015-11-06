/*
 * globalConfig.h
 *
 *  Created on: Oct 5, 2015
 *      Author: maciek
 */

#ifndef CONFIGGLOBDEMO_H_
#define CONFIGGLOBDEMO_H_

#define TTY_DEV0 		0
#define TTY_DEV1 		1
#define TTY_DEV2 		2
#define MAX_TTY_DEVS 	3

#define MAX_R05   		10
#define MAX_R06   		2
#define MAX_SLAVE 		1
#define MAX_R0X 		(MAX_R05 + MAX_R06 + MAX_SLAVE + 1)
#define GOOD_R0X_IDX 	(MAX_R05 + MAX_R06 + MAX_SLAVE - 1)
#define DUMMY_R0X_IDX 	(MAX_R05 + MAX_R06 + MAX_SLAVE)

#define USART_COM_BROADCAST_ADDR 0xFF
#define USART_COM_MASTER_ADDR    0xFE

#define DEF_THREAD_STACKSIZE    (128 * 1024)
#define DEF_TIME_SYNC_PRIO       80

/*TODO verify */
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;


#endif /* CONFIGGLOBDEMO_H_ */
