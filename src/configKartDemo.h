/*
 * configKartuzy.h
 *
 *  Created on: Oct 30, 2015
 *      Author: maciek
 */

#ifndef CONFIGKARTDEMO_H_
#define CONFIGKARTDEMO_H_

#define TTY_DEV0 		0
#define TTY_DEV1 		1
#define TTY_DEV2 		2
#define MAX_TTY_DEVS 	3


#define DEV_NAMES 			"/dev/ttyUSB0", 	/* TTY_DEV0 */	\
							"/dev/ttyUSB1", 	/* TTY_DEV1 */	\
							"/dev/ttyUSB2"  	/* TTY_DEV2 */

#define PROTO_THREAD_ARGS 	{ TTY_DEV0 , SLAVE  }, \
							{ TTY_DEV1 , MASTER }, \
							{ TTY_DEV2 , NONE   }

#endif /* CONFIGKARTDEMO_H_ */
