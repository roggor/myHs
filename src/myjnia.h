/*
 * myjnia.h
 *
 *  Created on: Nov 2, 2015
 *      Author: maciek
 */

#ifndef MYJNIA_H_
#define MYJNIA_H_

extern protoDev_t *protoDev[MAX_R0X];

unsigned char getNumberOfConfigured(void);
protoDev_t *mapDevAddr2Dev(unsigned char dev, unsigned char addr);
void devArrayInit(void);

#endif /* MYJNIA_H_ */
