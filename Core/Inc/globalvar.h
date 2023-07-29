/*
 * Author: Lucas de Jesus B. Gonçalves
 *
 * Last modified: 02/04/2023
 * Description: I2C library for the TMP100 sensor using stm32.
 */


#ifndef GLOBALVAR_H
#define GLOBALVAR_H

#include <stdint.h>
#include "stm32l0xx_hal.h"

typedef struct
{
	float temp;
	int status;
	uint8_t id;
} TMP100_DATA;

typedef struct
{
	long x;
	long y;
	long z;
	float gain;
	double uT;
	uint8_t id;
} RM3100_DATA;

enum ERROR {
ERROR_OK        = 0,
ERROR_FAIL      = 1,
ERROR_ALLTXBUSY = 2,
ERROR_FAILINIT  = 3,
ERROR_FAILTX    = 4,
ERROR_NOMSG     = 5
};

#endif
