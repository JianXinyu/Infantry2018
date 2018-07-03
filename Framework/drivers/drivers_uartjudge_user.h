#ifndef DRIVERS_UARTJUDGE_USER_H
#define DRIVERS_UARTJUDGE_USER_H

#include "utilities_iopool.h"

typedef struct{
	float voltage;
	float electricity;
	float remainPower;
}JudgePower_t;
IOPoolDeclare(judgePowerUartIOPool, JudgePower_t);

#endif
