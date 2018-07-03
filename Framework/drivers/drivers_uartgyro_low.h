/**
  ******************************************************************************
  * File Name          : pdrivers_uartgyro_low.h
  * Description        : 外接陀螺仪读取
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 底层函数
  ******************************************************************************
  */
#ifndef DRIVERS_UARTGYRO_LOW_H
#define DRIVERS_UARTGYRO_LOW_H

#include "utilities_iopool.h"
#include "cmsis_os.h"

void InitGyroUart(void);
void gyroUartRxCpltCallback(void);
uint8_t sumCheck(void);
#endif
