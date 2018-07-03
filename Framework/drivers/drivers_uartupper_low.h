/**
  ******************************************************************************
  * File Name          : drivers_uartupper_low.h
  * Description        : 妙算通信
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 协议处理函数
  ******************************************************************************
  */
#ifndef DRIVERS_UARTUPPER_LOW_H
#define DRIVERS_UARTUPPER_LOW_H

#include "drivers_uartupper_user.h"

void manifoldUartRxCpltCallback(void);

void InitManifoldUart(void);
void vSendUart(xdata_ctrlUart data);
xdata_ctrlUart xUartprocess(uint8_t *pData);

#endif
