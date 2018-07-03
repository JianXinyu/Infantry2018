/**
  ******************************************************************************
  * File Name          : rtos_semaphore.h
  * Description        : 添加FreeRTOS信号量
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 添加信号量用于任务（进程）间同步

  ******************************************************************************
  */
#ifndef RTOS_SEMAPHORE_H
#define RTOS_SEMAPHORE_H

#include "cmsis_os.h"


extern osSemaphoreId CMGMCanHaveTransmitSemaphoreHandle;
extern osSemaphoreId ZGYROCanHaveTransmitSemaphoreHandle;

extern osSemaphoreId CMGMCanTransmitSemaphoreHandle;
extern osSemaphoreId ZGYROCanTransmitSemaphoreHandle;

extern osSemaphoreId CMGMCanRefreshSemaphoreHandle;
extern osSemaphoreId ZGYROCanRefreshSemaphoreHandle;

extern osSemaphoreId imurefreshGimbalSemaphoreHandle;


//extern osSemaphoreId imuSpiTxRxCpltSemaphoreHandle;
extern osSemaphoreId refreshMPU6500SemaphoreHandle;

extern xSemaphoreHandle xSemaphore_mfuart;
extern xSemaphoreHandle xSemaphore_rcuart;
extern xSemaphoreHandle motorCanTransmitSemaphore;

#endif
