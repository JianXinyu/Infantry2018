/**
  ******************************************************************************
  * File Name          : rtos_init.h
  * Description        : FreeRTOS初始化
  ******************************************************************************
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  ******************************************************************************
  */
#ifndef RTOS_INIT_H
#define RTOS_INIT_H

#include <stdint.h>
#include <stdbool.h>

extern bool g_bInited;
void rtos_InitInfantry(void);
void rtos_AddSemaphores(void);
void rtos_AddThreads(void);

#endif
