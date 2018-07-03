/**
  ******************************************************************************
  * File Name          : tasks_upper.h
  * Description        : 上位机(妙算)通信任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef TASKS_UPPER_H
#define TASKS_UPPER_H

#include "utilities_iopool.h"


IOPoolDeclare(upperIOPool, struct{float yawAdd; float pitchAdd; uint8_t rune;});

void ManifoldUartTask(void const * argument);

#endif
