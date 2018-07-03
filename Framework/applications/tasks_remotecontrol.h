/**
  ******************************************************************************
  * File Name          : tasks_remotecontrol.h
  * Description        : 遥控器处理任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef FRAMEWORK_TASKS_REMOTECONTROL_H
#define FRAMEWORK_TASKS_REMOTECONTROL_H


#include "cmsis_os.h"
#include "drivers_uartrc_low.h"

void RControlTask(void const * argument);
void RemoteTaskInit(void);
void RemoteDataProcess(uint8_t *pData);
void MouseKeyControlProcess(Mouse *mouse, Key *key);
void RemoteControlProcess(Remote *rc);

#endif
