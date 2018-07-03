/**
  ******************************************************************************
  * File Name          : utilities_tim.h
  * Description        : ms计数定时器
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 使用1ms定时器获得上电后ms单位的时间
	* 
	* FreeRTOS提供了xTaskGetTickCount()，可以实现相同功能
  ******************************************************************************
  */
#ifndef UTILITIES_TIM_H
#define UTILITIES_TIM_H

#include <stdint.h>
void fw_userTimeEnable(void);
uint64_t fw_getTimeMicros(void);

#endif
