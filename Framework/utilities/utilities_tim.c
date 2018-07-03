/**
  ******************************************************************************
  * File Name          : utilities_tim.c
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
#include "utilities_tim.h"

#include "peripheral_define.h"
#include "tim.h"

uint64_t timeMicros = 0;
uint64_t fw_getTimeMicros(void){
	return timeMicros + __HAL_TIM_GET_COUNTER(&USER_TIM);
}

void fw_userTimeEnable(void){
	HAL_TIM_Base_Start_IT(&USER_TIM);
	//__HAL_TIM_ENABLE(&USER_TIM);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &USER_TIM){
		timeMicros += 0xFFFF;
	}
}
