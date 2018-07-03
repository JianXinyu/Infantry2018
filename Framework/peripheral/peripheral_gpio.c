/**
  ******************************************************************************
  * File Name          : peripheral_gpio.c
  * Description        : GPIO外部中断处理
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * MPU6050有数据可以读取时，触发外部中断，更新数据
  ******************************************************************************
  */
#include <gpio.h>
#include "peripheral_gpio.h"
#include "utilities_debug.h"
#include "rtos_semaphore.h"
#include "rtos_init.h"


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//fw_printfln("HAL_GPIO_EXTI_Callback %d", GPIO_Pin);
	switch(GPIO_Pin){
		case MPU_INT:
			//fw_printfln("MPU_INT");
			if(g_bInited == 1){
				osSemaphoreRelease(refreshMPU6500SemaphoreHandle);
			}
			break;
		case IST_INT:
			//osSemaphoreRelease(refreshMPU6050SemaphoreHandle);
			break;
		default:
			fw_Warning();
	}
}
