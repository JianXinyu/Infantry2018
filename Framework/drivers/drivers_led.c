/**
  ******************************************************************************
  * File Name          : drivers_led.c
  * Description        : LED闪烁任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * LED任务运行在最低优先级，可以用来判断程序是否宕机
	* 也可用来显示运行情况(todo)
  ******************************************************************************
  */
	
#include <cmsis_os.h>
#include <gpio.h>
#include "drivers_led_user.h"
#include "drivers_led_low.h"
#include "peripheral_gpio.h"
#include "iwdg.h"


#define ledGreenOn() HAL_GPIO_WritePin(GREEN_PIN, GREEN_GPIO_PORT, GPIO_PIN_RESET)
#define ledGreenOff() HAL_GPIO_WritePin(GREEN_PIN, GREEN_GPIO_PORT, GPIO_PIN_SET)
#define ledRedOn() HAL_GPIO_WritePin(RED_PIN, RED_GPIO_PORT, GPIO_PIN_RESET)
#define ledRedOff() HAL_GPIO_WritePin(RED_PIN, RED_GPIO_PORT, GPIO_PIN_SET)

LedStatus_t ledGreenStatus = blink, ledRedStatus = blink;

void ledGreenTask(void const * argument){
	while(1){
		if(ledGreenStatus == on){
			ledGreenOn();
		}else if(ledGreenStatus == off){
			ledGreenOff();
		}else if(ledGreenStatus == blink){
			ledGreenOn();
			osDelay(777);
			ledGreenOff();
			osDelay(222);
		}
	}
}

void ledRedTask(void const * argument){
	while(1){
		if(ledRedStatus == on){
			ledRedOn();
		}else if(ledRedStatus == off){
			ledRedOff();
		}else if(ledRedStatus == blink){
			ledRedOn();
			osDelay(66);
			ledRedOff();
			osDelay(88);
			#ifdef OFFBOARD
			HAL_IWDG_Refresh(&hiwdg);
			#endif
		}
	}
}
