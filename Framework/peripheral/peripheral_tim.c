/**
  ******************************************************************************
  * File Name          : peripheral_tim.c
  * Description        : 用户自定义定时器
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 用户自定义定时器初始化：
	*	摩擦轮、舵机PWM所需定时器
  ******************************************************************************
  */
#include "peripheral_tim.h"
#include "cmsis_os.h"
#include "tim.h"
#include "peripheral_define.h"
#include "pwm_server_motor.h"
#include "drivers_uartrc_user.h"
#include "tasks_motor.h"
void InitUserTimer(void)
{

	HAL_TIM_PWM_Start(&FRICTION_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&FRICTION_TIM, TIM_CHANNEL_2);
	
	//弹舱盖舵机，已取消
	pwm_server_motor_init(0);
#ifdef INFANTRY_1
				pwm_server_motor_set_angle(0,0.f);
#endif
#ifdef INFANTRY_4
				pwm_server_motor_set_angle(0,0.f);
#endif
#ifdef INFANTRY_5
				pwm_server_motor_set_angle(0,0.f);
#endif
				SetSlabState(OPEN);
}
