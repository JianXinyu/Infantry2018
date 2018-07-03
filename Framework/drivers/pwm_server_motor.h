/**
  ******************************************************************************
  * File Name          : pwm_server_motor.h
  * Description        : 舵机驱动
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 初始化，设定角度
  ******************************************************************************
  */
#ifndef _pwm_server_motor_
#define _pwm_server_motor_
#include "stm32f4xx_hal.h"
#include "main.h" 
#include "tim.h"

//motorIndex: 
void pwm_server_motor_init(uint8_t motorIndex);
void pwm_server_motor_deinit(uint8_t motorIndex);
//angle:0~180 in degrees
void pwm_server_motor_set_angle(uint8_t motorIndex,float angle);

#endif
