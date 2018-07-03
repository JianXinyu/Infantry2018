/**
  ******************************************************************************
  * File Name          : peripheral_define.h
  * Description        : 外设句柄define
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 提高程序可读性
	* 定时器、串口、CAN
  ******************************************************************************
  */
#ifndef PERIPHERAL_DEFINE_H
#define PERIPHERAL_DEFINE_H

//#include "peripheral_define.h"

#define BUZZER_TIM htim12

#define USER_TIM htim6

#define RC_UART huart1
#define DEBUG_UART huart3//huart2为蓝牙串口位
#define MANIFOLD_UART huart3
#define JUDGE_UART huart6
#define SERVO_UART huart2
#define GYRO_UART huart7

//#define CMGMMOTOR_CAN hcan1
//#define ZGYRO_CAN hcan2
#define CMCAN hcan2
#define GM_FRICTION_PLATE_CAN hcan1


#define FRICTION_TIM htim12
#define PLATE_ENCODER_TIM htimx
#define PLATE_MOTOR_TIM htimx

#endif
