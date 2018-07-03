/**
  ******************************************************************************
  * File Name          : tasks_platemotor.h
  * Description        : 
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef FRAMEWORK_TASKS_PLATECONTROL_H
#define FRAMEWORK_TASKS_PLATECONTROL_H

#include "cmsis_os.h"
#include "tasks_timed.h"
#include "pid_regulator.h"


#define OneShoot (1011) //5…»≤¶≈Ã, 722 7…»≤¶≈Ã

void PlateMotorTask(void const * argument);
void ShootOneBullet(void);
void ShootRefModify(void);
int32_t GetQuadEncoderDiff(void);

typedef enum
{
	SINGLE_MULTI,
	CONSTENT_4
}LaunchMode_e;

void setLaunchMode(LaunchMode_e launchMode);
LaunchMode_e getLaunchMode(void);
void toggleLaunchMode(void);

#define SHOOT_MOTOR_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	7.5f,\
	0.05f,\
	0.6f,\
	0,\
	0,\
	0,\
	4900,\
	3500,\
	1500,\
	0,\
	900,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define SHOOT_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	50.0f,\
	0.5f,\
	0.0f,\
	0,\
	0,\
	0,\
	1000,\
	200,\
	100,\
	0,\
	4950,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#endif
