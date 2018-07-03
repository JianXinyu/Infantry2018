/**
  ******************************************************************************
  * File Name          : tasks_timed.h
  * Description        : 2ms定时任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 2ms定时
	* 通过count可以获得500ms,1s等定时任务
	* 状态机切换，串口定时输出，看门狗等
  ******************************************************************************
  */
#ifndef FRAMEWORK_TASKS_CMCONTROL_H
#define FRAMEWORK_TASKS_CMCONTROL_H

#include "cmsis_os.h"

void CMControlInit(void);
void Timer_2ms_lTask(void const * argument);
void WorkStateFSM(void);
void WorkStateSwitchProcess(void);
void getJudgeState(void);
void CMControlLoop(void);
void RuneShootControl(void);


//initiate status: 
typedef enum
{
	PREPARE_STATE,     	//上电准备状态
	NORMAL_STATE,		    //正常状态
	STOP_STATE,         //停止状态
	RUNE_STATE          //大符状态
}WorkState_e;


WorkState_e GetWorkState(void);


#define PID_SHOOT_MOTOR_SPEED      (30)
#define CHASSIS_SPEED_ATTENUATION   (1.30f)
#define PREPARE_TIME_TICK_MS 3500      //prapare time in ms*2
#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0.9f,\
	0.0f,\
	8.0f,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}
#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT_old \
{\
	0,\
	0,\
	{0,0},\
	1.2f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	6.5f,\
	0.0f,\
	1.0f,\
	0,\
	0,\
	0,\
	4900,\
	3500,\
	1500,\
	0,\
	4950,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}
#define CHASSIS_MOTOR_SPEED_PID_DEFAULT_old \
{\
	0,\
	0,\
	{0,0},\
	220.f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	4900,\
	3500,\
	1500,\
	0,\
	4950,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}


#endif

