/**
  ******************************************************************************
  * File Name          : tasks_motor.h
  * Description        : 电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 使用define对于不同车辆的参数进行区分
	* 主要为云台初始位置时的编码器位置
	* 以及其他可能存在的性能差异
  ******************************************************************************
  */
#ifndef TASKS_MOTOR_H
#define TASKS_MOTOR_H

///*通过define使一套程序使用多台车*/
//#define INFANTRY_1
#define INFANTRY_4
//#define INFANTRY_5                                                                           

void CMGMControlTask(void const * argument);
void ControlYaw(void);
void ControlPitch(void);
void ControlRotate(void);
void ControlCMFL(void);
void ControlCMFR(void);
void ControlCMBL(void);
void ControlCMBR(void);
void ControlPLATE(void);
void ControlLFRICTION(void);
void ControlRFRICTION(void);
#endif
