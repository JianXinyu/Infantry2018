/**
  ******************************************************************************
  * File Name          : rtos_InitInfantry.c
  * Description        : FreeRTOS初始化
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 对于板上所有外设如定时器，串口，蜂鸣器，CAN总线，MPU6050等进行必要的初始化

  ******************************************************************************
  */
#include <tim.h>
#include <stdbool.h>
#include "rtos_init.h"
#include "drivers_platemotor.h"
#include "application_motorcontrol.h"
#include "utilities_debug.h"
#include "drivers_canmotor_low.h"
#include "peripheral_tim.h"
#include "drivers_uartupper_low.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartgyro_low.h"
#include "tasks_timed.h"
#include "drivers_imu_low.h"
#include "utilities_tim.h"
#include "drivers_buzzer_low.h"
#include "drivers_uartjudge_low.h"
#include "drivers_servo.h"
#include "iwdg.h"
bool g_bInited = 0;//匈牙利命名法,g_表示全局变量，b表示布尔型，Inited是否初始化完成
void rtos_InitInfantry()
{
	playMusicWhenInit();//上电音乐
	MX_IWDG_Init();
	InitMPU6500();
	InitIST8310();//初始化IMU
	InitJudgeUart();//初始化裁判系统读取串口
	InitGyroUart();//初始化外接陀螺仪读取串口
	InitManifoldUart();//初始化妙算Manifold通信串口，用来做大神符、自动瞄准
	InitRemoteControl();//初始化遥控器控制，接收机串口
	InitServoUart();//初始化舵机通信串口
	HAL_IWDG_Refresh(&hiwdg);
	CMControlInit();//底盘PID初始化，copy from官方开源程序
	InitCanReception();//初始化CAN接收(配置CAN过滤器)
	
	plateMotorInit();//初始化拨盘电机(电机PWM，编码器计数)
  InitUserTimer();//初始化用户定时器：摩擦轮PWM，舵机PWM
//	Init_Quaternion();//四元数初始化
	fw_printfln("init success");//串口发送成功初始化成功 printf line
}


