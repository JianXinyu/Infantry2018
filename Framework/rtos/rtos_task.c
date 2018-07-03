/**
  ******************************************************************************
  * File Name          : rtos_task.c
  * Description        : 添加FreeRTOS任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 添加RTOS任务；LED灯
	*								蜂鸣器
	*								IMU
	*								遥控器接收机
	*								妙算通信
	*								CM(Chasis Motor)GM(Gimbal Motor)控制任务
	*								定时循环任务
	*								CAN发送任务
  ******************************************************************************
  */
#include "rtos_task.h"
#include "rtos_init.h"

#include "drivers_buzzer_low.h"
#include "drivers_imu_low.h"

//#include "utilities_iopool.h"
#include "drivers_led_low.h"
#include "tasks_remotecontrol.h"
#include "tasks_upper.h"
#include "tasks_timed.h"
#include "drivers_canmotor_low.h"
#include "tasks_motor.h"
#include "drivers_sonar_low.h"
#include "tasks_platemotor.h"
//#include "drivers_mpu6050_low.h"
//#include "tasks_mpu6050.h"

//#include "tasks_testtasks.h"

#include "utilities_debug.h"

osThreadId ledGreenTaskHandle;
osThreadId ledRedTaskHandle;
osThreadId buzzerTaskHandle;
//IMU
osThreadId printIMUTaskHandle;
//UART
osThreadId RControlTaskHandle;
osThreadId getCtrlUartTaskHandle;
//Motor
osThreadId GMControlTaskHandle;
osThreadId TimerTaskHandle;
osThreadId PlateTaskHandle;

osThreadId CMGMCanTransmitTaskHandle;
osThreadId AMCanTransmitTaskHandle;

osThreadId sonarTaskHandle;

//extern osThreadId testFlashTaskHandle;

//#include "drivers_flash.h"
//osThreadId testFlashTaskHandle;

void rtos_AddThreads()
{
/**
  ******************************************************************************
  * osThreadDef(线程名，线程函数，优先级，线程数量，栈空间)
	* 注：注意栈空间不够用会导致宕机，FreeRTOS提供函数查看栈剩余以及溢出回调函数
	* 注：线程、任务对于FreeRTOS是同一概念，推荐使用任务概念
  ******************************************************************************
  */
//红绿LED闪烁任务，最低优先级
	osThreadDef(ledGreenTask, ledGreenTask, osPriorityNormal, 0, 128);
  ledGreenTaskHandle = osThreadCreate(osThread(ledGreenTask), NULL);
	osThreadDef(ledRedTask, ledRedTask, osPriorityNormal, 0, 128);
  ledRedTaskHandle = osThreadCreate(osThread(ledRedTask), NULL);
	
//蜂鸣器任务，暂时无用
//	osThreadDef(buzzerTask, buzzerTask, osPriorityNormal, 0, 128);
//  buzzerTaskHandle = osThreadCreate(osThread(buzzerTask), NULL);
	
//IMU数据获取，角速度(用于云台电机速度反馈)，角度(需四元数解算)
	osThreadDef(IMUTask, IMUTask, osPriorityHigh, 0, 256);
  printIMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

//遥控器控制任务	
	osThreadDef(RControlTask, RControlTask, osPriorityHigh , 0, 256);//zy0512
  RControlTaskHandle = osThreadCreate(osThread(RControlTask), NULL);

//妙算通信任务：大神符，自动瞄准
	osThreadDef(ManifoldUartTask, ManifoldUartTask, osPriorityAboveNormal, 0, 256);
  getCtrlUartTaskHandle = osThreadCreate(osThread(ManifoldUartTask), NULL);

//CM(ChasisMotor)底盘电机GM(Gimbla)云台电机控制任务
	osThreadDef(GMC_Task, CMGMControlTask, osPriorityAboveNormal, 0, 1024);
  GMControlTaskHandle = osThreadCreate(osThread(GMC_Task), NULL);
	
//拨盘电机任务 
//	osThreadDef(Plate_Task, PlateMotorTask, osPriorityAboveNormal, 0, 300);
//  PlateTaskHandle = osThreadCreate(osThread(Plate_Task), NULL);
//2ms定时任务，状态机切换，调试信息输出等

	osThreadDef(Timer_Task, Timer_2ms_lTask, osPriorityAboveNormal, 0, 1024);//zy512
  TimerTaskHandle = osThreadCreate(osThread(Timer_Task), NULL);

}
