/**
  ******************************************************************************
  * File Name          : tasks_platemotor.c
  * Description        : 拨盘电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 定时循环
	* 编码器模式对编码器脉冲计数
	* PWM波控制速度
  ******************************************************************************
  */
#include <tim.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <cmsis_os.h>
#include <task.h>
#include <usart.h>
#include "tasks_timed.h"
#include "pid_Regulator.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartrc_user.h"
#include "tasks_remotecontrol.h"
#include "application_motorcontrol.h"
#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"
#include "utilities_debug.h"
#include "rtos_semaphore.h"
#include "rtos_task.h"
#include "peripheral_define.h"
#include "drivers_platemotor.h"
#include "tasks_platemotor.h"
#include "drivers_uartjudge_low.h"

PID_Regulator_t ShootMotorPositionPID = SHOOT_MOTOR_POSITION_PID_DEFAULT;      //shoot motor
PID_Regulator_t ShootMotorSpeedPID = SHOOT_MOTOR_SPEED_PID_DEFAULT;

extern FrictionWheelState_e friction_wheel_stateZY;
static int s_count_bullet = 0;
int stuck = 0;	//卡弹标志位，未卡弹为false，卡弹为true
LaunchMode_e launchMode = SINGLE_MULTI;
extern float plateAngleTarget;
extern float plateRealAngle;
float nowPlateAngle;

void PlateMotorTask(void const * argument)
{
	int RotateAdd = 0;
	//int Stuck = 0;
	int32_t last_fdb = 0x0;
	int32_t this_fdb = 0x0;
	portTickType xLastWakeTimeQZK;
	xLastWakeTimeQZK = xTaskGetTickCount();
	static int s_count_1s = 0;
	static int s_stuck_cnt = 0;
	
	while(1)
	{
		s_count_1s++;
		if(s_count_1s == 500)
		{
			s_count_1s = 0;
			s_count_bullet = 0;
		}
		if(GetInputMode()==KEY_MOUSE_INPUT)//键鼠模式下直接在数据处理程序中实现// && Stuck==0
		{
			//ShootMotorPositionPID.ref = ShootMotorPositionPID.ref+OneShoot;//打一发弹编码器输出脉冲数
			//遥控器一帧14ms，此任务循环7次，最终是打了7发
			//ShootOneBullet();
		}

	//遥控器输入模式下，只要处于发射态，就一直转动
		if(GetShootState() == SHOOTING && GetInputMode() == REMOTE_INPUT) //&& Stuck==0
		{
			RotateAdd += 4;
			//fw_printfln("ref = %f",ShootMotorPositionPID.ref);
			if(RotateAdd>OneShoot)
			{
				ShootOneBullet();
				RotateAdd = 0;
			}
		}
		else if(GetShootState() == NOSHOOTING && GetInputMode() == REMOTE_INPUT)
		{
			RotateAdd = 0;
		}

		//卡弹检测
		//当参考值和反馈值长时间保持较大差别时，判定卡弹
		if(ShootMotorPositionPID.ref-ShootMotorPositionPID.fdb>OneShoot*3 || ShootMotorPositionPID.ref-ShootMotorPositionPID.fdb<OneShoot*(-3))
		{
			++s_stuck_cnt;
			if(s_stuck_cnt>250)
			{
				s_stuck_cnt = 0;
				stuck = 1;
			}
		}
		else
		{
			s_stuck_cnt = 0;
		}
		
		if(stuck == 1)
		{
			stuck = 0;
			ShootRefModify();
		}
		
		if(GetFrictionState()==FRICTION_WHEEL_ON||friction_wheel_stateZY==FRICTION_WHEEL_ON)//拨盘转动前提条件：摩擦轮转动GetFrictionState()==FRICTION_WHEEL_ON,张雁加后面的条件
		{
			this_fdb = GetQuadEncoderDiff(); 
			
			//fw_printfln("last_fdb = %d",last_fdb);
			//fw_printfln("this_fdb = %d",this_fdb);
			
			if(this_fdb<last_fdb-10000 && getPlateMotorDir()==FORWARD)	//cnt寄存器溢出判断 正转
			{
				ShootMotorPositionPID.fdb = ShootMotorPositionPID.fdb+(65536+this_fdb-last_fdb);
			}
			else if(this_fdb>last_fdb+10000 && getPlateMotorDir()==REVERSE)	//cnt寄存器溢出判断 反转
			{
				ShootMotorPositionPID.fdb = ShootMotorPositionPID.fdb-(65536-this_fdb+last_fdb);
			}
			else if((this_fdb-last_fdb)<500 && (this_fdb-last_fdb)>-500)
				ShootMotorPositionPID.fdb = ShootMotorPositionPID.fdb + this_fdb-last_fdb;

			last_fdb = this_fdb;
			//fw_printfln("fdb = %f",ShootMotorPositionPID.fdb);
			ShootMotorPositionPID.Calc(&ShootMotorPositionPID);
	
			if(ShootMotorPositionPID.output<0) //反转
			{
				setPlateMotorDir(REVERSE);
				ShootMotorPositionPID.output = -ShootMotorPositionPID.output;
			}
			else
				setPlateMotorDir(FORWARD);
			
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ShootMotorPositionPID.output);
		}
		
		else
		{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);//摩擦轮不转，立刻关闭拨盘
		}
		
		vTaskDelayUntil( &xLastWakeTimeQZK, ( 2 / portTICK_RATE_MS ) );//这里进入阻塞态等待2ms
	}
}

//uint8_t cdflag = 0;
//uint8_t burstflag = 0;
//extern float realBulletSpeed;
//extern uint16_t remainHeat;
//extern uint8_t burst;
//extern JudgeState_e JUDGE_State;
//void ShootOneBullet()
//{
//	s_count_bullet ++;
//	if(s_count_bullet <= 2)
//	{
//	//ShootMotorPositionPID.ref = ShootMotorPositionPID.ref+OneShoot;
//	}	
//		if(JUDGE_State == ONLINE && remainHeat < (realBulletSpeed+5) && !burst)cdflag = 1;
//		else cdflag = 0;
//		if((plateRealAngle - plateAngleTarget <= 50) && burst)
//			if(((!cdflag) || JUDGE_State == OFFLINE))plateAngleTarget -= 360;
//		if((plateRealAngle - plateAngleTarget <= 1))
//		{
//			if(((!cdflag) || JUDGE_State == OFFLINE))plateAngleTarget -= 60;
//		}
//}

void ShootRefModify()
{
	while(ShootMotorPositionPID.ref>ShootMotorPositionPID.fdb && ShootMotorPositionPID.ref>2*OneShoot)
		ShootMotorPositionPID.ref = ShootMotorPositionPID.ref - 2*OneShoot;	
}

	

int32_t GetQuadEncoderDiff(void)
{
  int32_t cnt = 0;    
	cnt = __HAL_TIM_GET_COUNTER(&htim5) - 0x0;
	return cnt;
}

void setLaunchMode(LaunchMode_e lm)
{
	launchMode = lm;
}

LaunchMode_e getLaunchMode()
{
	return launchMode;
}

void toggleLaunchMode()
{
	if(getLaunchMode() == SINGLE_MULTI)
		setLaunchMode(CONSTENT_4);
	else
		setLaunchMode(SINGLE_MULTI);
}
