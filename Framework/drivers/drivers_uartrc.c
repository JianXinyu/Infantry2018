/**
  ******************************************************************************
  * File Name          : drivers_uartrc.c
  * Description        : 遥控器串口
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 串口初始化
	* 串口数据读取
	* 数据处理函数
  ******************************************************************************
  */
#include "drivers_uartrc_user.h"
#include "drivers_uartrc_low.h"
#include "drivers_led_user.h"
#include "peripheral_laser.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "rtos_semaphore.h"
#include "drivers_ramp.h"
#include "peripheral_tim.h"
#include <stdlib.h>
#include <math.h>
#include "utilities_debug.h"
#include  "tim.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartrc_user.h"
#include "peripheral_define.h"
#include "drivers_uartupper_low.h"
#include "drivers_uartupper_user.h"
#include "stm32f4xx_hal_uart.h"
#include "tasks_platemotor.h"
#include "drivers_uartjudge_low.h"
NaiveIOPoolDefine(rcUartIOPool, {0});

void InitRemoteControl(){
	//遥控器DMA接收开启(一次接收18个字节)
	if(HAL_UART_Receive_DMA(&RC_UART, IOPool_pGetWriteData(rcUartIOPool)->ch, 18) != HAL_OK){
			Error_Handler();
	} 
//	__HAL_UART_ENABLE_IT(&RC_UART, UART_FLAG_IDLE);//空闲中断方式更优，未调通
	RemoteTaskInit();
}

void rcUartRxCpltCallback(){
	static portBASE_TYPE xHigherPriorityTaskWoken;
	 xHigherPriorityTaskWoken = pdFALSE; 
	//释放信号量
   xSemaphoreGiveFromISR(xSemaphore_rcuart, &xHigherPriorityTaskWoken);
	//切换上下文，RTOS提供
	//当在中断外有多个不同优先级任务等待信号量时
	//在退出中断前进行一次切换上下文
	//这里无用
	if( xHigherPriorityTaskWoken == pdTRUE ){
   portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	 }
 }



RC_Ctl_t RC_CtrlData;   //remote control data
ChassisSpeed_Ref_t ChassisSpeedRef;
Gimbal_Ref_t GimbalRef; 
FrictionWheelState_e g_friction_wheel_state = FRICTION_WHEEL_OFF; 
uint16_t nowDuty = 1350;
extern float realBulletSpeed;

volatile Shoot_State_e shootState = NOSHOOTING; 
InputMode_e inputmode = REMOTE_INPUT;  

unsigned int zyLeftPostion; //大符用左拨杆位置
 
static uint32_t RotateCNT = 0;	//长按连发计数
static uint16_t CNT_1s = 75;	//用于避免四连发模式下两秒内连射8发过于密集的情况
static uint16_t CNT_250ms = 18;	//用于点射模式下射频限制
extern uint8_t burst;
extern float friction_speed;
float now_friction_speed = 1500;

RampGen_t frictionRamp = RAMP_GEN_DAFAULT;  
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;   

void RemoteTaskInit()
{
  /*斜坡初始化，copy from官方程序，实现被封装在RMLib*/
	frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	frictionRamp.ResetCounter(&frictionRamp);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
  /*底盘速度初始化*/
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
  /*摩擦轮*/
	SetFrictionState(FRICTION_WHEEL_OFF);
}
/*拨杆数据处理*/
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
	static uint32_t switch_cnt = 0;

	/* ×îÐÂ×´Ì¬Öµ */
	sw->switch_value_raw = val;
	sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

	/* È¡×îÐÂÖµºÍÉÏÒ»´ÎÖµ */
	sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
	(sw->switch_value_buf[sw->buf_index]);


	/* ×îÀÏµÄ×´Ì¬ÖµµÄË÷Òý */
	sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	/* ºÏ²¢Èý¸öÖµ */
	sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;	

	/* ³¤°´ÅÐ¶Ï */
	if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}

	if(switch_cnt >= 40)
	{
		sw->switch_long_value = sw->switch_value_buf[sw->buf_index]; 	
	}

	//Ë÷ÒýÑ­»·
	sw->buf_last_index = sw->buf_index;
	sw->buf_index++;		
	if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		sw->buf_index = 0;	
	}			
}
//return the state of the remote 0:no action 1:action 
uint8_t IsRemoteBeingAction(void)
{
	return (abs(ChassisSpeedRef.forward_back_ref)>=10 || abs(ChassisSpeedRef.left_right_ref)>=10 || fabs(GimbalRef.yaw_speed_ref)>=10 || fabs(GimbalRef.pitch_speed_ref)>=10);
}
/*取得右上角拨杆数据*/
void SetInputMode(Remote *rc)
{
	if(rc->s2 == 1)
	{
		inputmode = REMOTE_INPUT;
	}
	else if(rc->s2 == 3)
	{
		inputmode = KEY_MOUSE_INPUT;
	}
	else if(rc->s2 == 2)
	{
		inputmode = STOP;
	}	
}

//张雁大符
void zySetLeftMode(Remote *rc)
{
	if(rc->s1 == 1)
	{
		zyLeftPostion = 1;
	}
	else if(rc->s1 == 3)
	{
		zyLeftPostion = 3;
	}
	else if(rc->s1 == 2)
	{
		zyLeftPostion = 2;
	}	
}
unsigned int zyGetLeftPostion()
{
	return zyLeftPostion;
}

InputMode_e GetInputMode()
{
	return inputmode;
}

/*
input: RemoteSwitch_t *sw, include the switch info
*/
#ifdef INFANTRY_1
#define FRICTION_WHEEL_MAX_DUTY             1500
#endif
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val) 
{
	switch(g_friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)   
			{
				SetShootState(NOSHOOTING);
				frictionRamp.ResetCounter(&frictionRamp);
				g_friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
				LASER_ON(); 
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   
			{
				LASER_OFF();//zy0802
				SetShootState(NOSHOOTING);
				SetFrictionWheelSpeed(1000);
				g_friction_wheel_state = FRICTION_WHEEL_OFF;
				frictionRamp.ResetCounter(&frictionRamp);
			}
			else
			{
				/*斜坡函数必须有，避免电流过大烧坏主控板*/
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
				//SetFrictionWheelSpeed(1000);
				//g_friction_wheel_state = FRICTION_WHEEL_ON; 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					g_friction_wheel_state = FRICTION_WHEEL_ON; 	
				}
				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   
			{
				LASER_OFF();//zy0802
				g_friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				SetShootState(NOSHOOTING);
			}
			else if(sw->switch_value_raw == 2)
			{
				SetShootState(SHOOTING);
			}
			else
			{
				SetShootState(NOSHOOTING);
			}					 
		} break;				
	}
}
	 
void MouseShootControl(Mouse *mouse,Key *key)
{
	++CNT_1s;
	++CNT_250ms;
	static int16_t closeDelayCount = 0;   
//	if(key->v == 2048)//z
//	{
//		nowDuty = 1200;
//		SetShootState(NOSHOOTING);
//		frictionRamp.ResetCounter(&frictionRamp);
//		g_friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
//		LASER_ON(); 
//		closeDelayCount = 0;
//		realBulletSpeed = 11.0f;
//	}
//	if(key->v == 4096)//x
//	{
//		nowDuty = 1350;
//		SetShootState(NOSHOOTING);
//		frictionRamp.ResetCounter(&frictionRamp);
//		g_friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
//		LASER_ON(); 
//		closeDelayCount = 0;
//		realBulletSpeed = 22.0f;
//	}
//	if(key->v == 8192)//c
//	{
//		nowDuty = 1500;
//		SetShootState(NOSHOOTING);
//		frictionRamp.ResetCounter(&frictionRamp);
//		g_friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
//		LASER_ON(); 
//		closeDelayCount = 0;
//		realBulletSpeed = 28.0f;
//	}
	switch(g_friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(mouse->last_press_r == 0 && mouse->press_r == 1)   
			{
//				SetShootState(NOSHOOTING);
//				frictionRamp.ResetCounter(&frictionRamp);
//				g_friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
//				LASER_ON(); 
//				closeDelayCount = 0;
				SetShootState(NOSHOOTING);
				g_friction_wheel_state = FRICTION_WHEEL_ON;
				friction_speed = now_friction_speed;
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(mouse->press_r == 1)
			{
				closeDelayCount++;
			}
			else
			{
				closeDelayCount = 0;
			}
			if(closeDelayCount>50)   
			{
				LASER_OFF();//zy0802
				g_friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				SetShootState(NOSHOOTING);
				closeDelayCount = 0;
			}
			else
			{
		    /*摩擦轮转速修改 FRICTION_WHEEL_MAX_DUTY*/
				SetFrictionWheelSpeed(1000 + (nowDuty-1000)*frictionRamp.Calc(&frictionRamp)); 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					g_friction_wheel_state = FRICTION_WHEEL_ON; 	
				}
				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(mouse->press_r == 1)
			{
				closeDelayCount++;
			}
			else
			{
				closeDelayCount = 0;
			}
			if(closeDelayCount>50)   //
			{
//				LASER_OFF();//zy0802
//				g_friction_wheel_state = FRICTION_WHEEL_OFF;				  
//				SetFrictionWheelSpeed(1000); 
//				frictionRamp.ResetCounter(&frictionRamp);
//				SetShootState(NOSHOOTING);
				LASER_OFF();//zy0802
				g_friction_wheel_state = FRICTION_WHEEL_OFF;
				friction_speed = 0;
				closeDelayCount = 0;
			}			
			else if(mouse->last_press_l == 0 && mouse->press_l== 1)  //检测鼠标左键单击动作
			{
				SetShootState(SHOOTING);
				if(getLaunchMode() == SINGLE_MULTI && GetFrictionState()==FRICTION_WHEEL_ON)		//单发模式下，点一下打一发
				{
					if(CNT_250ms>17)
					{
						CNT_250ms = 0;
						ShootOneBullet();
					}
				}
				else if(getLaunchMode() == CONSTENT_4 && GetFrictionState()==FRICTION_WHEEL_ON)	//四连发模式下，点一下打四发
				{
					
					if(CNT_1s>75 || burst)
					{
						CNT_1s = 0;
						ShootOneBullet();
						ShootOneBullet();
						ShootOneBullet();
						ShootOneBullet();
					}
				}
			}
			else if(mouse->last_press_l == 0 && mouse->press_l== 0)	//松开鼠标左键的状态
			{
				SetShootState(NOSHOOTING);	
				RotateCNT = 0;			
			}			
			else if(mouse->last_press_l == 1 && mouse->press_l== 1 && getLaunchMode() == SINGLE_MULTI)//单发模式下长按，便持续连发
			{
				RotateCNT+=1;
				if(RotateCNT>=1)
				{
					ShootOneBullet();
					RotateCNT = 0;
				}
				
			}
				
		} break;				
	}	
	mouse->last_press_r = mouse->press_r;
	mouse->last_press_l = mouse->press_l;
}

uint8_t cdflag = 0;
uint8_t burstflag = 0;
static int s_count_bullet = 0;
extern float plateAngleTarget;
extern float plateRealAngle;
extern float realBulletSpeed;
extern uint16_t remainHeat;
extern uint8_t burst;
extern JudgeState_e JUDGE_State;
void ShootOneBullet()
{
	s_count_bullet ++;
	if(s_count_bullet <= 2)
	{
	//ShootMotorPositionPID.ref = ShootMotorPositionPID.ref+OneShoot;
	}	
		if(JUDGE_State == ONLINE && remainHeat < (realBulletSpeed+5) && !burst)cdflag = 1;
		else cdflag = 0;
		if((plateRealAngle - plateAngleTarget <= 50) && burst)
			if(((!cdflag) || JUDGE_State == OFFLINE))plateAngleTarget -= 360;
		if((plateRealAngle - plateAngleTarget <= 1))
		{
			if(((!cdflag) || JUDGE_State == OFFLINE))plateAngleTarget -= 60;
		}
}

Shoot_State_e GetShootState()
{
	return shootState;
}

void SetShootState(Shoot_State_e v)
{
	shootState = v;
}

FrictionWheelState_e GetFrictionState()
{
	return g_friction_wheel_state;
}

void SetFrictionState(FrictionWheelState_e v)
{
	g_friction_wheel_state = v;
}
void SetFrictionWheelSpeed(uint16_t x)
{
	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_1, x);
	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_2, x);
}
Shoot_Mode_e shootMode = MANUL;

Shoot_Mode_e GetShootMode()
{
	return shootMode;
}
void SetShootMode(Shoot_Mode_e v)
{
	shootMode = v;
}

Emergency_Flag emergency_Flag = NORMAL;

Emergency_Flag GetEmergencyFlag()
{
	return emergency_Flag;
}

void SetEmergencyFlag(Emergency_Flag v)
{
	emergency_Flag = v;
}

Move_Speed_e movespeed = NORMAL_s;

Move_Speed_e GetMoveSpeed()
{
	return movespeed;
}

void SetMoveSpeed(Move_Speed_e v)
{
	movespeed = v;
}

Slab_Mode_e slabmode = CLOSE;

Slab_Mode_e GetSlabState()
{
	return slabmode;
}

void SetSlabState(Slab_Mode_e v)
{
	slabmode = v;
}
