/**
  ******************************************************************************
  * File Name          : tasks_cmcontrol.c
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
#include "tasks_platemotor.h"
#include "application_motorcontrol.h"
#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"
#include "utilities_debug.h"
#include "rtos_semaphore.h"
#include "rtos_task.h"
#include "peripheral_define.h"
#include "drivers_platemotor.h"
#include "application_waveform.h"
#include "drivers_uartjudge_low.h"
#include "drivers_uartupper_user.h"
#include "utilities_minmax.h"
#include "drivers_ramp.h"
#include "peripheral_laser.h"
#include "drivers_uartrc_low.h"
#include "tasks_motor.h"//zy
#include <stdbool.h>
extern PID_Regulator_t CMRotatePID ; 
extern PID_Regulator_t CM1SpeedPID;
extern PID_Regulator_t CM2SpeedPID;
extern PID_Regulator_t CM3SpeedPID;
extern PID_Regulator_t CM4SpeedPID;



Shoot_State_e last_shoot_state = NOSHOOTING;
Shoot_State_e this_shoot_state = NOSHOOTING;
//uint32_t last_Encoder = 0;
//uint32_t this_Encoder = 0;
int flag = 0;

WorkState_e g_workState = PREPARE_STATE;
WorkState_e lastWorkState = PREPARE_STATE;
WorkState_e GetWorkState()
{
	return g_workState;
}
/*2ms定时任务*/

extern float ZGyroModuleAngle;
float ZGyroModuleAngleMAX;
float ZGyroModuleAngleMIN;
extern float yawRealAngle;
extern uint8_t g_isGYRO_Rested;
extern float pitchAngleTarget;
extern float pitchRealAngle;
extern float gYroZs;
extern float yawAngleTarget;
extern float yawRealAngle;

extern uint8_t JUDGE_STATE;

int mouse_click_left = 0;


FrictionWheelState_e friction_wheel_stateZY = FRICTION_WHEEL_OFF;

extern uint8_t JUDGE_Received;
extern uint8_t JUDGE_State;

static uint32_t s_time_tick_2ms = 0;


extern RampGen_t frictionRamp ;
extern uint8_t bShoot;
uint16_t zyShootTimeCount=0;
uint8_t zyRuneMode=0;
uint16_t checkRecTime=300;
Location_Number_s pRunePosition[3] = {{0,0},{0,0},{0,0}};
uint16_t checkKeyTime=500;
//#ifdef INFANTRY_5
////手动标定0点
//#define yaw_zero 2163//2200
//#define pitch_zero 3275
//#endif
//#ifdef INFANTRY_4
//#define yaw_zero 2806//2840
//#define pitch_zero 5009 
//#endif
//#ifdef INFANTRY_1
//#define yaw_zero 6025
//#define pitch_zero 6400
//#endif //p在前，y在后//张雁大符
float zeroGyro;
extern float gyroZAngle;

void Timer_2ms_lTask(void const * argument)
{
	//RTOS提供，用来做2ms精确定时
	//与后面的vTaskDelayUntil()配合使用
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	//countwhile来获得不同定时任务
	static int s_countWhile = 0;

	
//static int shootwhile = 0;
//unsigned portBASE_TYPE StackResidue; //栈剩余
	while(1)  
	{       
		
		WorkStateFSM();//状态机
	  WorkStateSwitchProcess();//状态机动作

		//陀螺仪复位计时
    if(s_time_tick_2ms == 2000)
		{
			//GYRO_RST();//给单轴陀螺仪将当前位置写零，注意需要一定的稳定时间
			zeroGyro = gyroZAngle;
		}            //在从STOP切换到其他状态时，s_time_tick_2ms清零重加，会重新复位陀螺仪

		getJudgeState();
		
		if(g_workState==RUNE_STATE)
		{
			if(bShoot==1)
			{
				if(zyShootTimeCount<1)
				{
					zyShootTimeCount++;
				}
				else if(zyShootTimeCount==1)
				{
					bShoot=0;
					ShootOneBullet();//拨盘啵一个
					zyShootTimeCount=0;
					
					
					checkRecTime=0;
				}
			}
		}
		if(checkRecTime<65534)
		{
			checkRecTime++;
		}
		if(checkKeyTime<65534)
		{
			checkKeyTime++;
		}
		RuneShootControl();
		
		
		
		if(s_countWhile >= 1000)//150 1000
		{//定时1s,发送调试信息
			s_countWhile = 0;
//			IOPool_getNextRead(GMYAWRxIOPool, 0); 
//			float tempYaw = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle-100) * 360 / 8192.0f;
//			NORMALIZE_ANGLE180(tempYaw);
//			fw_printfln("YawAngle= %f,targetYaw:%f", tempYaw,yawAngleTarget);
////		fw_printfln("YawAngle= %f", tempYaw);
////			IOPool_getNextRead(GMPITCHRxIOPool, 0); 
//			float tempPitch = -(IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle - 6400) * 360 / 8192.0f;
//			NORMALIZE_ANGLE180(tempPitch);
//			fw_printfln("PitchAngle= %f,targetPitch:%f", tempPitch,pitchAngleTarget);
//			fw_printfln("PitchAngle= %f", tempPitch);
			//fw_printfln("ZGyroModuleAngle:  %f",ZGyroModuleAngle);
//			fw_printfln("YawAngle= %d", IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle);
//			fw_printfln("PitchAngle= %d", IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle);
			/*****查看任务栈空间剩余示例*******/
			//		StackResidue = uxTaskGetStackHighWaterMark( GMControlTaskHandle );
			//		fw_printfln("GM%ld",StackResidue);
			if(JUDGE_State == OFFLINE)
			{
				fw_printfln("Judge not received");
			}
			else
			{
//				fw_printfln("Judge received");

			}
		}
		else
		{
			s_countWhile++;
		}
		
		vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_RATE_MS ) );//这里进入阻塞态等待2ms
	}
}
	

void CMControlInit(void)
{
//底盘电机PID初始化，copy from官方开源程序
	//ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}
/**********************************************************
*工作状态切换状态机
**********************************************************/

extern RemoteSwitch_t g_switch1; 
extern RC_Ctl_t RC_CtrlData; 
extern bool g_switchRead;

uint8_t waitRuneMSG[4] = {0xff, 0x00, 0x00, 0xfe};
uint8_t littleRuneMSG[4] = {0xff, 0x01, 0x00, 0xfe};
uint8_t bigRuneMSG[4] = {0xff, 0x02, 0x00, 0xfe};

void WorkStateFSM(void)
{
	lastWorkState = g_workState;
	s_time_tick_2ms ++;
	
	switch(g_workState)
	{
		case PREPARE_STATE:
		{
			if(GetInputMode() == STOP )
			{
				g_workState = STOP_STATE;
			}
			else if(s_time_tick_2ms > PREPARE_TIME_TICK_MS)
			{
				zyRuneMode=0;
				LASER_ON();
				g_workState = NORMAL_STATE;
			}			
		}break;
		case NORMAL_STATE:     
		{
//			fw_printfln("switch%d",g_switch1.switch_value1);
			if(GetInputMode() == STOP )
			{
				g_workState = STOP_STATE;
			}
			//ZY
			else if(GetInputMode() == KEY_MOUSE_INPUT
								&& (RC_CtrlData.key.v == 16384)&& g_switchRead == 1)
			{
				g_switchRead = 0;
				if(checkKeyTime>450)
				{
					checkKeyTime=0;
					zyRuneMode=0;
					LASER_ON();
					g_workState=RUNE_STATE;
				}
				
			}
			/*else if(GetInputMode() == KEY_MOUSE_INPUT
								&& (g_switch1.switch_value1 == REMOTE_SWITCH_CHANGE_3TO1 
										|| g_switch1.switch_value1 == REMOTE_SWITCH_CHANGE_3TO2
							||RC_CtrlData.key.v == 32768//B
							||RC_CtrlData.key.v == 1024//G
							||RC_CtrlData.key.v == 16384)//V
							&& g_switchRead == 1)
			{
				g_workState = RUNE_STATE;
				g_switchRead = 0;
				LASER_ON();
				zyRuneMode=0;
				if(g_switch1.switch_value1 == REMOTE_SWITCH_CHANGE_3TO1
					||RC_CtrlData.key.v == 1024)//小符
				{
					LASER_OFF();
					zyRuneMode=2;
					HAL_UART_Transmit(&MANIFOLD_UART , (uint8_t *)&littleRuneMSG, 4, 0xFFFF);
				}else if(g_switch1.switch_value1 == REMOTE_SWITCH_CHANGE_3TO2
					||RC_CtrlData.key.v == 32768)//大符
				{
					LASER_OFF();
					zyRuneMode=3;
					HAL_UART_Transmit(&MANIFOLD_UART , (uint8_t *)&bigRuneMSG, 4, 0xFFFF);
				}
			}*/
			//ZY
		}break;
		case STOP_STATE:   
		{
			if(GetInputMode() != STOP )
			{
				g_workState = PREPARE_STATE;   
			}
		}break;
		case RUNE_STATE:
		{
//			fw_printfln("in rune state");
			if(GetInputMode() == STOP )
			{
				g_workState = STOP_STATE;
			}
			else if(((g_switch1.switch_value1 == REMOTE_SWITCH_CHANGE_1TO3 
									|| g_switch1.switch_value1 == REMOTE_SWITCH_CHANGE_2TO3) 
							||RC_CtrlData.key.v == 512)//F
							&& g_switchRead == 1)
			{
				g_workState = NORMAL_STATE;
				g_switchRead = 0;
				zyRuneMode=4;
			}
			else if(GetInputMode() == KEY_MOUSE_INPUT
								&& (RC_CtrlData.key.v == 16384)&& g_switchRead == 1&&zyRuneMode==0)
			{
				g_switchRead = 0;
				if(checkKeyTime>450)
				{
					checkKeyTime=0;
					pRunePosition[0].pitch_position=pitchAngleTarget;
					pRunePosition[0].yaw_position=yawAngleTarget;
					zyRuneMode++;
				}
			}
			else if(GetInputMode() == KEY_MOUSE_INPUT
								&& (RC_CtrlData.key.v == 16384)&& g_switchRead == 1&&zyRuneMode==1)
			{
				g_switchRead = 0;
				if(checkKeyTime>450)
				{
					checkKeyTime=0;
					pRunePosition[1].pitch_position=pitchAngleTarget;
					pRunePosition[1].yaw_position=yawAngleTarget;
					zyRuneMode++;
				}
			}else if(GetInputMode() == KEY_MOUSE_INPUT
								&& (RC_CtrlData.key.v == 16384)&& g_switchRead == 1&&zyRuneMode==2)
			{
				g_switchRead = 0;
				if(checkKeyTime>450)
				{
					checkKeyTime=0;
					pRunePosition[2].pitch_position=pitchAngleTarget;
					pRunePosition[2].yaw_position=yawAngleTarget;
					zyLocationInit(pRunePosition);
					zyRuneMode=4;
				}
			}
		}break;
		default:
		{
			
		}
	}	
}


extern float gap_angle;
extern float pitchRealAngle;
	
void WorkStateSwitchProcess(void)
{
	if((lastWorkState != g_workState) && (g_workState == STOP_STATE))  
	{
		LASER_OFF();
		SetShootState(NOSHOOTING);
		SetFrictionWheelSpeed(1000);
		SetFrictionState(FRICTION_WHEEL_OFF);
		frictionRamp.ResetCounter(&frictionRamp);
	}
	//如果从其他模式切换到prapare模式，要将一系列参数初始化
	if((lastWorkState != g_workState) && (g_workState == PREPARE_STATE))  
	{
		//计数初始化
	  s_time_tick_2ms = 0;   
		yawAngleTarget = 0;
		pitchAngleTarget = 0;
		CMControlInit();
		RemoteTaskInit();
	}
	if((lastWorkState != g_workState) && (g_workState == RUNE_STATE))  
	{
		/*zyLocationInit(gap_angle, pitchAngleTarget);
		//yawAngleTarget = gap_angle;
		
//		fw_printfln("Rune gap_angle:%f",gap_angle);
//		fw_printfln("Rune pitchRealAngle:%f",pitchRealAngle);
//		
		//LASER_OFF();//zy0726
		*/
		yawAngleTarget = 0;
		pitchAngleTarget = pitchRealAngle;
		*(IOPool_pGetWriteData(ctrlUartIOPool) -> ch) = 4;
		IOPool_getNextWrite(ctrlUartIOPool);
	}
	if((lastWorkState != g_workState) && (lastWorkState == RUNE_STATE))  
	{
		//LASER_OFF();
		LASER_ON();
		SetShootState(NOSHOOTING);
		SetFrictionWheelSpeed(1000);
		SetFrictionState(FRICTION_WHEEL_OFF);
		frictionRamp.ResetCounter(&frictionRamp);
		
		if(HAL_UART_Transmit(&MANIFOLD_UART , (uint8_t *)&waitRuneMSG, 4, 0xFFFF) != HAL_OK)
		{
			fw_Warning();
		};
	}
	if((g_workState == NORMAL_STATE) && (lastWorkState == RUNE_STATE))  
	{
		yawAngleTarget = -ZGyroModuleAngle;
	}


}
  
void RuneShootControl(void) 
{ 
	if(g_workState == RUNE_STATE)
	{
		switch(GetFrictionState())
		{
			case FRICTION_WHEEL_OFF:
			{
				SetShootState(NOSHOOTING);
				frictionRamp.ResetCounter(&frictionRamp);
				SetFrictionState(FRICTION_WHEEL_START_TURNNING);	 
				//LASER_OFF(); //zy0726
			}break;
			case FRICTION_WHEEL_START_TURNNING:
			{
				/*斜坡函数必须有，避免电流过大烧坏主控板*/
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
				//SetFrictionWheelSpeed(1000);
				//g_friction_wheel_state = FRICTION_WHEEL_ON; 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					SetFrictionState(FRICTION_WHEEL_ON); 	
				}
			}break;
			case FRICTION_WHEEL_ON:
			{
//				SetShootState(SHOOTING);
			} break;				
		}
	}
}

extern uint8_t autoReceived;
extern uint8_t autoGet;
void getJudgeState(void)
{
	static int s_count_judge = 0;
	static int k_count_auto = 0;
	if(JUDGE_Received==1)
	{
		s_count_judge = 0;
		JUDGE_State = ONLINE;
		JUDGE_Received = 0;
	}
	else
	{
		s_count_judge++;
		if(s_count_judge > 150)
		{//300ms
			JUDGE_State = OFFLINE;
		}
	}
	if(autoReceived==1)
	{
		k_count_auto = 0;
		autoGet = 1;
		JUDGE_Received = 0;
	}
	else
	{
		k_count_auto++;
		if(k_count_auto > 150)
		{//300ms
			autoGet = 0;
		}
	}
}

