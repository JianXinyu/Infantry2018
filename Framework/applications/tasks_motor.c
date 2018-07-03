/**
  ******************************************************************************
  * File Name          : tasks_motor.c
  * Description        : 电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 云台、底盘电机控制任务
	* 处于阻塞态等待CAN接收任务释放信号量
	* 对CAN收到的数据进行PID计算，再将电流值发送到CAN
  ******************************************************************************
  */
#include "tasks_motor.h"
#include "drivers_canmotor_user.h"
#include "rtos_semaphore.h"
#include "drivers_uartrc_user.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <tim.h>
#include <usart.h>
#include "utilities_debug.h"
#include "tasks_upper.h"
#include "tasks_timed.h"
#include "tasks_remotecontrol.h"
#include "drivers_led_user.h"
#include "utilities_minmax.h"
#include "pid_regulator.h"
#include "application_motorcontrol.h"
#include "drivers_sonar_user.h"
#include "peripheral_define.h"
#include "drivers_uartupper_user.h"
#include "tasks_platemotor.h"
#include "drivers_uartjudge_low.h"

//PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax)
//云台PID
#ifdef INFANTRY_5
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(8.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 6000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(5.0, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 6000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(40.0, 0.0, 15.0, 10000.0, 10000.0, 10000.0, 3500.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(30.0, 0.0, 5, 10000.0, 10000.0, 10000.0, 4000.0);
//手动标定0点
#define yaw_zero 2560
#define pitch_zero 1810
#endif
#ifdef INFANTRY_4
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(4, 5.0, 5, 10000.0, 10000.0, 10000.0, 6000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(15, 0.0, 4.5, 10000.0, 10000.0, 10000.0, 6000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(1.5, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 3500.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(2.0, 0.0, 0, 10000.0, 10000.0, 10000.0, 4000.0);
#define yaw_zero 5570
#define pitch_zero 4629 
#endif
#ifdef INFANTRY_1
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(200.0, 1.0, 40, 10000.0, 10000.0, 10000.0, 6000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(25.0, 0.0, 5, 10000.0, 10000.0, 10000.0, 6000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(1, 0.0, 20, 10000.0, 10000.0, 10000.0, 3500.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(2, 0.0, 2, 10000.0, 10000.0, 10000.0, 4000.0);
#define yaw_zero 2136
#define pitch_zero 7087
#endif
fw_PID_Regulator_t PLATEPositionPID = fw_PID_INIT(200.0, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 8000.0);
fw_PID_Regulator_t PLATESpeedPID = fw_PID_INIT(200.0, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 8000.0);


//底盘速度PID
PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t plateSpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t LFRICTIONSpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t RFRICTIONSpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

extern uint8_t g_isGYRO_Rested;
//陀螺仪角速度
extern float gYroXs, gYroYs, gYroZs;
//外接单轴陀螺仪角度
extern float ZGyroModuleAngle;
extern float shootdir;
float yawAngleTarget = 0.0;
float gap_angle = 0.0;
float pitchRealAngle = 0.0;
float pitchAngleTarget = 0.0;
float boost = 0.80f;
float plateAngleTarget = 0.0;
float plateRealAngle = 0.0;
double plateRealSpeed = 0;
uint16_t plateThisAngle = 0;
uint16_t plateLastAngle = 0;
extern Shoot_State_e shootState;
//大神符、自瞄
extern Location_Number_s Location_Number[];
extern uint8_t CReceive;
extern uint8_t rune_flag;
//扭腰
int twist_state = 0;
int twist_count = 0;
int twist =0;
float mm =0;
float nn =0;
int16_t twist_target = 0;
extern uint8_t cancel_chassis_rotate;

extern float zyYawTarget,zyPitchTarget;
float yawRealAngle = 0.0;//张雁调试大符

static uint8_t s_yawCount = 0;
static uint8_t s_pitchCount = 0;
static uint8_t s_CMFLCount = 0;
static uint8_t s_CMFRCount = 0;
static uint8_t s_CMBLCount = 0;
static uint8_t s_CMBRCount = 0;
static uint8_t s_plateCount = 0;
static uint8_t s_LFRICTIONCount = 0;
static uint8_t s_RFRICTIONCount = 0;
uint16_t pcnt;
extern RC_Ctl_t RC_CtrlData; 
extern uint8_t burstflag;
extern float nowPlateAngle;
extern JudgeState_e JUDGE_State;
extern float realBulletSpeed;
extern uint16_t remainHeat;
extern float realBulletSpeed;
extern uint8_t burst;
extern uint16_t autoBuffer[10];
extern float gyroZAngle;
extern float gyroXspeed,gyroYspeed,gyroZspeed;
extern float zeroGyro;
float deltaGyro;
float readsth;
float friction_speed = 0;

int strange_coefficient_pitch = 1;
int strange_coefficient_lf = 1;
int strange_coefficient_rf = 1;

void CMGMControlTask(void const * argument)
{
	while(1)
	{
		//等待CAN接收回调函数信号量
		osSemaphoreWait(CMGMCanRefreshSemaphoreHandle, osWaitForever);
		
		ControlYaw();
		ControlPitch();

	 
		ChassisSpeedRef.rotate_ref = 0;//取消底盘跟随
		ControlCMFL();
		ControlCMFR();
		ControlCMBL();
		ControlCMBR();
		ControlPLATE();
		ControlLFRICTION();
		ControlRFRICTION();
		
		if(plateRealAngle - plateAngleTarget > 1 || (plateRealAngle - plateAngleTarget > 50 && burst))pcnt ++;
		else pcnt = 0;
		if(pcnt > 10000){plateAngleTarget = plateRealAngle + 72;pcnt = 0;}
//		pcnt++;
//		if(pcnt > 5000)
//		{ShootOneBullet();pcnt = 0;}

	//	ShootState = SHOOTING;
		
	}//end of while
}


float zero;
uint16_t zyg;
uint16_t findyawzero, findpitchzero;

/*Yaw电机*/
int16_t yawIntensity = 0;
int isGMYAWFirstEnter = 1;
void ControlYaw(void)
{
	if(IOPool_hasNextRead(GMYAWRxIOPool, 0))
	{
		if(s_yawCount == 1)
		{
			uint16_t yawZeroAngle = yaw_zero;
			//float yawRealAngle = 0.0;张雁改全局
			
			
			/*从IOPool读编码器*/
			IOPool_getNextRead(GMYAWRxIOPool, 0); 
	//		fw_printfln("yaw%d",IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle);
			findyawzero = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle);
			yawRealAngle = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle - yawZeroAngle) * 360 / 8192.0f;
			NORMALIZE_ANGLE180(yawRealAngle);
			
			
			if(GetWorkState() == NORMAL_STATE) 
			{
				//yawRealAngle = -ZGyroModuleAngle;//yawrealangle的值改为复位后陀螺仪的绝对值，进行yaw轴运动设定
				deltaGyro = gyroZAngle - zeroGyro;
				yawRealAngle = NORMALIZE_ANGLE180(deltaGyro);
			}
			/*else if(GetWorkState()==RUNE_STATE)
			{
				//fw_printfln("Rune State:%f",yawAngleTarget);
				//yawAngleTarget=zyYawTartet;
				//yawRealAngle = -ZGyroModuleAngle;
			}*/
			yawIntensity = ProcessYawPID(yawAngleTarget, yawRealAngle, -gyroZspeed);
			//yawIntensity = ProcessYawPID(yawAngleTarget, yawRealAngle, -gYroZs);
			//yawIntensity = 0;
			setMotor(GMYAW, yawIntensity);
			s_yawCount = 0;
			
			ControlRotate();
			
		}
		else
		{
			s_yawCount++;
		}
		 
	}
}
int16_t pitchIntensity = 0;
/*Pitch电机*/
void ControlPitch(void)
{
	if(IOPool_hasNextRead(GMPITCHRxIOPool, 0))
	{
		if(s_pitchCount == 1)
		{
			uint16_t pitchZeroAngle = pitch_zero;
			
			
			IOPool_getNextRead(GMPITCHRxIOPool, 0);
			findpitchzero = (IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle);
			pitchRealAngle = -(IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle - pitchZeroAngle) * 360 / 8192.0;
			NORMALIZE_ANGLE180(pitchRealAngle);
		
			#ifdef INFANTRY_1
			MINMAX(pitchAngleTarget, -40.0f, 35);
			strange_coefficient_pitch = -1;
			#endif			
			#ifdef INFANTRY_4
			MINMAX(pitchAngleTarget, -15.f, 45);
			strange_coefficient_pitch = 1;
			#endif
			
			pitchIntensity = strange_coefficient_pitch * ProcessPitchPID(pitchAngleTarget,pitchRealAngle,-gyroYspeed);
			setMotor(GMPITCH, pitchIntensity);
			
			s_pitchCount = 0;
		}
		else
		{
			s_pitchCount++;
		}
	}
}
int16_t plateIntensity = 0;
uint16_t plate_zero = 2000;
void ControlPLATE(void)
{
	if(IOPool_hasNextRead(PLATERxIOPool, 0))
	{
		if(s_plateCount == 1)
		{
			//int16_t plateIntensity = 0;
			
			IOPool_getNextRead(PLATERxIOPool, 0);
			//dirread = (IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle);
			plateThisAngle = IOPool_pGetReadData(PLATERxIOPool, 0)->angle;
			plateRealSpeed = IOPool_pGetReadData(PLATERxIOPool, 0)->RotateSpeed;
			
			if(plateThisAngle<=plateLastAngle)//2006减速比36:1
			{
				if((plateLastAngle-plateThisAngle)>3000)//编码器上溢
					plateRealAngle = plateRealAngle + (plateThisAngle+8192-plateLastAngle) * 360 / 8192.0 / 36;
				else//反转
					plateRealAngle = plateRealAngle - (plateLastAngle - plateThisAngle) * 360 / 8192.0 / 36;
			}
			else
			{
				if((plateThisAngle-plateLastAngle)>3000)//编码器下溢
					plateRealAngle = plateRealAngle - (plateLastAngle+8192-plateThisAngle) * 360 / 8192.0 / 36;
				else//正转
					plateRealAngle = plateRealAngle + (plateThisAngle - plateLastAngle) * 360 / 8192.0 / 36;
			}
//			NORMALIZE_ANGLE180(plateRealAngle);

			plateLastAngle = plateThisAngle;
			if((JUDGE_State == ONLINE && remainHeat < realBulletSpeed) || (JUDGE_State == ONLINE && burst && remainHeat < realBulletSpeed*2+7
				))plateAngleTarget = plateRealAngle;
			plateIntensity = PID_PROCESS_Double(PLATEPositionPID,PLATESpeedPID,plateAngleTarget,plateRealAngle,plateRealSpeed);
			if(fabs(plateAngleTarget - plateRealAngle) < 5)plateIntensity = 0.3 * plateIntensity;
			if(plateAngleTarget - plateRealAngle > 10 && plateAngleTarget - plateRealAngle < 100 && GetWorkState() == RUNE_STATE)plateIntensity = 20 * PID_PROCESS_Double(PLATEPositionPID,PLATESpeedPID,plateAngleTarget,plateRealAngle,plateRealSpeed);
			//		plateIntensity = PID_PROCESS_Double(PlatePositionPID,PlateSpeedPID,plateAngleTarget,plateRealAngle,plateRealSpeed);
			//防止TDAngleTarget&TDRealAngle溢出
			if(plateAngleTarget <= -5000 && plateRealAngle <= -5000)
			{
				plateAngleTarget = plateAngleTarget + 4000;
				plateRealAngle = plateRealAngle + 4000;
			}

			setMotor(PLATE, plateIntensity);
			
			s_plateCount = 0;
		}
		else
		{
			s_plateCount++;
		}
	}
}
float raw_gap = 0.0;
/*底盘转动控制：跟随云台/扭腰等*/
void ControlRotate(void)
{
	raw_gap  = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle - yaw_zero) * 360 / 8192.0f;
	if(shootdir <= 45 && shootdir >= -45)gap_angle = raw_gap + shootdir;
	else gap_angle = raw_gap;
    NORMALIZE_ANGLE180(gap_angle);	
	
	
	
	if(GetWorkState() == NORMAL_STATE) 
	{
		/*扭腰*/
		//试图用PID
		if (twist_state == 1)
		{
			CMRotatePID.output = 0; //一定角度之间进行扭腰
			twist = (twist_count / 600)%2 ;	
			if (twist == nn){
				CMRotatePID.output = -10;
				twist_count = twist_count + 1;
			}
			if (twist == (1-nn)){
				CMRotatePID.output = 10;
				twist_count = twist_count + 1;
			}
			 ChassisSpeedRef.rotate_ref = CMRotatePID.output;
		}				
		else
		{
			/*产生扭腰随机数*/  
			 srand(xTaskGetTickCount());
			 mm = (1.0f*rand()/RAND_MAX);//产生随机方向
			 nn = floor(2.0f*mm);
					
			/*底盘跟随编码器旋转PID计算*/		
			 CMRotatePID.ref = 0;
			 CMRotatePID.fdb = gap_angle;
			if(cancel_chassis_rotate) // key: r
			{				
			if(gap_angle<10&&gap_angle>-10) CMRotatePID.fdb = 0;
			}
			 CMRotatePID.Calc(&CMRotatePID);   
			 ChassisSpeedRef.rotate_ref = CMRotatePID.output;
		}
	}
}

/*底盘电机控制FL(ForwardLeft)FR BL BR*/
void ControlCMFL(void)
{		
	if(IOPool_hasNextRead(CMFLRxIOPool, 0))
	{
		if(s_CMFLCount == 1)
		{
			IOPool_getNextRead(CMFLRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMFLRxIOPool, 0);
			
			CM2SpeedPID.ref = - ChassisSpeedRef.forward_back_ref*0.075 
											 + ChassisSpeedRef.left_right_ref*0.075 
											 + ChassisSpeedRef.rotate_ref * boost;
			CM2SpeedPID.ref = 160 * CM2SpeedPID.ref;
			readsth = CM2SpeedPID.ref;
			
			if(GetWorkState() == RUNE_STATE) 
			{
				CM2SpeedPID.ref = 0;
			}
			
			CM2SpeedPID.fdb = pData->RotateSpeed;
			#ifdef INFANTRY_4
			CM2SpeedPID.ref =  CM2SpeedPID.ref / 1.0f;
			#endif
			CM2SpeedPID.Calc(&CM2SpeedPID);
			
			setMotor(CMFR, CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output);

			s_CMFLCount = 0;
		}
		else
		{
			s_CMFLCount++;
		}
	}
}

void ControlCMFR(void)
{
	if(IOPool_hasNextRead(CMFRRxIOPool, 0))
	{
		if(s_CMFRCount == 1)
		{
			IOPool_getNextRead(CMFRRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMFRRxIOPool, 0);
			
			CM1SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075 
											 + ChassisSpeedRef.left_right_ref*0.075 
											 + ChassisSpeedRef.rotate_ref * boost;	
			CM1SpeedPID.ref = 160 * CM1SpeedPID.ref;
			CM1SpeedPID.fdb = pData->RotateSpeed;
			#ifdef INFANTRY_4
			CM1SpeedPID.ref = CM1SpeedPID.ref / 1.0f;
			#endif
			
			if(GetWorkState() == RUNE_STATE) 
			{
				CM1SpeedPID.ref = 0;
			}
			
			CM1SpeedPID.Calc(&CM1SpeedPID);
			
			setMotor(CMFL, CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output);

			s_CMFRCount = 0;
		}
		else
		{
			s_CMFRCount++;
		}
	}
}
	
void ControlCMBL(void)
{
	if(IOPool_hasNextRead(CMBLRxIOPool, 0))
	{
		if(s_CMBLCount == 1)
		{
			IOPool_getNextRead(CMBLRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMBLRxIOPool, 0);
			
			CM3SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075 
											 - ChassisSpeedRef.left_right_ref*0.075 
											 + ChassisSpeedRef.rotate_ref * boost;
			CM3SpeedPID.ref = 160 * CM3SpeedPID.ref;
			CM3SpeedPID.fdb = pData->RotateSpeed;
			#ifdef INFANTRY_4
			CM3SpeedPID.ref = CM3SpeedPID.ref / 1.0f;
			#endif
			
			if(GetWorkState() == RUNE_STATE) 
			{
				CM3SpeedPID.ref = 0;
			}
			
			CM3SpeedPID.Calc(&CM3SpeedPID);
			
			setMotor(CMBL, CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output);

			s_CMBLCount = 0;
		}
		else
		{
			s_CMBLCount++;
		}
	}
}

void ControlCMBR()
{
	if(IOPool_hasNextRead(CMBRRxIOPool, 0))
	{
		if(s_CMBRCount ==1)
		{
			IOPool_getNextRead(CMBRRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMBRRxIOPool, 0);
			
			CM4SpeedPID.ref = - ChassisSpeedRef.forward_back_ref*0.075 
											 - ChassisSpeedRef.left_right_ref*0.075 
											 + ChassisSpeedRef.rotate_ref * boost;
			CM4SpeedPID.ref = 160 * CM4SpeedPID.ref;
			CM4SpeedPID.fdb = pData->RotateSpeed;
			#ifdef INFANTRY_4
			CM4SpeedPID.ref = CM4SpeedPID.ref / 1.0f;
			#endif
			
			if(GetWorkState() == RUNE_STATE) 
			{
				CM4SpeedPID.ref = 0;
			}
			
			CM4SpeedPID.Calc(&CM4SpeedPID);
			
			setMotor(CMBR, CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output);

			s_CMBRCount = 0;
		}
		else
		{
			s_CMBRCount++;
		}
	}
}

float readRef;
//Left Friction Wheel
void ControlLFRICTION(void)
{		
	if(IOPool_hasNextRead(LFRICTIONRxIOPool, 0))
	{
		if(s_LFRICTIONCount == 1)
		{
			IOPool_getNextRead(LFRICTIONRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(LFRICTIONRxIOPool, 0);
			#ifdef INFANTRY_4
			strange_coefficient_lf = 1;
			#endif
			//4号取正
			LFRICTIONSpeedPID.ref = strange_coefficient_lf * friction_speed;
			
			LFRICTIONSpeedPID.fdb = pData->RotateSpeed;

      LFRICTIONSpeedPID.Calc(&LFRICTIONSpeedPID);
			//4号电流取-
			setMotor(LFRICTION, -strange_coefficient_lf * CHASSIS_SPEED_ATTENUATION * LFRICTIONSpeedPID.output);

			s_LFRICTIONCount = 0;
		}
		else
		{
			s_LFRICTIONCount++;
		}
	}
}

void ControlRFRICTION(void)
{
	if(IOPool_hasNextRead(RFRICTIONRxIOPool, 0))
	{
		if(s_RFRICTIONCount == 1)
		{
			IOPool_getNextRead(RFRICTIONRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(RFRICTIONRxIOPool, 0);
			
			#ifdef INFANTRY_4
			strange_coefficient_rf = 1;
			#endif
			//4号取-
			RFRICTIONSpeedPID.ref = -strange_coefficient_rf * friction_speed;
			RFRICTIONSpeedPID.fdb = pData->RotateSpeed;
			
			RFRICTIONSpeedPID.Calc(&RFRICTIONSpeedPID);
			//4号电流取+
			setMotor(RFRICTION, strange_coefficient_rf * CHASSIS_SPEED_ATTENUATION * RFRICTIONSpeedPID.output);

			s_RFRICTIONCount = 0;
		}
		else
		{
			s_RFRICTIONCount++;
		}
	}
}
