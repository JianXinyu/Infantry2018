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
#ifdef INFANTRY_1
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(7.0, 2, 4, 10000.0, 10000.0, 10000.0, 6000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(15.0, 0.0, 0, 10000.0, 10000.0, 10000.0, 6000.0);
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(5, 0.0, 0, 10000.0, 10000.0, 10000.0, 5000.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(15.0, 0.5, 0, 10000.0, 10000.0, 10000.0, 5000.0);
#define yaw_zero 2136
#define pitch_zero 7100
#endif

#ifdef INFANTRY_4
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(4, 5.0, 5, 10000.0, 10000.0, 10000.0, 6000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(15, 0.0, 0, 10000.0, 10000.0, 10000.0, 6000.0);//30, 0, 4.5
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(1.5, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 5000.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(4.0, 0, 0, 10000.0, 10000.0, 10000.0, 5000.0);//4, 0 ,0
#define yaw_zero 5590
#define pitch_zero 6600
#endif

#ifdef INFANTRY_5
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(8.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 6000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(15.0, 0.0, 0, 10000.0, 10000.0, 10000.0, 6000.0);
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(12, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 5000.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(15.0, 0.5, 0, 10000.0, 10000.0, 10000.0, 5000.0);
#define yaw_zero 7820
#define pitch_zero 7740
#endif

#ifdef LITTLE_SON
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(5.0, 2, 5, 10000.0, 10000.0, 10000.0, 6000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(15.0, 0.0, 0, 10000.0, 10000.0, 10000.0, 6000.0);
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(5, 0.0, 0, 10000.0, 10000.0, 10000.0, 5000.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(15.0, 0.5, 0, 10000.0, 10000.0, 10000.0, 5000.0);
#define yaw_zero 7400
#define pitch_zero 4555
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
float PitchMotorAngle = 0.0;
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
uint16_t zyPlateFrequency=0,zyCurrentHeat=0;//拨盘射频(单位:Hz)


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
//extern uint16_t autoBuffer[10];
extern float gyroYAngle, gyroZAngle;
extern float gyroXspeed, gyroYspeed, gyroZspeed;
extern float zeroGyro;
float deltaGyro;
float readsth;
float friction_speed = 0;

//该系数是为了解决电机电流必须取反的问题, 尚未找出原因
int strange_coefficient_yaw = 1;
int strange_coefficient_pitch_intensity = 1;
int strange_coefficient_lf_intensity = 1;
int strange_coefficient_rf_intensity = 1;
//该系数是为了解决电机反转的问题
int strange_coefficient_pitch_dir = 1;
int strange_coefficient_lf_dir = 1;
int strange_coefficient_rf_dir = 1;

void CMGMControlTask(void const * argument)
{
	while(1)
	{
		//等待CAN接收回调函数信号量
		osSemaphoreWait(CMGMCanRefreshSemaphoreHandle, osWaitForever);
		
		ControlYaw();
		ControlPitch();

	 
//		ChassisSpeedRef.rotate_ref = 0;//取消底盘跟随
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
int isGMYawFirstEnter = 1;
int isGMYawGyroFirstEnter = 1;
float yawMotorAngle = 0.0;
float yawRealAngle = 0.0;
float GMYAWThisAngle, GMYAWLastAngle;
float GMYAWGyroThisAngle, GMYAWGyroLastAngle;

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
/*			yawMotorAngle = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle - yawZeroAngle) * 360 / 8192.0f;
			NORMALIZE_ANGLE180(yawMotorAngle);
			yawRealAngle = yawMotorAngle;*/
	
			GMYAWThisAngle = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle);
			if(isGMYawFirstEnter == 1)
				{
					GMYAWLastAngle = GMYAWThisAngle;
					yawMotorAngle = ((IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle) - yawZeroAngle) * 360 / 8192.0f;
					isGMYawFirstEnter = 0;
				}
			if(GMYAWThisAngle <= GMYAWLastAngle)
				{
						if((GMYAWLastAngle-GMYAWThisAngle) > 4000)//编码器上溢
							 yawMotorAngle = yawMotorAngle + (GMYAWThisAngle+8192-GMYAWLastAngle) * 360 / 8192.0f;
						else//反转
							 yawMotorAngle = yawMotorAngle + (GMYAWThisAngle - GMYAWLastAngle) * 360 / 8192.0f;
				}
			else
				{
						if((GMYAWThisAngle-GMYAWLastAngle) > 4000)//编码器下溢
							 yawMotorAngle = yawMotorAngle - (GMYAWLastAngle+8192-GMYAWThisAngle) * 360 / 8192.0f;
						else//正转
							 yawMotorAngle = yawMotorAngle + (GMYAWThisAngle - GMYAWLastAngle) * 360 / 8192.0f;
				}
			GMYAWLastAngle = GMYAWThisAngle;
			NORMALIZE_ANGLE180(yawMotorAngle);
			if(GetWorkState() == NORMAL_STATE || GetWorkState() == PREPARE_STATE) 
			{
				//yawRealAngle = -ZGyroModuleAngle;//yawrealangle的值改为复位后陀螺仪的绝对值，进行yaw轴运动设定
				//deltaGyro = gyroZAngle - zeroGyro;
			if(isGMYawFirstEnter == 1)
			{
				yawRealAngle = yawMotorAngle;
				isGMYawFirstEnter = 0;
			}
//			else yawRealAngle = NORMALIZE_ANGLE180(deltaGyro);
//				if(isGMYawFirstEnter == 0) yawRealAngle = NORMALIZE_ANGLE180(deltaGyro);
				GMYAWGyroThisAngle = gyroZAngle;
				if(GMYAWGyroThisAngle <= GMYAWGyroLastAngle)
					{
							if((GMYAWGyroLastAngle-GMYAWGyroThisAngle) > 180)
								 yawRealAngle = yawRealAngle + (GMYAWGyroThisAngle+360-GMYAWGyroLastAngle);
							else
								 yawRealAngle = yawRealAngle + (GMYAWGyroThisAngle - GMYAWGyroLastAngle);
					}
				else
					{
							if((GMYAWGyroThisAngle-GMYAWGyroLastAngle) > 180)
								 yawRealAngle = yawRealAngle - (GMYAWGyroLastAngle+360-GMYAWGyroThisAngle);
							else
								 yawRealAngle = yawRealAngle + (GMYAWGyroThisAngle - GMYAWGyroLastAngle);
					}
				GMYAWGyroLastAngle = GMYAWGyroThisAngle ;
			}
			
 			if(isGMYawGyroFirstEnter == 1)
					{
						GMYAWGyroLastAngle = GMYAWGyroThisAngle;
						yawRealAngle = gyroZAngle - zeroGyro;
						isGMYawGyroFirstEnter = 0;
					}
				else if(GetWorkState() == PREPARE_STATE)
				{
					yawRealAngle = yawMotorAngle;
				}

			/*else if(GetWorkState()==RUNE_STATE)
			{
				//fw_printfln("Rune State:%f",yawAngleTarget);
				//yawAngleTarget=zyYawTartet;
				//yawRealAngle = -ZGyroModuleAngle;
			}*/

				
				//yawIntensity = ProcessYawPID(yawAngleTarget, yawRealAngle, -gYroZs);
			#ifdef INFANTRY_4
				strange_coefficient_yaw = 1;
			#endif
			#ifdef INFANTRY_5
				strange_coefficient_yaw = -1;
			#endif
			#ifdef LITTLE_SON 
				strange_coefficient_yaw = -1;
			#endif				
			yawIntensity = strange_coefficient_yaw * ProcessYawPID(yawAngleTarget, yawRealAngle, -gyroZspeed);
			
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
int isPitchMotorFirstEnter = 1, isPitchGyroFirstEnter = 1;
float PitchMotorThisAngle, PitchMotorLastAngle;
float PitchGyroThisAngle, PitchGyroLastAngle;
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
			
			PitchMotorThisAngle = (IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle);
			if(isPitchMotorFirstEnter == 1)
				{
					PitchMotorLastAngle = PitchMotorThisAngle;
					PitchMotorAngle = ((IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle) - pitchZeroAngle) * 360 / 8192.0f;
					isPitchMotorFirstEnter = 0;
				}
			if(PitchMotorThisAngle <= PitchMotorLastAngle)
				{
						if((PitchMotorLastAngle-PitchMotorThisAngle) > 4000)//编码器上溢
							 PitchMotorAngle = PitchMotorAngle + (PitchMotorThisAngle+8192-PitchMotorLastAngle) * 360 / 8192.0f;
						else//反转
							 PitchMotorAngle = PitchMotorAngle + (PitchMotorThisAngle - PitchMotorLastAngle) * 360 / 8192.0f;
				}
			else
				{
						if((PitchMotorThisAngle-PitchMotorLastAngle) > 4000)//编码器下溢
							 PitchMotorAngle = PitchMotorAngle - (PitchMotorLastAngle+8192-PitchMotorThisAngle) * 360 / 8192.0f;
						else//正转
							 PitchMotorAngle = PitchMotorAngle + (PitchMotorThisAngle - PitchMotorLastAngle) * 360 / 8192.0f;
				}
			PitchMotorLastAngle = PitchMotorThisAngle;
			NORMALIZE_ANGLE180(PitchMotorAngle);
			
				PitchGyroThisAngle = gyroYAngle;
				if(isPitchGyroFirstEnter == 1)
					{
						PitchGyroLastAngle = PitchGyroThisAngle;
						pitchRealAngle = NORMALIZE_ANGLE180(gyroYAngle);
						isPitchGyroFirstEnter = 0;
					}
				if(PitchGyroThisAngle <= PitchGyroLastAngle)
					{
							if((PitchGyroLastAngle-PitchGyroThisAngle) > 180)
								 pitchRealAngle = pitchRealAngle + (PitchGyroThisAngle+360-PitchGyroLastAngle);
							else
								 pitchRealAngle = pitchRealAngle + (PitchGyroThisAngle - PitchGyroLastAngle);
					}
				else
					{
							if((PitchGyroThisAngle-PitchGyroLastAngle) > 180)
								 pitchRealAngle = pitchRealAngle - (PitchGyroLastAngle+360-PitchGyroThisAngle);
							else
								 pitchRealAngle = pitchRealAngle + (PitchGyroThisAngle - PitchGyroLastAngle);
					}
				PitchGyroLastAngle = PitchGyroThisAngle ;
		
			#ifdef INFANTRY_1
			MINMAX(pitchAngleTarget, -30.0f, 30);
			strange_coefficient_pitch_intensity = 1;
			strange_coefficient_pitch_dir = -1;
			#endif			
			#ifdef INFANTRY_4
			MINMAX(pitchAngleTarget, -30.0f, 60);
			strange_coefficient_pitch_intensity = -1;
			strange_coefficient_pitch_dir = 1;	
			#endif
			#ifdef INFANTRY_5
			MINMAX(pitchAngleTarget, -40.0f, 60);//1:2的减速比,实际角度除以2
			strange_coefficient_pitch_intensity = 1;
			strange_coefficient_pitch_dir = -1;
			#endif
			#ifdef LITTLE_SON
			MINMAX(pitchAngleTarget, -30.0f, 60);
			strange_coefficient_pitch_intensity = -1;
			strange_coefficient_pitch_dir = -1;	
			#endif			
//			pitchIntensity = strange_coefficient_pitch_intensity * ProcessPitchPID(strange_coefficient_pitch_dir * pitchAngleTarget,PitchMotorAngle,-gyroYspeed);
			pitchIntensity = strange_coefficient_pitch_intensity * ProcessPitchPID(strange_coefficient_pitch_dir * pitchAngleTarget,2 * pitchRealAngle,-gyroYspeed);
//			pitchIntensity = 0;
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
extern uint16_t maxHeat;
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
			
			zyPlateFrequency=6*plateRealSpeed/(60*36);
			
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
			
			if((JUDGE_State == ONLINE && remainHeat < realBulletSpeed) || (JUDGE_State == ONLINE && burst && remainHeat < realBulletSpeed*3
				))plateAngleTarget = plateRealAngle;
			/*if(JUDGE_State==ONLINE&&remainHeat!=666)
			{
				zyCurrentHeat=remainHeat;
				remainHeat=666;
				
			}
			if(burst &&(zyCurrentHeat+1.8)<((((plateRealAngle-plateAngleTarget)/60)*realBulletSpeed)*0.05))
			{
				plateAngleTarget = plateRealAngle;
			}
			zyCurrentHeat-= realBulletSpeed*(plateRealAngle-plateAngleTarget)/60;
			if(zyCurrentHeat<0){
				zyCurrentHeat=0;
			}*/
				
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
#ifdef INFANTRY_1
#define Rotate_Coefficient_FL 1;
#endif
#ifdef INFANTRY_4 
#define Rotate_Coefficient_FL 1;
#endif
#ifdef INFANTRY_5 
#define Rotate_Coefficient_FL 163.18/193.18;
//#define Rotate_Coefficient_FL 1;
#endif
#ifdef LITTLE_SON
#define Rotate_Coefficient_FL 1;
#endif
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
											 + ChassisSpeedRef.rotate_ref * boost * Rotate_Coefficient_FL;
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
			strange_coefficient_lf_dir = 1;
			strange_coefficient_lf_intensity = -1;
			#endif
			#ifdef INFANTRY_5
			strange_coefficient_lf_dir = 1;
			strange_coefficient_lf_intensity = 1;
			#endif
			#ifdef LITTLE_SON
			strange_coefficient_lf_dir = 1;
			strange_coefficient_lf_intensity = -1;
			#endif
			
			LFRICTIONSpeedPID.ref = strange_coefficient_lf_dir * friction_speed;
			
			LFRICTIONSpeedPID.fdb = pData->RotateSpeed;

      LFRICTIONSpeedPID.Calc(&LFRICTIONSpeedPID);
			
			setMotor(LFRICTION, strange_coefficient_lf_intensity * CHASSIS_SPEED_ATTENUATION * LFRICTIONSpeedPID.output);

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
			
			#ifdef INFANTRY_1
			strange_coefficient_rf_dir = -1;
			strange_coefficient_rf_intensity = -1;
			#endif
			#ifdef INFANTRY_4
			strange_coefficient_rf_dir = -1;
			strange_coefficient_rf_intensity = 1;
			#endif
			#ifdef INFANTRY_5
			strange_coefficient_rf_dir = -1;
			strange_coefficient_rf_intensity = 1;
			#endif
			#ifdef LITTLE_SON
			strange_coefficient_rf_dir = -1;
			strange_coefficient_rf_intensity = 1;
			#endif
			RFRICTIONSpeedPID.ref = strange_coefficient_rf_dir * friction_speed;
			RFRICTIONSpeedPID.fdb = pData->RotateSpeed;
			
			RFRICTIONSpeedPID.Calc(&RFRICTIONSpeedPID);

			setMotor(RFRICTION, strange_coefficient_rf_intensity * CHASSIS_SPEED_ATTENUATION * RFRICTIONSpeedPID.output);

			s_RFRICTIONCount = 0;
		}
		else
		{
			s_RFRICTIONCount++;
		}
	}
}
