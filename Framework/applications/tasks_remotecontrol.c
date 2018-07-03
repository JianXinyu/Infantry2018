/**
  ******************************************************************************
  * File Name          : tasks_remotecontrol.c
  * Description        : 遥控器处理任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "tasks_remotecontrol.h"
#include "drivers_uartrc_user.h"
#include "drivers_uartrc_low.h"
#include "utilities_debug.h"
#include "stdint.h"
#include "stddef.h"
#include "drivers_ramp.h"
#include "pid_regulator.h"
#include "tasks_timed.h"
#include "usart.h"
#include "peripheral_define.h"
#include "pwm_server_motor.h"
#include "drivers_uartjudge_low.h"
#include "tasks_motor.h"
#include "iwdg.h"
//**//
#include "utilities_minmax.h"
#include "math.h"
#include <stdlib.h>
#include <stdbool.h>
#include "tasks_platemotor.h"
#include "drivers_uartupper_user.h"
#include "drivers_servo.h"

#include "peripheral_laser.h"
extern uint8_t zyRuneMode;//ZY激光瞄准镜
uint8_t going = 0;
uint8_t burst = 0;
float shootdir = 0.0;
uint16_t dircnt = 0;
uint8_t cancel_chassis_rotate = 0;

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\


extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern Gimbal_Ref_t GimbalRef;
extern FrictionWheelState_e g_friction_wheel_state ;

RemoteSwitch_t g_switch1;   //ң������ದ��

extern RampGen_t frictionRamp ;  //摩擦轮斜坡
extern RampGen_t LRSpeedRamp ;   //键盘速度斜坡
extern RampGen_t FBSpeedRamp  ;   

extern RC_Ctl_t RC_CtrlData; 
extern xSemaphoreHandle xSemaphore_rcuart;
extern float yawAngleTarget, pitchAngleTarget;
extern uint8_t g_isGYRO_Rested ;
extern int twist_state ;

extern WorkState_e g_workState;//张雁大符

//static uint32_t delayCnt = 500;	//用于按键e去抖

void RControlTask(void const * argument){
	uint8_t data[18];
	static int countwhile = 0;
	static TickType_t lastcount_rc;
	static TickType_t thiscount_rc;
	static uint8_t first_frame = 0;
	while(1){
		if(first_frame == 0)
		{
			MX_IWDG_Init();
		}
		HAL_IWDG_Refresh(&hiwdg);
		/*等待串口接收中断回调函数释放信号量*/
		xSemaphoreTake(xSemaphore_rcuart, osWaitForever);
		//fw_printfln("RC is running");
		/*获取两帧时间间隔，正常14ms，大于16ms认为错误*/
		thiscount_rc = xTaskGetTickCount();

		if( ((thiscount_rc - lastcount_rc) <= 16) && (first_frame == 1))//第一帧认为错误
		{
			/*从IOPool读数据到数组*/
			IOPool_getNextWrite(rcUartIOPool);
			if(IOPool_hasNextRead(rcUartIOPool, 0))
			{
				IOPool_getNextRead(rcUartIOPool, 0);
				uint8_t *pData = IOPool_pGetReadData(rcUartIOPool, 0)->ch;
				for(uint8_t i = 0; i != 18; ++i)
				{
					data[i] = pData[i];
				}

				/*处理数据*/
				RemoteDataProcess(data);	//process raw data then execute new order
				/*扔掉多余数据，重新开启接收中断*/
				vTaskDelay(2 / portTICK_RATE_MS);
				HAL_UART_AbortReceive(&RC_UART);
				HAL_UART_Receive_DMA(&RC_UART, IOPool_pGetWriteData(rcUartIOPool)->ch, 18);

				if(countwhile >= 300){
					countwhile = 0;
//			    fw_printf("ch0 = %d | ", RC_CtrlData.rc.ch0);
//				fw_printf("ch1 = %d | ", RC_CtrlData.rc.ch1);
//				fw_printf("ch2 = %d | ", RC_CtrlData.rc.ch2);
//				fw_printf("ch3 = %d \r\n", RC_CtrlData.rc.ch3);
//				
//				fw_printf("s1 = %d | ", RC_CtrlData.rc.s1);
//				fw_printf("s2 = %d \r\n", RC_CtrlData.rc.s2);
//				
//				fw_printf("x = %d | ", RC_CtrlData.mouse.x);
//				fw_printf("y = %d | ", RC_CtrlData.mouse.y);
//				fw_printf("z = %d | ", RC_CtrlData.mouse.z);
//				fw_printf("l = %d | ", RC_CtrlData.mouse.press_l);
//				fw_printf("r = %d \r\n", RC_CtrlData.mouse.press_r);
//				
//				fw_printf("key = %d \r\n", RC_CtrlData.key.v);
//				fw_printf("===========\r\n");
				}else{
					countwhile++;
				}
	    }
		}
		else{
			/*错误帧等待2ms后清空缓存，开启中断*/
			//fw_printfln("RC discarded");
			first_frame = 1;
			vTaskDelay(2 / portTICK_RATE_MS);
			HAL_UART_AbortReceive(&RC_UART);
			HAL_UART_Receive_DMA(&RC_UART, IOPool_pGetWriteData(rcUartIOPool)->ch, 18);
		}
		lastcount_rc = thiscount_rc;
	}
}

bool g_switchRead = 0;

void RemoteDataProcess(uint8_t *pData)
{
	if(pData == NULL)
	{
			return;
	}
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
											 ((int16_t)pData[4] << 10)) & 0x07FF;
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

	RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];

	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);//16 bits correspond to 16 keys
	
	SetInputMode(&RC_CtrlData.rc);
	
		/*左上角拨杆状态获取*/
	GetRemoteSwitchAction(&g_switch1, RC_CtrlData.rc.s1);
	g_switchRead = 1;
	
	zySetLeftMode(&RC_CtrlData.rc);//张雁大符

	switch(GetInputMode())
	{
		case REMOTE_INPUT:
		{
			if(GetWorkState() == NORMAL_STATE)
			{ //if gyro has been reseted
//				fw_printfln("RC is running");
				RemoteControlProcess(&(RC_CtrlData.rc));//遥控器模式
			}
		}break;
		case KEY_MOUSE_INPUT:
		{
			if(GetWorkState() != PREPARE_STATE)
			{
//				if(RC_CtrlData.rc.s1==3)
//				{
//					g_workState=RUNE_STATE;
//				}
//				else
//				{
					MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key);//键鼠模式
					SetShootMode(AUTO);//调试自瞄用
	//			RemoteShootControl(&g_switch1, RC_CtrlData.rc.s1);
				//}
			}
//			else if(GetWorkState()==RUNE_STATE&&RC_CtrlData.rc.s1!=3)
//			{
//				g_workState=NORMAL_STATE;
//			}
		}break;
		case STOP:
		{
			 //停止
		}break;
	}
}

void RemoteControlProcess(Remote *rc)
{
	if(GetWorkState()!=PREPARE_STATE)
	{
		SetShootMode(MANUL);
		ChassisSpeedRef.forward_back_ref = (RC_CtrlData.rc.ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
		ChassisSpeedRef.left_right_ref   = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT; 
		
 		pitchAngleTarget += (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
		yawAngleTarget   -= (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT;

		if(rc->ch3 == 0x16C)
		{
			//twist_state = 1;
			int id = 0, pwm = 2400, time = 0;
			char ServoMes[15];
			sprintf(ServoMes, "#%03dP%04dT%04d!", id, pwm, time);
			HAL_UART_Transmit(&SERVO_UART,(uint8_t *)&ServoMes, 15, 0xFFFF);
		}
		else 
		{
//			twist_state = 0;
			int id = 0, pwm = 500, time = 0;
			char ServoMes[15];
			sprintf(ServoMes, "#%03dP%04dT%04d!", id, pwm, time);
			HAL_UART_Transmit(&SERVO_UART,(uint8_t *)&ServoMes, 15, 0xFFFF);
		}
	}
	//RemoteShootControl(&g_switch1, rc->s1);
}


extern uint8_t JUDGE_State;

//为不同操作手调整鼠标灵敏度
#ifndef INFANTRY_1
  #define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.025f * 2
  #define MOUSE_TO_YAW_ANGLE_INC_FACT 		0.025f * 2
#else
  #define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.025f * 2
  #define MOUSE_TO_YAW_ANGLE_INC_FACT 		0.025f * 2
#endif

extern uint8_t waitRuneMSG[4];
extern uint8_t littleRuneMSG[4];
extern uint8_t bigRuneMSG[4];
uint16_t fbss;
float auto_kpx = 0.006f;
float auto_kpy = 0.006f;
extern uint8_t auto_getting;
extern uint16_t autoBuffer[10];
uint16_t tmpx,tmpy;
uint16_t auto_x_default = 320;
uint16_t auto_y_default = 380;
extern float friction_speed;
extern float now_friction_speed;
extern float realBulletSpeed;
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{
	//++delayCnt;
	if(dircnt > 8000)dircnt = 0;
	dircnt++;
	static uint16_t forward_back_speed = 0;
	static uint16_t left_right_speed = 0;
	if(GetWorkState() == NORMAL_STATE)
	{
		VAL_LIMIT(mouse->x, -150, 150); 
		VAL_LIMIT(mouse->y, -150, 150); 
		#ifdef INFANTRY_4
		VAL_LIMIT(mouse->x, -100, 100); 
		VAL_LIMIT(mouse->y, -100, 100); 
		#endif
	
		tmpx = (0x0000 | autoBuffer[2] | autoBuffer[1]<<8);
		tmpy = (0x0000 | autoBuffer[5] | autoBuffer[4]<<8);
		if((autoBuffer[3] == 0xA6 || autoBuffer[3] == 0xA8) && (key->v&256))
		{
			pitchAngleTarget -= (tmpy - auto_y_default) * auto_kpy;
			yawAngleTarget -= (tmpx - auto_x_default) * auto_kpx;
		}
		else
		{
			pitchAngleTarget -= mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT;  
			yawAngleTarget    -= mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;
		}

		//speed mode: normal speed/high speed 
		forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
		left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
		fbss = forward_back_speed;
		if(key->v & 0x10)//Shift
		{
			going = 1;
			forward_back_speed =  LOW_FORWARD_BACK_SPEED;
			left_right_speed = LOW_LEFT_RIGHT_SPEED;
			int id = 0, pwm = 2400, time = 0;
			char ServoMes[15];
			sprintf(ServoMes, "#%03dP%04dT%04d!", id, pwm, time);
			HAL_UART_Transmit(&SERVO_UART,(uint8_t *)&ServoMes, 15, 0xFFFF);
		}
		else 
		{
			going = 0;
			int id = 0, pwm = 500, time = 0;
			char ServoMes[15];
			sprintf(ServoMes, "#%03dP%04dT%04d!", id, pwm, time);
			HAL_UART_Transmit(&SERVO_UART,(uint8_t *)&ServoMes, 15, 0xFFFF);
		}
		if(key->v & 0x20)//Ctrl
		{
			burst = 1;
//			forward_back_speed =  MIDDLE_FORWARD_BACK_SPEED;
//			left_right_speed = MIDDLE_LEFT_RIGHT_SPEED;
		}
		else
		{
			burst = 0;
//			forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
//			left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
		}
		
		//movement process
		if(key->v & 0x01)  // key: w
		{
			ChassisSpeedRef.forward_back_ref = forward_back_speed* FBSpeedRamp.Calc(&FBSpeedRamp);
			twist_state = 0;
		}
		else if(key->v & 0x02) //key: s
		{
			ChassisSpeedRef.forward_back_ref = -forward_back_speed* FBSpeedRamp.Calc(&FBSpeedRamp);
			twist_state = 0;
		}
		else
		{
			ChassisSpeedRef.forward_back_ref = 0;
			FBSpeedRamp.ResetCounter(&FBSpeedRamp);
		}
		if(key->v & 0x04)  // key: d
		{
			ChassisSpeedRef.left_right_ref = -left_right_speed* LRSpeedRamp.Calc(&LRSpeedRamp);
			twist_state = 0;
		}
		else if(key->v & 0x08) //key: a
		{
			ChassisSpeedRef.left_right_ref = left_right_speed* LRSpeedRamp.Calc(&LRSpeedRamp);
			twist_state = 0;
		}
		else
		{
			ChassisSpeedRef.left_right_ref = 0;
			LRSpeedRamp.ResetCounter(&LRSpeedRamp);
		}
		if(key->v & 0x80)	//key:e  检测第8位是不是1
		{
			if(shootdir < 45 && dircnt >= 30)
			{
				shootdir += 45;
				dircnt = 0;
			}
//			setLaunchMode(SINGLE_MULTI);
//			if(delayCnt>500)
//			{
//				toggleLaunchMode();
//				delayCnt = 0;
//			}
		}
		if(key->v & 0x40)	//key:q
		{
			if(shootdir > -45 && dircnt >= 30)
			{
				shootdir -= 45;
				dircnt = 0;
			}
//			setLaunchMode(CONSTENT_4);
		}
		
		if(key->v == 2048)//z
		{
			now_friction_speed = 1250;
			friction_speed = 1250;
			LASER_ON(); 
			realBulletSpeed = 11.0f;
		}
		if(key->v == 4096)//x
		{
			now_friction_speed = 1500;
			friction_speed = 1500;
			LASER_ON(); 
			realBulletSpeed = 22.0f;
		}
		if(key->v == 8192)//c
		{
			now_friction_speed = 10000;
			friction_speed = 10000;
			LASER_ON();
			realBulletSpeed = 28.0f;
		}
		
		/*裁判系统离线时的功率限制方式*/
		if(JUDGE_State == OFFLINE)
		{
			if(abs(ChassisSpeedRef.forward_back_ref) + abs(ChassisSpeedRef.left_right_ref) > 500)
			{
				if(ChassisSpeedRef.forward_back_ref > 325)
				{
				ChassisSpeedRef.forward_back_ref =  325 +  (ChassisSpeedRef.forward_back_ref - 325) * 0.15f;
				}
				else if(ChassisSpeedRef.forward_back_ref < -325)
				{
				ChassisSpeedRef.forward_back_ref =  -325 +  (ChassisSpeedRef.forward_back_ref + 325) * 0.15f;
				}
				if(ChassisSpeedRef.left_right_ref > 300)
				{
				ChassisSpeedRef.left_right_ref =  300 +  (ChassisSpeedRef.left_right_ref - 300) * 0.15f;
				}
				else if(ChassisSpeedRef.left_right_ref < -300)
				{
				ChassisSpeedRef.left_right_ref =  -300 +  (ChassisSpeedRef.left_right_ref + 300) * 0.15f;
				}
			}

			if ((mouse->x < -2.6) || (mouse->x > 2.6))
			{
				if(abs(ChassisSpeedRef.forward_back_ref) + abs(ChassisSpeedRef.left_right_ref) > 400)
				{
					if(ChassisSpeedRef.forward_back_ref > 250){
					 ChassisSpeedRef.forward_back_ref =  250 +  (ChassisSpeedRef.forward_back_ref - 250) * 0.15f;
					}
					else if(ChassisSpeedRef.forward_back_ref < -250)
					{
						ChassisSpeedRef.forward_back_ref =  -250 +  (ChassisSpeedRef.forward_back_ref + 250) * 0.15f;
					}
					if(ChassisSpeedRef.left_right_ref > 250)
					{
					 ChassisSpeedRef.left_right_ref =  250 +  (ChassisSpeedRef.left_right_ref - 250) * 0.15f;
					}
					else if(ChassisSpeedRef.left_right_ref < -250)
					{
						ChassisSpeedRef.left_right_ref =  -250 +  (ChassisSpeedRef.left_right_ref + 250) * 0.15f;
					}
				}
			}
		}
		
		if(key->v == 256)  // key: r
		{
			cancel_chassis_rotate = 1;
		}
		else
		{
			cancel_chassis_rotate = 0;
		}
		if(key->v == 272)  // key: r+Shift
		{
//			twist_state = 0;
//			int id = 0, pwm = 500, time = 0;
//			char ServoMes[15];
//			sprintf(ServoMes, "#%03dP%04dT%04d!", id, pwm, time);
//			HAL_UART_Transmit(&SERVO_UART,(uint8_t *)&ServoMes, 15, 0xFFFF);
		}
		if(key->v == 1024)//��key: G
		{
			twist_state = 1;
		}
		if(key->v == 1040)  // key: G+Shift
		{
			twist_state = 0;
		}

		MouseShootControl(mouse,key);
	}
	else if(GetWorkState() == RUNE_STATE)
	{
		VAL_LIMIT(mouse->x, -150, 150); 
		VAL_LIMIT(mouse->y, -150, 150); 
	
		pitchAngleTarget -= mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT;  
		yawAngleTarget    -= mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;

		switch(RC_CtrlData.key.v)
		{
			case 64://q
			{
				uint8_t location = 0;
				ShootRune(location);
			}break;
			case 1://w
			{
				uint8_t location = 1;
				ShootRune(location);
			}break;
			case 128://e
			{
				uint8_t location = 2;
				ShootRune(location);
			}break;
			case 4://a
			{
				uint8_t location = 3;
				ShootRune(location);
			}break;
			case 2://s
			{
				uint8_t location = 4;
				ShootRune(location);
			}break;
			case 8://d
			{
				uint8_t location = 5;
				ShootRune(location);
			}break;
			case 2048://z
			{
				uint8_t location = 6;
				ShootRune(location);
			}break;
			case 4096://x
			{
				uint8_t location = 7;
				ShootRune(location);
			}break;
			case 8192://c
			{
				uint8_t location = 8;
				ShootRune(location);
			}break;
			default:
			{
			}
		}
		if(RC_CtrlData.key.v == 1024)//小符 G
		{
			//LASER_OFF();
			zyRuneMode=5;
			HAL_UART_Transmit(&MANIFOLD_UART , (uint8_t *)&littleRuneMSG, 4, 0xFFFF);
		}else if(RC_CtrlData.key.v == 32768)//大符 B
		{
			//LASER_OFF();
			zyRuneMode=3;
			HAL_UART_Transmit(&MANIFOLD_UART , (uint8_t *)&bigRuneMSG, 4, 0xFFFF);
		}
	}
}





