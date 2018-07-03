/**
  ******************************************************************************
  * File Name          : application_motorcontrol.c
  * Description        : 电机控制驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 设定电机电流
	* 陀螺仪复位
  ******************************************************************************
  */
#include "application_motorcontrol.h"
#include "drivers_uartrc_user.h"
#include "can.h"
#include "peripheral_define.h"
#include "drivers_canmotor_user.h"
#include "rtos_semaphore.h"
#include "utilities_debug.h"
#include "tasks_timed.h"
#include "math.h"
#include <math.h>
#include <stdlib.h>
#include "tasks_motor.h"
#include "drivers_uartjudge_low.h"
#include "drivers_cmpower.h"
#include <stdbool.h>
#include "tasks_remotecontrol.h"

extern extPowerHeatData_t PowerHeatData;
extern uint8_t JUDGE_State;
extern uint8_t going;
extern float realPowerBuffer;
extern float realPower;
float mxcc;
extern uint32_t ADC_Value[120];
extern bool g_bInited;
int32_t ad1=0;

void setMotor(MotorId motorId, int16_t Intensity){
	static int16_t CMFLIntensity = 0, CMFRIntensity = 0, CMBLIntensity = 0, CMBRIntensity = 0;
	static int8_t CMReady = 0;
	
	static int16_t GMYAWIntensity = 0, GMPITCHIntensity = 0;
	static int8_t GMReady = 0;
	
	static int16_t PLATEIntensity = 0;
	
	static int16_t LFRICTIONIntensity = 0;
	static int16_t RFRICTIONIntensity = 0;
	static int16_t FRICTIONReady = 0;
	switch(motorId)
	{
		case CMFL:
			if(CMReady & 0x1){CMReady = 0xF;}else{CMReady |= 0x1;}
			CMFLIntensity = Intensity;break;
		case CMFR:
			if(CMReady & 0x2){CMReady = 0xF;}else{CMReady |= 0x2;}
			CMFRIntensity = Intensity;break;
		case CMBL:
			if(CMReady & 0x4){CMReady = 0xF;}else{CMReady |= 0x4;}
			CMBLIntensity = Intensity;break;
		case CMBR:
			if(CMReady & 0x8){CMReady = 0xF;}else{CMReady |= 0x8;}
			CMBRIntensity = Intensity;break;
		
		case GMYAW:
			if(GMReady & 0x1){GMReady = 0x7;}else{GMReady |= 0x1;}
			GMYAWIntensity = Intensity;break;
		case GMPITCH:
			if(GMReady & 0x2){GMReady = 0x7;}else{GMReady |= 0x2;}
			GMPITCHIntensity = Intensity;break;
		
		case PLATE:
			if(GMReady & 0x4){GMReady = 0x7;}else{GMReady |= 0x4;}
			PLATEIntensity = Intensity;break;
		
		case LFRICTION:
			if(FRICTIONReady & 0x1){FRICTIONReady = 0x3;}else{FRICTIONReady |= 0x1;}
			LFRICTIONIntensity = Intensity;break;
		case RFRICTION:
			if(FRICTIONReady & 0x2){FRICTIONReady = 0x3;}else{FRICTIONReady |= 0x2;}
			RFRICTIONIntensity = Intensity;break;

			
		default:
			fw_Error_Handler();
	}
	
	for(uint16_t i=0;i<120;i++)
	{
		ad1+=ADC_Value[i];
	}
	ad1/=120;

	//底盘功率限制，80W，能量槽满60，低于0掉血
//    RestrictPower(&CMFLIntensity, &CMFRIntensity, &CMBLIntensity, &CMBRIntensity);
	//ׁԶ٦Ê޸ѐ֧·О׆
//ĬɏһО
	float CM_current_max = CM_current_MAX;
	float CMFLIntensity_max = CMFLIntensity_MAX;
	float CMFRIntensity_max = CMFRIntensity_MAX;
	float CMBLIntensity_max = CMBLIntensity_MAX;
	float CMBRIntensity_max = CMBRIntensity_MAX;
	
	//40-50
	//10-40ԵҽО׆
//	if (realPowerBuffer > 40)
//	{
//		 CM_current_max = CM_current_MAX;
//		 CMFLIntensity_max = CMFLIntensity_MAX;
//		 CMFRIntensity_max = CMFRIntensity_MAX;
//		 CMBLIntensity_max = CMBLIntensity_MAX;
//		 CMBRIntensity_max = CMBRIntensity_MAX;
//	}
//	else if (realPowerBuffer > 25)
//	{
//		 CM_current_max = CM_current_lower;
//		 CMFLIntensity_max = CMFLIntensity_lower;
//		 CMFRIntensity_max = CMFRIntensity_lower;
//		 CMBLIntensity_max = CMBLIntensity_lower;
//		 CMBRIntensity_max = CMBRIntensity_lower;
//	}
//	//0-10ܫОО׆
//	else if (realPowerBuffer >= 10 ){
//			
//		 CM_current_max = CM_current_bottom;
//		 CMFLIntensity_max = CMFLIntensity_bottom;
//		 CMFRIntensity_max = CMFRIntensity_bottom;
//		 CMBLIntensity_max = CMBLIntensity_bottom;
//		 CMBRIntensity_max = CMBRIntensity_bottom;
//	}
//	//߸הһӬ
//	if (realPowerBuffer < 10 ){
//			
//		 CM_current_max = 0;
//		 CMFLIntensity_max = 0;
//		 CMFRIntensity_max = 0;
//		 CMBLIntensity_max = 0;
//		 CMBRIntensity_max = 0;
//	}

	if (JUDGE_State == OFFLINE)
	{
		 CM_current_max = 13000;
		 CMFLIntensity_max = 4500;
		 CMFRIntensity_max = 4500;
		 CMBLIntensity_max = 4500;
		 CMBRIntensity_max = 4500;
	}
	if (going || GetInputMode() == REMOTE_INPUT)
	{
		 CM_current_max = 25000;
		 CMFLIntensity_max = 7000;
		 CMFRIntensity_max = 7000;
		 CMBLIntensity_max = 7000;
		 CMBRIntensity_max = 7000;
	}
	

//	float sum = (abs(CMFLIntensity) + abs(CMFRIntensity) + abs(CMBLIntensity) + abs(CMBRIntensity));
//	
//	if ((CMFLIntensity > CMFLIntensity_max))
//	{
//		CMFLIntensity = CMFLIntensity_max;
//	}
//	else if ((CMFLIntensity < -CMFLIntensity_max))
//	{
//		CMFLIntensity = -CMFLIntensity_max;
//	}
//	if(CMFRIntensity > CMFRIntensity_max)
//	{
//	  CMFRIntensity = CMFRIntensity_max;
//	}
//	else if(CMFRIntensity < -CMFRIntensity_max)
//	{
//	  CMFRIntensity = -CMFRIntensity_max;
//	}
//	if(CMBLIntensity > CMBLIntensity_max)
//	{
//	  CMBLIntensity = CMBLIntensity_max;
//	}
//	else if(CMBLIntensity < -CMBLIntensity_max)
//	{
//	  CMBLIntensity = -CMBLIntensity_max;
//	}
//	if(CMBRIntensity > CMBRIntensity_max)
//	{
//	  CMBRIntensity = CMBRIntensity_max;
//	}
//	else	if(CMBRIntensity < -CMBRIntensity_max)
//	{
//	  CMBRIntensity = -CMBRIntensity_max;
//	}
//	mxcc = CM_current_max;
//	if( sum > CM_current_max){
//		CMFLIntensity = (CM_current_max/sum)*CMFLIntensity;
//		CMFRIntensity = (CM_current_max/sum)*CMFRIntensity;
//		CMBLIntensity = (CM_current_max/sum)*CMBLIntensity;
//		CMBRIntensity = (CM_current_max/sum)*CMBRIntensity;
//	}
	
	if(realPower >= 70 && JUDGE_State == ONLINE)
	{
		CMFLIntensity = 0.6*CMFLIntensity;
		CMFRIntensity = 0.6*CMFRIntensity;
		CMBLIntensity = 0.6*CMBLIntensity;
		CMBRIntensity = 0.6*CMBRIntensity;
	}
	
	if(GetWorkState() == STOP_STATE || g_bInited != 1)
	{
		CMFLIntensity = 0;
		CMFRIntensity = 0;
		CMBLIntensity = 0;
		CMBRIntensity = 0;
		GMYAWIntensity = 0;
		GMPITCHIntensity = 0;
		PLATEIntensity = 0;
	}

	if(CMReady == 0xF)
	{
		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(CMTxIOPool);
		
		pData->StdId = CM_TXID;
  	pData->Data[0] = (uint8_t)(CMFLIntensity >> 8);
		pData->Data[1] = (uint8_t)CMFLIntensity;
		pData->Data[2] = (uint8_t)(CMFRIntensity >> 8);
		pData->Data[3] = (uint8_t)CMFRIntensity;
		pData->Data[4] = (uint8_t)(CMBLIntensity >> 8);
		pData->Data[5] = (uint8_t)CMBLIntensity;
		pData->Data[6] = (uint8_t)(CMBRIntensity >> 8);
		pData->Data[7] = (uint8_t)CMBRIntensity;
 	
		IOPool_getNextWrite(CMTxIOPool);
		
		TransmitCMCan();
		CMReady = 0;
    }
	
	if(GMReady == 0x7)
	{
		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(GMTxIOPool);
		pData->StdId = GMPLATE_TXID;
		pData->Data[0] = (uint8_t)(GMYAWIntensity >> 8);
		pData->Data[1] = (uint8_t)GMYAWIntensity;
		pData->Data[2] = (uint8_t)(GMPITCHIntensity >> 8);
		pData->Data[3] = (uint8_t)GMPITCHIntensity;
		pData->Data[4] = (uint8_t)(PLATEIntensity >> 8);
		pData->Data[5] = (uint8_t)PLATEIntensity;
		pData->Data[6] = 0;
		pData->Data[7] = 0;
		IOPool_getNextWrite(GMTxIOPool);
		GMReady = 0;

    TransmitGM_FRICTION_PLATE_CAN();
	}
	if(FRICTIONReady == 0x3)
	{
		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(FRICTIONTxIOPool);
		
		pData->StdId = FRICTION_TXID;
  	pData->Data[0] = (uint8_t)(LFRICTIONIntensity >> 8);
		pData->Data[1] = (uint8_t)LFRICTIONIntensity;
		pData->Data[2] = (uint8_t)(RFRICTIONIntensity >> 8);
		pData->Data[3] = (uint8_t)RFRICTIONIntensity;
		pData->Data[4] = 0;
		pData->Data[5] = 0;
		pData->Data[6] = 0;
		pData->Data[7] = 0;
 	
		IOPool_getNextWrite(FRICTIONTxIOPool);
		
	  TransmitGM_FRICTION_PLATE_CAN();
		FRICTIONReady = 0;
    }
}
	

void GYRO_RST(void)
{
	CanTxMsgTypeDef *pData = IOPool_pGetWriteData(ZGYROTxIOPool);
	pData->StdId = ZGYRO_TXID;
	pData->Data[0] = 0x00;
	pData->Data[1] = 0x01;
	pData->Data[2] = 0x02;
	pData->Data[3] = 0x03;
	pData->Data[4] = 0x04;
	pData->Data[5] = 0x05;
	pData->Data[6] = 0x06;
	pData->Data[7] = 0x07;
	IOPool_getNextWrite(ZGYROTxIOPool);

	TransmitGM_FRICTION_PLATE_CAN();
}
