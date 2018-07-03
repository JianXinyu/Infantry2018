/**
  ******************************************************************************
  * File Name          : tasks_upper.c
  * Description        : 与上位机(妙算)通信任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 通信串口数据获取
  ******************************************************************************
  */
#include <stdint.h>
#include <cmsis_os.h>
#include "tasks_upper.h"
#include "drivers_uartupper_user.h"
#include "drivers_uartupper_low.h"
#include "rtos_semaphore.h"
#include "drivers_flash.h"
#include "utilities_debug.h"


NaiveIOPoolDefine(upperIOPool, {0});

extern uint16_t yawAngle, pitchAngle;
int forPidDebug = 0;

extern float yawAngleTarget, pitchAngleTarget;
extern xSemaphoreHandle xSemaphore_mfuart;
extern xdata_ctrlUart ctrlData; 
extern uint16_t x;
uint8_t CReceive = 0;
uint8_t rune_flag = 0;
float yawAdd = 0;
float last_yawAdd = 0;
float yaw_speed = 0;


extern float pitchRealAngle;
extern float ZGyroModuleAngle;	//陀螺仪角度
//extern float yawAngleTarget, pitchAngleTarget;
void ManifoldUartTask(void const * argument){
	while(1){
			xSemaphoreTake(xSemaphore_mfuart, osWaitForever);
	//		fw_printfln("Ctrl");
			uint8_t *pData = IOPool_pGetReadData(ctrlUartIOPool, 0)->ch;
//			fw_printfln("manifold task:%d",*pData);
			
	//		ctrlData = xUartprocess( pData );
	//		if( ctrlData.Success == 1) {
	//			yawAdd = ((float)ctrlData.dev_yaw - 9000)/100;
	//			pitchAdd = ((float)ctrlData.dev_pitch - 5000)/100;
	////			yawAdd = 0;
	////			fw_printfln("yawAdd:%f",yawAdd);
	////			fw_printfln("pitchAdd:%f",pitchAdd);
	//			if((yawAdd != 0) || (pitchAdd != 0)){
	//			yaw_speed = yawAdd - last_yawAdd;
	//			IOPool_pGetWriteData(upperIOPool)->yawAdd = yawAdd ;//+ 2.0f*yaw_speed;
	//			IOPool_pGetWriteData(upperIOPool)->pitchAdd = pitchAdd;
	////			IOPool_pGetWriteData(upperIOPool)->rune = ctrlData.rune;
	////			IOPool_getNextWrite(upperIOPool);
	//			last_yawAdd = yawAdd;
	//			CReceive = 2;
	//			}
	////ճʱػ֨λ
	//				if(ctrlData.rune_locate != 0){
	//					SetLocateState(Locating);
	//				}
	//				/////
	//				else if((ctrlData.rune_locate == 0) && (last_rune_locate != 0)){
	//					SetLocateState(Located);
	//					fw_printfln("Located");
	////					fw_printfln("ZGyroModuleAngle:  %f",ZGyroModuleAngle);
	//					vRefreshLocation( -ZGyroModuleAngle, pitchRealAngle);
	//					fw_printfln("pitchRealAngle:  %f",pitchRealAngle);
	//				 }
	//				last_rune_locate = ctrlData.rune_locate;
	////ճʱػղܷ
	//				if(ctrlData.rune != 0){
	//				IOPool_pGetWriteData(upperIOPool)->rune = ctrlData.rune;
	//				SetRuneState(AIMING);
	//				}
	//				else{
	//				IOPool_pGetWriteData(upperIOPool)->rune = 0;
	//				SetRuneState(NOAIMING);
	//				}
	// 				if((ctrlData.rune != last_rune) || (ctrlData.rune == 10)){
	//					rune_flag = 2;
	//					if(ctrlData.rune == 10){
	//						IOPool_pGetWriteData(upperIOPool)->rune = last_rune;
	//					}
	//				}
	//				IOPool_getNextWrite(upperIOPool);
	//				last_rune = ctrlData.rune;
	//			} 
	//			else {
	//			yawAdd = 0;
	//			pitchAdd = 0;
	//			CReceive = 0;
	//		  }
	 }
}

	
