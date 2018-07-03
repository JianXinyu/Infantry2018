/**
  ******************************************************************************
  * File Name          : drivers_canmotor.c
  * Description        : 电机CAN总线驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * CAN总线初始化
	* CAN接收处理
	* CAN发送任务
	* CAN1 CMGM电机
	* CAN2 单轴陀螺仪
  ******************************************************************************
  */
#include <cmsis_os.h>
#include "stdint.h"
#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"
#include "peripheral_define.h"
#include "utilities_debug.h"
#include "utilities_iopool.h"
#include "utilities_tim.h"
#include "rtos_init.h"
#include "rtos_semaphore.h"


FilterParameter filter = {{1,-2.3741,1.9294,-0.5321},{0.0029,0.0087,0.0087,0.0029}};//50Hz


float ZGyroModuleAngle = 0.0f;
float ZGyroModuleRealAngle = 0.0f;
float ZGyroOffset = 0;

//RxIOPool
NaiveIOPoolDefine(CMFLRxIOPool, {0});
NaiveIOPoolDefine(CMFRRxIOPool, {0});
NaiveIOPoolDefine(CMBLRxIOPool, {0});
NaiveIOPoolDefine(CMBRRxIOPool, {0});

NaiveIOPoolDefine(GMPITCHRxIOPool, {0});
NaiveIOPoolDefine(GMYAWRxIOPool, {0});
NaiveIOPoolDefine(PLATERxIOPool, {0});
NaiveIOPoolDefine(LFRICTIONRxIOPool, {0});
NaiveIOPoolDefine(RFRICTIONRxIOPool, {0});

//TxIOPool
#define DataPoolInit \
	{ \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(CMTxIOPool, DataPoolInit);
#undef DataPoolInit 
	
#define DataPoolInit \
	{ \
		{GMPLATE_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{GMPLATE_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{GMPLATE_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(GMTxIOPool, DataPoolInit);
#undef DataPoolInit 

#define DataPoolInit \
	{ \
		{GMPLATE_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{GMPLATE_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{GMPLATE_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(PLATETxIOPool, DataPoolInit);
#undef DataPoolInit 
	
#define DataPoolInit \
	{ \
		{ZGYRO_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{ZGYRO_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{ZGYRO_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(ZGYROTxIOPool, DataPoolInit);
#undef DataPoolInit 
	
#define DataPoolInit \
	{ \
		{FRICTION_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{FRICTION_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{FRICTION_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(FRICTIONTxIOPool, DataPoolInit);
#undef DataPoolInit 

#define CanRxGetU16(canRxMsg, num) (((uint16_t)canRxMsg.Data[num * 2] << 8) | (uint16_t)canRxMsg.Data[num * 2 + 1])

uint8_t isRcanStarted_CMGM = 0, isRcanStarted_ZGYRO = 0;

CanRxMsgTypeDef CMCanRxMsg, GM_FRICTION_PLATECanRxMsg;
/********************CAN接收******************************/
void InitCanReception()
{
	//CAN接收过滤器配置
	//http://www.eeworld.com.cn/mcu/article_2016122732674_3.html
	CMCAN.pRxMsg = &CMCanRxMsg;
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef  sFilterConfig;
	sFilterConfig.FilterNumber = 0;//14 - 27//14
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  HAL_CAN_ConfigFilter(&CMCAN, &sFilterConfig);
	if(HAL_CAN_Receive_IT(&CMCAN, CAN_FIFO0) != HAL_OK){
		fw_Error_Handler(); 
	}
	isRcanStarted_CMGM = 1;
	
	GM_FRICTION_PLATE_CAN.pRxMsg = &GM_FRICTION_PLATECanRxMsg;
	/*##-- Configure the CAN1 Filter ###########################################*/
	CAN_FilterConfTypeDef sFilterConfig2;
	sFilterConfig2.FilterNumber = 14;//14 - 27//14
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig2.FilterIdHigh = 0x0000;
  sFilterConfig2.FilterIdLow = 0x0000;
  sFilterConfig2.FilterMaskIdHigh = 0x0000;
  sFilterConfig2.FilterMaskIdLow = 0x0000;
  sFilterConfig2.FilterFIFOAssignment = 0;
  sFilterConfig2.FilterActivation = ENABLE;
  sFilterConfig2.BankNumber = 14;
  HAL_CAN_ConfigFilter(&GM_FRICTION_PLATE_CAN, &sFilterConfig2);
	if(HAL_CAN_Receive_IT(&GM_FRICTION_PLATE_CAN, CAN_FIFO0) != HAL_OK){
		fw_Error_Handler(); 
	}
	isRcanStarted_ZGYRO = 1;
}
/*********************所有CAN接收终端****************************/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	if(hcan == &CMCAN){
		switch(CMCanRxMsg.StdId){
			case CMFL_RXID:
				//读取0 1字节为角度，2 3字节为速度
				IOPool_pGetWriteData(CMFLRxIOPool)->angle = CanRxGetU16(CMCanRxMsg, 0);
				IOPool_pGetWriteData(CMFLRxIOPool)->RotateSpeed = CanRxGetU16(CMCanRxMsg, 1);
				IOPool_getNextWrite(CMFLRxIOPool);
				break;
			case CMFR_RXID:
				IOPool_pGetWriteData(CMFRRxIOPool)->angle = CanRxGetU16(CMCanRxMsg, 0);
				IOPool_pGetWriteData(CMFRRxIOPool)->RotateSpeed = CanRxGetU16(CMCanRxMsg, 1);
				IOPool_getNextWrite(CMFRRxIOPool);
				break;
			case CMBL_RXID:
				IOPool_pGetWriteData(CMBLRxIOPool)->angle = CanRxGetU16(CMCanRxMsg, 0);
				IOPool_pGetWriteData(CMBLRxIOPool)->RotateSpeed = CanRxGetU16(CMCanRxMsg, 1);
				IOPool_getNextWrite(CMBLRxIOPool);
				break;
			case CMBR_RXID:
				IOPool_pGetWriteData(CMBRRxIOPool)->angle = CanRxGetU16(CMCanRxMsg, 0);
				IOPool_pGetWriteData(CMBRRxIOPool)->RotateSpeed = CanRxGetU16(CMCanRxMsg, 1);
				IOPool_getNextWrite(CMBRRxIOPool);
				break;
//			case ZGYRO_RXID:
//			 {
//				//单轴陀螺仪没有datasheet，完全参照官方程序
//				CanRxMsgTypeDef *msg = &GM_FRICTION_PLATECanRxMsg;
//				ZGyroModuleAngle = -0.01f*((int32_t)(msg->Data[0]<<24)|(int32_t)(msg->Data[1]<<16) | (int32_t)(msg->Data[2]<<8) | (int32_t)(msg->Data[3])); 
//				ZGyroModuleRealAngle = GetZGyroFilterData(ZGyroModuleAngle);
//				ZGyroModuleAngle = ZGyroModuleRealAngle;
//			 }
//			 break;
			default:
			fw_Error_Handler();
		}
		//HAL CAN总线存在一定bug，使用自己的标志位
		if(HAL_CAN_Receive_IT(&CMCAN, CAN_FIFO0) != HAL_OK){
			fw_Warning();
			isRcanStarted_CMGM = 0;
		}else{
			isRcanStarted_CMGM = 1;
		}
		if(g_bInited == 1){
			//释放信号量交给控制任务
			osSemaphoreRelease(CMGMCanRefreshSemaphoreHandle);
		}
	}
	else if(hcan == &GM_FRICTION_PLATE_CAN)
	{
		switch(GM_FRICTION_PLATECanRxMsg.StdId)
		{
			case GMYAW_RXID:
				IOPool_pGetWriteData(GMYAWRxIOPool)->angle = CanRxGetU16(GM_FRICTION_PLATECanRxMsg, 0);
				IOPool_pGetWriteData(GMYAWRxIOPool)->realIntensity = CanRxGetU16(GM_FRICTION_PLATECanRxMsg, 1);
				IOPool_pGetWriteData(GMYAWRxIOPool)->giveIntensity = CanRxGetU16(GM_FRICTION_PLATECanRxMsg, 2);
				IOPool_getNextWrite(GMYAWRxIOPool);
				break;
			case GMPITCH_RXID:
				IOPool_pGetWriteData(GMPITCHRxIOPool)->angle = CanRxGetU16(GM_FRICTION_PLATECanRxMsg, 0);
				IOPool_pGetWriteData(GMPITCHRxIOPool)->realIntensity = CanRxGetU16(GM_FRICTION_PLATECanRxMsg, 1);
				IOPool_pGetWriteData(GMPITCHRxIOPool)->giveIntensity = CanRxGetU16(GM_FRICTION_PLATECanRxMsg, 2);
				IOPool_getNextWrite(GMPITCHRxIOPool);
				break;
			case LFRICTION_RXID:
				//读取0 1字节为角度，2 3字节为速度
				IOPool_pGetWriteData(LFRICTIONRxIOPool)->angle = CanRxGetU16(GM_FRICTION_PLATECanRxMsg, 0);
				IOPool_pGetWriteData(LFRICTIONRxIOPool)->RotateSpeed = CanRxGetU16(GM_FRICTION_PLATECanRxMsg, 1);
				IOPool_getNextWrite(LFRICTIONRxIOPool);
				break;
			case RFRICTION_RXID:
				//读取0 1字节为角度，2 3字节为速度
				IOPool_pGetWriteData(RFRICTIONRxIOPool)->angle = CanRxGetU16(GM_FRICTION_PLATECanRxMsg, 0);
				IOPool_pGetWriteData(RFRICTIONRxIOPool)->RotateSpeed = CanRxGetU16(GM_FRICTION_PLATECanRxMsg, 1);
				IOPool_getNextWrite(RFRICTIONRxIOPool);
				break;
			case PLATE_RXID:
				IOPool_pGetWriteData(PLATERxIOPool)->angle = CanRxGetU16(GM_FRICTION_PLATECanRxMsg, 0);
				IOPool_pGetWriteData(PLATERxIOPool)->RotateSpeed = CanRxGetU16(GM_FRICTION_PLATECanRxMsg, 1);
				IOPool_getNextWrite(PLATERxIOPool);
				break;
			default:
			fw_Error_Handler();
		}
		if(HAL_CAN_Receive_IT(&GM_FRICTION_PLATE_CAN, CAN_FIFO0) != HAL_OK)
		{
			//fw_Warning();
			isRcanStarted_ZGYRO = 0;
		}else{
			isRcanStarted_ZGYRO = 1;
		}
		if(g_bInited == 1){
			//这个信号量并没有人理0.o
			osSemaphoreRelease(ZGYROCanRefreshSemaphoreHandle);
		}
	}
}
/********************CAN发送*****************************/
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan == &CMCAN){
		osSemaphoreRelease(CMGMCanTransmitSemaphoreHandle);
	}
	else if(hcan == &GM_FRICTION_PLATE_CAN)
	{
		osSemaphoreRelease(ZGYROCanTransmitSemaphoreHandle);
	}
}

void TransmitCMCan(void)
{
		if(IOPool_hasNextRead(CMTxIOPool, 0))
		{
			//使用信号量保护CAN发送资源，在发送中断中release
			osSemaphoreWait(CMGMCanTransmitSemaphoreHandle, osWaitForever);
			
			IOPool_getNextRead(CMTxIOPool, 0);
			CMCAN.pTxMsg = IOPool_pGetReadData(CMTxIOPool, 0);
			//使用临界区避免被抢占
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&CMCAN) != HAL_OK){
				fw_Warning();
			}
			taskEXIT_CRITICAL();
		}
}

void TransmitGM_FRICTION_PLATE_CAN(void){
//	if(IOPool_hasNextRead(ZGYROTxIOPool, 0))
//	{
//			osSemaphoreWait(ZGYROCanTransmitSemaphoreHandle, osWaitForever);
//		
//			IOPool_getNextRead(ZGYROTxIOPool, 0);
//			GM_FRICTION_PLATE_CAN.pTxMsg = IOPool_pGetReadData(ZGYROTxIOPool, 0);
//		
//			taskENTER_CRITICAL();
//			if(HAL_CAN_Transmit_IT(&GM_FRICTION_PLATE_CAN) != HAL_OK)
//			{
//				fw_Warning();
//			}
//			taskEXIT_CRITICAL();
//	}
		if(IOPool_hasNextRead(GMTxIOPool, 0))
		{
			osSemaphoreWait(ZGYROCanTransmitSemaphoreHandle, osWaitForever);
			
			IOPool_getNextRead(GMTxIOPool, 0);
			GM_FRICTION_PLATE_CAN.pTxMsg = IOPool_pGetReadData(GMTxIOPool, 0);
			
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&GM_FRICTION_PLATE_CAN) != HAL_OK){
				fw_Warning();
			}
			taskEXIT_CRITICAL();
		}
		if(IOPool_hasNextRead(PLATETxIOPool, 0))
		{
			osSemaphoreWait(ZGYROCanTransmitSemaphoreHandle, osWaitForever);
		
			IOPool_getNextRead(PLATETxIOPool, 0);
			GM_FRICTION_PLATE_CAN.pTxMsg = IOPool_pGetReadData(PLATETxIOPool, 0);
		
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&GM_FRICTION_PLATE_CAN) != HAL_OK)
			{
				fw_Warning();
			}
			taskEXIT_CRITICAL();
		}
	if(IOPool_hasNextRead(FRICTIONTxIOPool, 0))
	{
		osSemaphoreWait(ZGYROCanTransmitSemaphoreHandle, osWaitForever);
		
		IOPool_getNextRead(FRICTIONTxIOPool, 0);
		GM_FRICTION_PLATE_CAN.pTxMsg = IOPool_pGetReadData(FRICTIONTxIOPool, 0);
		
		taskENTER_CRITICAL();
		if(HAL_CAN_Transmit_IT(&GM_FRICTION_PLATE_CAN) != HAL_OK){
			fw_Warning();
		}
		taskEXIT_CRITICAL();
	}
}




/*
***********************************************************************************************
*Name          :EncoderProcess
*Input         :can message
*Return        :void
*Description   :对编码器数据进行处理得到速度
*               copy from官方程序
*								步兵未用到
*								值得参考
*
*
***********************************************************************************************
*/
void EncoderProcess(volatile Encoder *v, Motor820RRxMsg_t * msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = msg->angle;
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -7500)    
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>7500)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}

	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);					
}

void GetEncoderBias(volatile Encoder *v, Motor820RRxMsg_t * msg)
{
	v->ecd_bias = msg->angle;  
	v->ecd_value = v->ecd_bias;
	v->last_raw_value = v->ecd_bias;
	v->temp_count++;
}

float GyroDeOffset(float ZGyro)
{
	static float last_ZGyro = 0;
	float temp = 0;
	temp = ZGyro - last_ZGyro-ZGyroOffset;
	last_ZGyro = ZGyro;
	return temp;

}


void GetZGyroOffset(float ZGyro)
{
	static int i = 0;
	static float first_ZGyro = 0;
	if(i>=100)
		return;
	if(i == 0)
		first_ZGyro = ZGyro;
	
	i++;
	if(i ==100)
	{
		ZGyroOffset = (ZGyro-first_ZGyro)/100.0f;
	}
}

float GetZGyroFilterData(float ZGyro)
{
	static float inZGyro[4] = {0};
	static float outZGyro[4] = {0};
	
	inZGyro[0] = ZGyro;
	outZGyro[0] = (filter.b[3]*inZGyro[3]+filter.b[2]*inZGyro[2]+filter.b[1]*inZGyro[1]+filter.b[0]*inZGyro[0] - \
								(filter.a[3]*outZGyro[3]+filter.a[2]*outZGyro[2]+filter.a[1]*outZGyro[1]))/filter.a[0];
	
	inZGyro[3] = inZGyro[2];
	inZGyro[2] = inZGyro[1];
	inZGyro[1] = inZGyro[0];
	
	outZGyro[3] = outZGyro[2];
	outZGyro[2] = outZGyro[1];
	outZGyro[1] = outZGyro[0];
	
	return outZGyro[0];
}