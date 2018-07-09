/**
  ******************************************************************************
  * File Name          : pdrivers_uartjudge_low.h
  * Description        : 裁判系统读取
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 底层函数
  ******************************************************************************
  */
#ifndef DRIVERS_UARTJUDGE_LOW_H
#define DRIVERS_UARTJUDGE_LOW_H

#include "utilities_iopool.h"
#include "cmsis_os.h"
typedef __packed struct
{
uint16_t stageRemianTime;
uint8_t gameProgress;
uint8_t robotLevel;
uint16_t remainHP;
uint16_t maxHP;
}extGameRobotState_t;
typedef __packed struct
{
uint8_t armorType : 4;
uint8_t hurtType : 4;
}extRobotHurt_t;
typedef __packed struct
{
uint8_t bulletType;
uint8_t bulletFreq;
float bulletSpeed;
}extShootData_t;
typedef __packed struct
{
float chassisVolt;
float chassisCurrent;
float chassisPower;
float chassisPowerBuffer;
uint16_t shooterHeat0;
uint16_t shooterHeat1;
}extPowerHeatData_t;
typedef __packed struct
{
uint8_t cardType;
uint8_t cardIdx;
}extRfidDetect_t;
typedef __packed struct
{
uint8_t winner;
}extGameResult_t;
typedef __packed struct
{
uint8_t buffType;
uint8_t buffAddition;
} extGetBuff_t;
typedef __packed struct
{
float x;
float y;
float z;
float yaw;
}extGameRobotPos_t;

typedef __packed struct
{
float data1;
float data2;
float data3;
uint8_t mask;
}extShowData_t;

typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;
//Data


typedef enum
{
	ONLINE,
	OFFLINE
}JudgeState_e;

void judgeUartRxCpltCallback(void);
void InitJudgeUart(void);
void Judge_Refresh_Power();
void Judge_Refresh_State();
void Judge_Refresh_Position();
void Judge_Refresh_Shoot();
void Judge_Refresh_Hit();
void Judge_Refresh_Interact();
void Judge_Refresh_Result();
void Judge_Refresh_Buff();
void data_pack_send();

#endif
