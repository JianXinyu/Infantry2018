/**
  ******************************************************************************
  * File Name          : drivers_uartrc_user.h
  * Description        : 遥控器串口
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 遥控器用户函数
  ******************************************************************************
  */
#ifndef DRIVERS_UARTRC_USER_H
#define DRIVERS_UARTRC_USER_H

#include "utilities_iopool.h"

typedef struct{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote_t;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse_t;	

typedef struct{
	uint16_t v;
}Key_t;

typedef struct{
	Remote_t rc;
	Mouse_t mouse;
	Key_t key;
}RC_CtrlData_t; 


IOPoolDeclare(rcUartIOPool, struct{uint8_t ch[18];});

typedef enum
{
	AUTO = 0,
	MANUL = 1,
}Shoot_Mode_e;

typedef enum
{
	NORMAL = 0,
	EMERGENCY = 1,
}Emergency_Flag;

typedef enum
{
	LOW_s = 0,
	NORMAL_s = 1,
	HIGH_s = 2,
}Move_Speed_e;

typedef enum
{
	OPEN = 0,
	CLOSE = 1,
}Slab_Mode_e;

Shoot_Mode_e GetShootMode(void);
void SetShootMode(Shoot_Mode_e v);
Emergency_Flag GetEmergencyFlag(void);
void SetEmergencyFlag(Emergency_Flag v);
Move_Speed_e GetMoveSpeed(void);
void SetMoveSpeed(Move_Speed_e v);
Slab_Mode_e GetSlabState(void);
void SetSlabState(Slab_Mode_e v);

#endif
