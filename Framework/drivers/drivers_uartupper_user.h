/**
  ******************************************************************************
  * File Name          : drivers_uartupper_low.h
  * Description        : å¦™ç®—é€šä¿¡
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * å¤§ç¥ç¬¦ã€è‡ªç„ç›¸å…³å‡½æ•°
  ******************************************************************************
  */
#ifndef DRIVERS_UARTUPPER_USER_H
#define DRIVERS_UARTUPPER_USER_H

#include "utilities_iopool.h"
#define size_frame 14  //¶¨ÒåÃîËã´®¿ÚÖ¡³¤¶È/×Ö½Ú
#define byte_SOF 0x7d    //ÆğÊ¼×Ö½Ú	
#define byte_EOF 0x7e    //ÖÕÖ¹×Ö½Ú
#define byte_ESCAPE 0xff //×ªÒå×Ö½Ú

typedef struct{
	uint16_t dev_yaw;
	uint16_t dev_pitch;
	uint8_t rune_locate;	
	uint8_t rune;
	uint16_t target_dis;
	uint8_t DLC;
	uint8_t Success;
}xdata_ctrlUart;

typedef enum{
	Locating = 0,
	Located = 1,
}Locate_State_e;
void SetLocateState(Locate_State_e v);
Locate_State_e GetLocateState(void);

typedef enum{
	WAITING = 0,
	BIGRUNE = 1,
	SMALLRUNE = 2,
}Rune_State_e;
void SetRuneState(Rune_State_e v);
Rune_State_e GetRuneState(void);

typedef struct{
	float yaw_position;
	float pitch_position;
}Location_Number_s;

void ShootRune(uint8_t location);
void vRefreshLocation(float yaw_center, float pitch_center);
	
IOPoolDeclare(ctrlUartIOPool, struct{uint8_t ch[size_frame];});

void manifoldUartRxCpltCallback(void);

//void zyLocationInit(float yaw_center, float pitch_center);
void zyLocationInit(Location_Number_s * Rune3Position);//å¼ é›å¤§ç¬¦åˆå§‹

#endif
