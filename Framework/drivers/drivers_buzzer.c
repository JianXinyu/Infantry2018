/**
  ******************************************************************************
  * File Name          : drivers_buzzer.c
  * Description        : 蜂鸣器驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 不同的PWM波占空比对用不同音高，按表格依次改变频率可获得音乐
  ******************************************************************************
  */
#include <cmsis_os.h>
#include <tim.h>	
#include "drivers_buzzer_low.h"
#include "drivers_buzzer_user.h"
#include "drivers_buzzer_notes.h"
#include "drivers_buzzer_music.h"
#include "peripheral_define.h"


void myDelay(uint16_t time){
	for(int i=0; i < time; i++)
	{
		int a = 45000; //at 168MHz 42000 is ok
		while(a--);
	}
}


#define STOP(time) __HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, 0);osDelay(time);

#define PLAY(note, time) \
if(note == 0){ \
	__HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, 0);osDelay(time); \
}else{ \
	__HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, 1000000 / note); \
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500000 / note); \
	osDelay(time); \
}

#define PLAYWHENINIT(note, time) \
if(note == 0){ \
	__HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, 0);myDelay(time); \
}else{ \
	__HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, 1000000 / note); \
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500000 / note); \
	myDelay(time); \
}

#define SET_BUZZER_FREQUENCY(f) \
__HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, 1000000 / f); \
__HAL_TIM_SET_COMPARE(&BUZZER_TIM, TIM_CHANNEL_1, 500000 / f);


void playMusicWhenInit(void){
	HAL_TIM_PWM_Start(&BUZZER_TIM, TIM_CHANNEL_1);
	for(int i = 0; i < sizeof(SuperMario) / sizeof(MusicNote); i++){
			PLAYWHENINIT(SuperMario[i].note, SuperMario[i].time);
	}
	HAL_TIM_PWM_Stop(&BUZZER_TIM, TIM_CHANNEL_1);
	
//	HAL_TIM_PWM_Start(&BUZZER_TIM, TIM_CHANNEL_1);
//	for(int i = 0; i < sizeof(Intel) / sizeof(MusicNote); i++){
//			PLAYWHENINIT(Intel[i].note, Intel[i].time);
//	}
//	HAL_TIM_PWM_Stop(&BUZZER_TIM, TIM_CHANNEL_1);
	
}

void playMusicSuperMario(void){
	HAL_TIM_PWM_Start(&BUZZER_TIM, TIM_CHANNEL_1);
	for(int i = 0; i < sizeof(SuperMario) / sizeof(MusicNote); i++){
			PLAY(SuperMario[i].note, SuperMario[i].time);
	}
	HAL_TIM_PWM_Stop(&BUZZER_TIM, TIM_CHANNEL_1);
}

void buzzerTask(void const * argument){
	//HAL_TIM_PWM_Start(&BUZZER_TIM, TIM_CHANNEL_1);
	while(1){
//		for(int i = 0; i < sizeof(SuperMario) / sizeof(MusicNote); i++){
//			PLAY(SuperMario[i].note, SuperMario[i].time);
//		}
//		STOP(1000);
		
//		for(int i = 0; i != 12; i++){
//			PLAY(notes_low[i], 500);
//		}
//		STOP(1000);
//		for(int i = 0; i != 12; i++){
//			PLAY(notes_mid[i], 500);
//		}
//		STOP(1000);
//		for(int i = 0; i != 12; i++){
//			PLAY(notes_high[i], 500);
//		}
//		STOP(1000);
//		SET_BUZZER_FREQUENCY(523);
//		osDelay(1000);
//		SET_BUZZER_FREQUENCY(587);
//		osDelay(1000);
//		SET_BUZZER_FREQUENCY(659);
//		osDelay(1000);
//		SET_BUZZER_FREQUENCY(698);
//		osDelay(1000);
//		SET_BUZZER_FREQUENCY(784);
//		osDelay(1000);
//		SET_BUZZER_FREQUENCY(880);
//		osDelay(1000);
//		SET_BUZZER_FREQUENCY(988);
//		osDelay(1000);
//		SET_BUZZER_FREQUENCY(1046);
//		osDelay(1000);
	}
}
