/**
  ******************************************************************************
  * File Name          : pwm_server_motor.c
  * Description        : 舵机驱动
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * PWM占空比对应舵机角度
  ******************************************************************************
  */
#include "pwm_server_motor.h"

void pwm_server_motor_init(uint8_t motorIndex)
{
	switch (motorIndex)
	{
		case 0:
		{
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,1500);
			break;
		}
		case 1:
		{
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,1500);
			break;
		}
		case 2:
		{
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,1500);
			break;
		}
		case 3:
		{
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,1500);
			break;
		}
		case 4:
		{
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1500);
			break;
		}
		case 5:
		{
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1500);
			break;
		}
		case 6:
		{
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,1500);
			break;
		}
		case 7:
		{
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,1500);
			break;
		}
		case 8:
		{
			HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,1500);
			break;
		}
		case 9:
		{
			HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,1500);
			break;
		}
		case 10:
		{
			HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,1500);
			break;
		}
		case 11:
		{
			HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,1500);
			break;
		}
		case 12:
		{
			HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,1500);
			break;
		}
		case 13:
		{
			HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,1500);
			break;
		}
		case 14:
		{
			HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,1500);
			break;
		}
		case 15:
		{
			HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,1500);
			break;
		}
	}
}

void pwm_server_motor_set_angle(uint8_t motorIndex,float angle)
{
	if(angle<0)
	{
		angle=0;
	}
	if(angle>180)
	{
		angle=180;
	}
	
	uint16_t x=angle/180*1900+500;
	
	switch(motorIndex)
	{
		case 0:
		{
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,x);
			break;
		}
		case 1:
		{
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,x);
			break;
		}
		case 2:
		{
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,x);
			break;
		}
		case 3:
		{
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,x);
			break;
		}
		case 4:
		{
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,x);
			break;
		}
		case 5:
		{
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,x);
			break;
		}
		case 6:
		{
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,x);
			break;
		}
		case 7:
		{
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,x);
			break;
		}
		case 8:
		{
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,x);
			break;
		}
		case 9:
		{
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,x);
			break;
		}
		case 10:
		{
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,x);
			break;
		}
		case 11:
		{
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,x);
			break;
		}
		case 12:
		{
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,x);
			break;
		}
		case 13:
		{
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,x);
			break;
		}
		case 14:
		{
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,x);
			break;
		}
		case 15:
		{
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,x);
			break;
		}
	}
}
