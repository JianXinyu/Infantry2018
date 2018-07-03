/**
  ******************************************************************************
  * File Name          : drivers_cmpower.c
  * Description        : 底盘功率限制
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 底层函数
  ******************************************************************************
  */
#include "drivers_uartjudge_low.h"
#include "drivers_cmpower.h"
#include "utilities_minmax.h"
#include "utilities_debug.h"
#include <math.h>
#include <stdlib.h>

extern extPowerHeatData_t PowerHeatData;
extern float realPowerBuffer;
extern uint8_t JUDGE_State;
extern uint8_t going;

static float CM_current_max = CM_current_MAX;
static float CMFLIntensity_max = CMFLIntensity_MAX;
static float CMFRIntensity_max = CMFRIntensity_MAX;
static float CMBLIntensity_max = CMBLIntensity_MAX;
static float CMBRIntensity_max = CMBRIntensity_MAX;

void RestrictPower(int16_t *intensity1, int16_t *intensity2, int16_t *intensity3, int16_t *intensity4)
{
	//根据能量槽剩余做动态上限
	dynamicUpperBound();

	fw_printfln("remainPower: %f \r\n",PowerHeatData.chassisPowerBuffer);
	fw_printfln("max%f",CM_current_max);
	
	int16_t *CMFLIntensity = intensity1;
	int16_t *CMFRIntensity = intensity2;
	int16_t *CMBLIntensity = intensity3;
	int16_t *CMBRIntensity = intensity4;
	
	float sum = (abs(*CMFLIntensity) + abs(*CMFRIntensity) + abs(*CMBLIntensity) + abs(*CMBRIntensity));
	
	//单独限制 + 总和比例限制
	MINMAX(*CMFLIntensity, -CMFLIntensity_max, CMFLIntensity_max);
	MINMAX(*CMFRIntensity, -CMFRIntensity_max, CMFRIntensity_max);
	MINMAX(*CMBLIntensity, -CMBLIntensity_max, CMBLIntensity_max);
	MINMAX(*CMBRIntensity, -CMBRIntensity_max, CMBRIntensity_max);

	if(sum > CM_current_max)
	{
		*CMFLIntensity = (CM_current_max/sum) * (*CMFLIntensity);
		*CMFRIntensity = (CM_current_max/sum) * (*CMFRIntensity);
		*CMBLIntensity = (CM_current_max/sum) * (*CMBLIntensity);
		*CMBRIntensity = (CM_current_max/sum) * (*CMBRIntensity);
	}
}	

void dynamicUpperBound()
{
	if (realPowerBuffer > 40)
	{
		 CM_current_max = CM_current_MAX;
		 CMFLIntensity_max = CMFLIntensity_MAX;
		 CMFRIntensity_max = CMFRIntensity_MAX;
		 CMBLIntensity_max = CMBLIntensity_MAX;
		 CMBRIntensity_max = CMBRIntensity_MAX;
	}
	
	else if (realPowerBuffer > 15)
	{
		 CM_current_max = CM_current_lower;
		 CMFLIntensity_max = CMFLIntensity_lower;
		 CMFRIntensity_max = CMFRIntensity_lower;
		 CMBLIntensity_max = CMBLIntensity_lower;
		 CMBRIntensity_max = CMBRIntensity_lower;
	}

	else if (realPowerBuffer >= 7 )
	{
		 CM_current_max = CM_current_bottom;
		 CMFLIntensity_max = CMFLIntensity_bottom;
		 CMFRIntensity_max = CMFRIntensity_bottom;
		 CMBLIntensity_max = CMBLIntensity_bottom;
		 CMBRIntensity_max = CMBRIntensity_bottom;
	}
	else if (realPowerBuffer < 7)
	{
		 CM_current_max = 0;
		 CMFLIntensity_max = 0;
		 CMFRIntensity_max = 0;
		 CMBLIntensity_max = 0;
		 CMBRIntensity_max = 0;
	}

	if (JUDGE_State == OFFLINE)
	{
		 CM_current_max = 13000;
		 CMFLIntensity_max = 4500;
		 CMFRIntensity_max = 4500;
		 CMBLIntensity_max = 4500;
		 CMBRIntensity_max = 4500;
	}
	if (going)
	{
		 CM_current_max = 25000;
		 CMFLIntensity_max = 7000;
		 CMFRIntensity_max = 7000;
		 CMBLIntensity_max = 7000;
		 CMBRIntensity_max = 7000;
	}
//	fw_printfln("max%f",CM_current_max);
}
