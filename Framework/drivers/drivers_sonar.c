/**
  ******************************************************************************
  * File Name          : drivers_sonar.c
  * Description        : 超声测距传感器驱动
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 超声测距，步兵未用到
  ******************************************************************************
  */
//#include "drivers_sonar_low.h"
//#include "drivers_sonar_user.h"

//#include "stdint.h"
//#include "gpio.h"
//#include "cmsis_os.h"
//#include "utilities_tim.h"
//#include "utilities_debug.h"

//NaiveIOPoolDefine(SonarIOPool, {0});

//float sonar_distance_f = 0.0, sonar_distance_l = 0.0, sonar_distance_b = 0.0, sonar_distance_r = 0.0;
//uint8_t sonar_measuring_flag_f = 0, sonar_measuring_flag_l = 0, sonar_measuring_flag_b = 0, sonar_measuring_flag_r = 0;
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == GPIO_PIN_1)
//	{
//		//fw_printfln("GPIO_Pin == GPIO_PIN_1");
//		static uint64_t initial1;
//		//sonar_echo
//		//jump from low to high
//		if(HAL_GPIO_ReadPin(GPIOF,GPIO_Pin)==GPIO_PIN_SET)
//		{
//			initial1 = fw_getTimeMicros();
//		}
//		//jump from high to low
//		else if(HAL_GPIO_ReadPin(GPIOF,GPIO_Pin)==GPIO_PIN_RESET)
//		{
//			uint32_t tmp= fw_getTimeMicros()-initial1;
//			float tmp_dis=(double)tmp/10000*170;
//			
//			if(tmp_dis<300)
//			{
//				sonar_distance_f=tmp_dis;
//			}
//			//release flag
//			sonar_measuring_flag_f=0;
//		}
//	}
//	
//	if(GPIO_Pin == GPIO_PIN_5)
//	{
//		//fw_printfln("GPIO_Pin == GPIO_PIN_5");
//		static uint64_t initial1;
//		//sonar_echo
//		//jump from low to high
//		if(HAL_GPIO_ReadPin(GPIOE,GPIO_Pin)==GPIO_PIN_SET)
//		{
//			initial1 = fw_getTimeMicros();
//		}
//		//jump from high to low
//		else if(HAL_GPIO_ReadPin(GPIOE,GPIO_Pin)==GPIO_PIN_RESET)
//		{
//			uint32_t tmp= fw_getTimeMicros()-initial1;
//			float tmp_dis=(double)tmp/10000*170;
//			
//			if(tmp_dis<300)
//			{
//				sonar_distance_l=tmp_dis;
//			}
//			//release flag
//			sonar_measuring_flag_l=0;
//		}
//	}
//	
//	if(GPIO_Pin == GPIO_PIN_6)//E6
//	{
//		//fw_printfln("GPIO_Pin == GPIO_PIN_6");
//		static uint64_t initial1;
//		//sonar_echo
//		//jump from low to high
//		if(HAL_GPIO_ReadPin(GPIOE,GPIO_Pin)==GPIO_PIN_SET)
//		{
//			initial1 = fw_getTimeMicros();
//		}
//		//jump from high to low
//		else if(HAL_GPIO_ReadPin(GPIOE,GPIO_Pin)==GPIO_PIN_RESET)
//		{
//			uint32_t tmp= fw_getTimeMicros()-initial1;
//			float tmp_dis=(double)tmp/10000*170;
//			
//			if(tmp_dis<300)
//			{
//				sonar_distance_b=tmp_dis;
//			}
//			//release flag
//			sonar_measuring_flag_b=0;
//		}
//	}
//	
//	if(GPIO_Pin == GPIO_PIN_10)//GPIO_PIN_2
//	{
//		//fw_printfln("GPIO_Pin == GPIO_PIN_2");//c2
//		static uint64_t initial1;
//		//sonar_echo
//		//jump from low to high
//		if(HAL_GPIO_ReadPin(GPIOF,GPIO_Pin)==GPIO_PIN_SET)
//		{
//			initial1 = fw_getTimeMicros();
//		}
//		//jump from high to low
//		else if(HAL_GPIO_ReadPin(GPIOF,GPIO_Pin)==GPIO_PIN_RESET)
//		{
//			uint32_t tmp= fw_getTimeMicros()-initial1;
//			float tmp_dis=(double)tmp/10000*170;
//			
//			if(tmp_dis<300)
//			{
//				sonar_distance_r=tmp_dis;
//			}
//			//release flag
//			sonar_measuring_flag_r=0;
//		}
//	}
//}


//void sonarTask(void const * argument){
//	while(1){
//		if(sonar_measuring_flag_f==0){
//				//ready for next measure
//				uint32_t i=42*15;
//				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
//				while(i--);
//				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
//				sonar_measuring_flag_f=1;
//		}
//		
//		osDelay(2);
//		
//		if(sonar_measuring_flag_l==0){
//				//ready for next measure
//				uint32_t i=42*15;
//				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
//				while(i--);
//				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
//				sonar_measuring_flag_l=1;
//		}
//		//fw_printf("l = %3f\t", sonar_distance_l);
//		osDelay(2);
//		
//		if(sonar_measuring_flag_b==0){//e12
//				//ready for next measure
//				uint32_t i=42*15;
//				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);
//				while(i--);
//				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);
//				sonar_measuring_flag_b=1;
//		}
//		//fw_printf("b = %3f\t", sonar_distance_b);
//		osDelay(2);
//	
//		if(sonar_measuring_flag_r==0){//b8
//				//ready for next measure
//				uint32_t i=42*15;
//				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_9,GPIO_PIN_SET);
//				while(i--);
//				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_9,GPIO_PIN_RESET);
//				sonar_measuring_flag_r=1;
//		}
//		//fw_printfln("r = %3f\t", sonar_distance_r);
//		
//		IOPool_pGetWriteData(SonarIOPool)->sonarValue[0] = sonar_distance_f;
//		IOPool_pGetWriteData(SonarIOPool)->sonarValue[1] = sonar_distance_l;
//		IOPool_pGetWriteData(SonarIOPool)->sonarValue[2] = sonar_distance_b;
//		IOPool_pGetWriteData(SonarIOPool)->sonarValue[3] = sonar_distance_r;
//		IOPool_getNextWrite(SonarIOPool);
//		osDelay(2);
//		//fw_printf("f = %3f\t", sonar_distance_f);
//	}
//}


