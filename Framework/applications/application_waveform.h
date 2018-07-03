/**
  ******************************************************************************
  * File Name          : application_waveform.h
  * Description        : 波形观察
  **********************debug by ZY*********************************************
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 使用匿名四轴地面站做上位机观察电机波形
	* 按地面站的串口协议发送
	*
  ******************************************************************************
  */
#ifndef APPLICATION_WAVEFOR_H
#define APPLICATION_WAVEFOR_H

void send_data_to_PC(UART_HandleTypeDef *huart,float zyPitch,float zyYaw,float zySpd);
#endif

