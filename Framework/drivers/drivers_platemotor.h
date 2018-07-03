/**
  ******************************************************************************
  * File Name          : drivers_platemotor.h
  * Description        : 拨盘电机驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 拨盘电机速度/位置控制
	* 卡弹检测及处理
  ******************************************************************************
  */
#ifndef DRIVERS_PLATEMOTOR_H
#define DRIVERS_PLATEMOTOR_H


typedef enum{REVERSE, FORWARD,}RotateDir_e;
void plateMotorInit(void);
void setPlateMotorDir(RotateDir_e dir);
RotateDir_e getPlateMotorDir(void);


#endif
