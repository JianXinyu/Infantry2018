/**
  ******************************************************************************
  * File Name          : drivers_imu_low.h
  * Description        : IMU驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * IMU底层函数声明
  ******************************************************************************
  */
#ifndef DRIVERS_IMU_LOW_H
#define DRIVERS_IMU_LOW_H

#include "stdint.h"
typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  int16_t mx;
  int16_t my;
  int16_t mz;
}IMUDataTypedef;

extern uint8_t MPU_id;

uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data);
uint8_t MPU6500_Read_Reg(uint8_t const reg);
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len);
void IMU_Get_Data(void);

uint8_t InitMPU6500(void);
uint8_t InitIST8310(void);

void IMUTask(void const * argument);

void updateQuaternion(void);

#endif
