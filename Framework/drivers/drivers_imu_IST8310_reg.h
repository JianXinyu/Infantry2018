/**
  ******************************************************************************
  * File Name          : drivers_imu_IST8310_reg.h
  * Description        : IMU驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * IST8310寄存器地址定义
  ******************************************************************************
  */
//IST8310 REG address
#ifndef __IST8310_REG
#define __IST8310_REG

#define IST8310_ADDRESS           0x0E
#define IST8310_DEVICE_ID_A       0x10

//refer to IST8310 datasheet for more informations
#define IST8310_WHO_AM_I          0x00
#define IST8310_R_CONFA           0x0A
#define IST8310_R_CONFB           0x0B
#define IST8310_R_MODE            0x02

#define IST8310_R_XL              0x03
#define IST8310_R_XM              0x04
#define IST8310_R_YL              0x05
#define IST8310_R_YM              0x06
#define IST8310_R_ZL              0x07
#define IST8310_R_ZM              0x08

#define IST8310_AVGCNTL           0x41
#define IST8310_PDCNTL            0x42

#define IST8310_ODR_MODE          0x01

#endif
