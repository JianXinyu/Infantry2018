/**
  ******************************************************************************
  * File Name          : drivers_cmpower.h
  * Description        : 底盘功率限制
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 底层函数
  ******************************************************************************
  */
#ifndef DRIVERS_CMPOWER_H
#define DRIVERS_CMPOWER_H

#define    CM_current_bottom      3000.f
#define 	 CMFLIntensity_bottom   1000.f
#define    CMFRIntensity_bottom   1000.f
#define  	 CMBLIntensity_bottom   1000.f
#define	   CMBRIntensity_bottom   1000.f

#define    CM_current_low      12000.f
#define 	 CMFLIntensity_low   3000.f
#define    CMFRIntensity_low   3000.f
#define  	 CMBLIntensity_low   3000.f
#define	   CMBRIntensity_low   3000.f

#define    CM_current_lower      14000.f
#define 	 CMFLIntensity_lower   4500.f
#define    CMFRIntensity_lower   4500.f
#define  	 CMBLIntensity_lower   4500.f
#define	   CMBRIntensity_lower   4500.f

#define    CM_current_MAX      18000.f
#define    CMFLIntensity_MAX   5900.f
#define    CMFRIntensity_MAX   5900.f
#define    CMBLIntensity_MAX   5900.f
#define    CMBRIntensity_MAX   5900.f

void RestrictPower(int16_t *intensity1, int16_t *intensity2, int16_t *intensity3, int16_t *intensity4);
void dynamicUpperBound(void);
#endif
