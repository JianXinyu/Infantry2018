/**
  ******************************************************************************
  * File Name          : drivers_led_user.h
  * Description        : LED闪烁任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * user用户函数声明
  ******************************************************************************
  */
#ifndef DRIVERS_LED_USER_H
#define DRIVERS_LED_USER_H
//#define OFFBOARD

typedef enum{off, on, blink} LedStatus_t;
extern LedStatus_t ledGreenStatus, ledRedStatus;

#endif
