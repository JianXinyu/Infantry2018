/**
  ******************************************************************************
  * File Name          : drivers_led_low.h
  * Description        : LED闪烁任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * low底层函数声明
  ******************************************************************************
  */
#ifndef DRIVERS_LED_LOW_H
#define DRIVERS_LED_LOW_H

void ledGreenTask(void const * argument);
void ledRedTask(void const * argument);

#endif
