/**
  ******************************************************************************
  * File Name          : drivers_buzzer_low.h
  * Description        : 蜂鸣器驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 蜂鸣器底层函数
  ******************************************************************************
  */
#ifndef DRIVERS_BUZZER_LOW_H
#define DRIVERS_BUZZER_LOW_H

void playMusicWhenInit(void);
void buzzerTask(void const * argument);

#endif
