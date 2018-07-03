/**
  ******************************************************************************
  * File Name          : peripheral_gpio.h
  * Description        : GPIO外部中断
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 红、绿LED，MPU外部中断相应GPIO口句柄重定义，提高可读性
  ******************************************************************************
  */
#ifndef PERIPHERAL_GPIO_H
#define PERIPHERAL_GPIO_H

//LED PIN
#define GREEN_PIN GPIOF
#define GREEN_GPIO_PORT GPIO_PIN_14
#define RED_PIN GPIOE
#define RED_GPIO_PORT GPIO_PIN_11

//EXIT
#define MPU_INT GPIO_PIN_8 //PE1
#define IST_INT GPIO_PIN_3 //PE3


#endif
