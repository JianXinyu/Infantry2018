/**
  ******************************************************************************
  * File Name          : rtos_semaphore.c
  * Description        : 添加FreeRTOS信号量
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 添加信号量用于进程间同步

  ******************************************************************************
  */
#include <rtos_semaphore.h>
#include <rtos_init.h>

osSemaphoreId CMGMCanHaveTransmitSemaphoreHandle;
osSemaphoreId ZGYROCanHaveTransmitSemaphoreHandle;

osSemaphoreId CMGMCanTransmitSemaphoreHandle;
osSemaphoreId ZGYROCanTransmitSemaphoreHandle;

osSemaphoreId motorCanReceiveSemaphoreHandle;

osSemaphoreId CMGMCanRefreshSemaphoreHandle;
osSemaphoreId ZGYROCanRefreshSemaphoreHandle;

osSemaphoreId imurefreshGimbalSemaphoreHandle;

//osSemaphoreId imuSpiTxRxCpltSemaphoreHandle;
osSemaphoreId refreshMPU6500SemaphoreHandle;

xSemaphoreHandle xSemaphore_mfuart;
xSemaphoreHandle xSemaphore_rcuart;
xSemaphoreHandle motorCanTransmitSemaphore;
void rtos_AddSemaphores()
{
	osSemaphoreDef(CMGMCanTransmitSemaphore);//电机CAN发送信号量
	CMGMCanTransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(CMGMCanTransmitSemaphore), 1);
	osSemaphoreDef(ZGYROCanTransmitSemaphore);//外接单轴陀螺仪CAN发送信号量
	ZGYROCanTransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(ZGYROCanTransmitSemaphore), 1);

	osSemaphoreDef(CMGMCanRefreshSemaphore);//电机CAN接收信号量
	CMGMCanRefreshSemaphoreHandle = osSemaphoreCreate(osSemaphore(CMGMCanRefreshSemaphore), 1);
	osSemaphoreDef(ZGYROCanRefreshSemaphore);//外接单轴陀螺仪CAN接收信号量(有Relase，无进程Take)
	ZGYROCanRefreshSemaphoreHandle = osSemaphoreCreate(osSemaphore(ZGYROCanRefreshSemaphore), 1);
	
	osSemaphoreDef(imurefreshGimbalSemaphore);//IMU数据刷新信号量(有Release，无进程Take)
	imurefreshGimbalSemaphoreHandle = osSemaphoreCreate(osSemaphore(imurefreshGimbalSemaphore), 1);
	
//	osSemaphoreDef(imuSpiTxRxCpltSemaphore);
//	imuSpiTxRxCpltSemaphoreHandle = osSemaphoreCreate(osSemaphore(imuSpiTxRxCpltSemaphore), 1);
	osSemaphoreDef(refreshMPU6500Semaphore);//MPU6050数据刷新信号量：IO口外部中断Release，数据处理Task Take
	refreshMPU6500SemaphoreHandle = osSemaphoreCreate(osSemaphore(refreshMPU6500Semaphore), 1);
/**
******************************************************************************
* 提示：
* 这里示范了两种创建信号量的方式
* osSemaphoreDef(),osSemaphoreCreate()是CMSIS要求的RTOS统一接口
* vSemaphoreCreateBinary()是FreeRTOS提供的创建方式
* 两种方式本质相同，osSemaphoreCreate()实际上调用了xSemaphoreCreateBinary()
******************************************************************************
*/
	vSemaphoreCreateBinary(xSemaphore_mfuart);//mf(Manifold妙算)通信信号量，串口接收回掉函数Release，数据处理Task Take
	vSemaphoreCreateBinary(xSemaphore_rcuart);//rc(Remote Control遥控)同上
	
	motorCanTransmitSemaphore = xSemaphoreCreateCounting(10,0);//另一种信号量，未用 注意开启此功能需要在Cube中配置RTOS
}
