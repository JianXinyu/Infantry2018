/**
  ******************************************************************************
  * File Name          : utilities_iopool.c
  * Description        : IOPOOL
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * IOPOOL 
	* 注释存在于骆庭晟队长脑海中
	* 有志者可以自己读
	*
	* 粗浅解释如下
	* IOPOOL是一种线程安全的进程间通讯方式，效率高于队列（骆队长如是说）
	* 一个IOPOOL有三层，ReadPool, WritePool 和 ExchangePool
	* 通过对层间交换的管理实现线程安全
	*
	* 定义方法：
	* IOPoolDeclare(IOPOOL name，数据类型)在头文件中
	* NaiveIOPoolDefine(IOPOOL name, 初始化)在.c中
	*
	* 使用方法：
	* 线程1使用IOPool_getNextWrite(ioPool)获得指向WritePool的指针并修改WritePool
  * 线程1修改WritePool完成后调用IOPool_getNextWrite(ioPool)，将WritePool数据复制到ExchangePool
	* 线程2使用IOPool_hasNextRead(ioPool, id)查看ExchangePool是否有数据
	* 若检测到有数据，线程2使用IOPool_hasNextRead(ioPool, id)将ExchangePool数据复制到ReadPool
	* 最后，线程1使用IOPool_pGetReadData(ioPool, id)获得指向ReadPool的指针并读取数据
	*
	* 细节请查看具体使用
  ******************************************************************************
  */
#include "utilities_iopool.h"
#include "utilities_debug.h"

ReadPoolIndex_t getReadPoolIndexPrototype(Id_t id, uint8_t readPoolSize, const Id_t* const readPoolMap){
	ReadPoolIndex_t i;
	for(i = 0; i != readPoolSize; ++i){
		if(readPoolMap[i] == id){
			return i;
		}
	}
	fw_Error_Handler();
	return i;
}
