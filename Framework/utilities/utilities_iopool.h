/**
  ******************************************************************************
  * File Name          : utilities_iopool.h
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
#ifndef UTILITIES_IOPOOL_H
#define UTILITIES_IOPOOL_H

#include <stdint.h>


typedef uint16_t Id_t;
typedef uint8_t DataIndex_t;
typedef uint8_t ReadPoolIndex_t;
typedef enum{Empty, NextRead, Locked} ExchangeStatus_t;

typedef struct{
	DataIndex_t forRead;
	ExchangeStatus_t exchangeStatus;
	DataIndex_t forExchange;
}ReadPool_t;

#define DataTypeDef(ioPool, dataType) typedef dataType ioPool##_Data_t

#define IOPoolDef(ioPool) \
struct{ \
	uint8_t readPoolSize; \
	Id_t *readPoolMap; \
	ioPool##_Data_t *data; \
	ReadPool_t *readPool; \
	DataIndex_t forWrite; \
}

#define getIdDec(ioPool) \
Id_t ioPool##_getId(ioPool##_Data_t data)

#define getIdDef(ioPool, function) \
Id_t ioPool##_getId(ioPool##_Data_t data){ \
		return function; \
}

ReadPoolIndex_t getReadPoolIndexPrototype(Id_t id, uint8_t readPoolSize, const Id_t* const readPoolMap);
#define getReadPoolIndex(ioPool, id) \
	getReadPoolIndexPrototype(id, ioPool.readPoolSize, ioPool.readPoolMap)


#define IOPool_hasNextRead(ioPool, id) \
	(ioPool.readPool[getReadPoolIndex(ioPool, id)].exchangeStatus == NextRead)

#define IOPool_getNextRead(ioPool, id) { \
	ReadPool_t *readPool = ioPool.readPool + getReadPoolIndex(ioPool, id); \
	if(readPool->exchangeStatus == NextRead){ \
		readPool->exchangeStatus = Locked; \
		DataIndex_t temp = readPool->forRead; \
		readPool->forRead = readPool->forExchange; \
		readPool->forExchange = temp; \
		readPool->exchangeStatus = Empty; \
	} \
}

#define IOPool_pGetReadData(ioPool, id) \
	(ioPool.data + ioPool.readPool[getReadPoolIndex(ioPool, id)].forRead)

#define IOPool_getNextWrite(ioPool) { \
	ReadPool_t *readPool = ioPool.readPool \
		+ getReadPoolIndex(ioPool, ioPool##_getId(ioPool.data[ioPool.forWrite])); \
	if(readPool->exchangeStatus != Locked){ \
		readPool->exchangeStatus = Locked; \
		DataIndex_t temp = ioPool.forWrite; \
		ioPool.forWrite = readPool->forExchange; \
		readPool->forExchange = temp; \
		readPool->exchangeStatus = NextRead; \
	} \
}

#define IOPool_pGetWriteData(ioPool) \
	(ioPool.data + ioPool.forWrite)


#define IOPoolDeclare( \
	IOPoolName, \
	DataType \
) \
DataTypeDef(IOPoolName, DataType); \
typedef IOPoolDef(IOPoolName) IOPoolName##_t; \
getIdDec(IOPoolName); \
extern IOPoolName##_t IOPoolName;

#define NaiveIOPoolDefine( \
	IOPoolName, \
	DataPoolInit \
) \
Id_t IOPoolName##_readPoolMap[] = {0}; \
IOPoolName##_Data_t IOPoolName##_data[1 * 2 + 1] = DataPoolInit; \
ReadPool_t IOPoolName##_readPool[1] = {0, Empty, 1}; \
IOPoolName##_t IOPoolName = { \
	1, \
	IOPoolName##_readPoolMap, \
	IOPoolName##_data, \
	IOPoolName##_readPool, \
	1 * 2 \
}; \
getIdDef(IOPoolName, 0);

//#define IOPoolDefine( \
//	IOPoolName, \
//	DataPoolInit, \
//	ReadPoolSize, \
//	ReadPoolMap, \
//	GetIdFunc, \
//	ReadPoolInit \
//) \
//Id_t IOPoolName##_readPoolMap[] = ReadPoolMap; \
//IOPoolName##_Data_t IOPoolName##_data[ReadPoolSize * 2 + 1] = DataPoolInit; \
//ReadPool_t IOPoolName##_readPool[ReadPoolSize] = ReadPoolInit; \
//IOPoolName##_t IOPoolName = { \
//	ReadPoolSize, \
//	IOPoolName##_readPoolMap, \
//	IOPoolName##_data, \
//	IOPoolName##_readPool, \
//	ReadPoolSize * 2 \
//}; \
//getIdDef(IOPoolName, GetIdFunc);

#endif
