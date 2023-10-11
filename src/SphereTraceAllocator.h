#pragma once
#include <stdlib.h>
#include "SphereTraceLists.h"

typedef uintptr_t ST_Index;

typedef struct ST_FreeStack
{
	ST_Index* pStack;
	ST_Index head;
	ST_Index capacity;
} ST_FreeStack;

ST_FreeStack sphereTraceAllocatorFreeStackConstruct(ST_Index capacity, void* pArr, ST_Index objectSize);
ST_Index* sphereTraceAllocatorFreeStackPurchase(ST_FreeStack* const pFreeStack);
void sphereTraceAllocatorFreeStackSell(ST_FreeStack* const pFreeStack, ST_Index* pIndex);


typedef struct ST_ObjectPool
{
	void* data;
	ST_FreeStack freeStack;
	ST_Index occupied;
	ST_Index capacity;
	ST_Index objectSize;
} ST_ObjectPool;

ST_ObjectPool sphereTraceAllocatorObjectPoolConstruct(ST_Index capacity, ST_Index objectSize);
void* sphereTraceAllocatorObjectPoolAllocateObject(ST_ObjectPool* const pObjectPool);
void sphereTraceAllocatorObjectPoolFreeObject(ST_ObjectPool* const pObjectPool, void* pObject);

typedef struct ST_Allocator
{
	ST_ObjectPool** objectPools;
	ST_Index numObjects;
	ST_Index curPool;
	ST_Index maxPools;
} ST_Allocator;

ST_Allocator sphereTraceAllocatorConstruct(ST_Index poolCapacity, ST_Index objectSize, ST_Index maxPools);
void* sphereTraceAllocatorAllocateObject(ST_Allocator* const pAllocator);
void sphereTraceAllocatorFreeObject(ST_Allocator* const pAllocator, void* pObject);

void sphereTraceAllocatorInitialize();
ST_IndexListData* sphereTraceAllocatorAllocateIndexListData();
void sphereTraceAllocatorFreeIndexListData(void* pIndex);
ST_Vector3ListData* sphereTraceAllocatorAllocateVector3ListData();
void sphereTraceAllocatorFreeVector3ListData(void* pIndex);
ST_Vector3* sphereTraceAllocatorAllocateVector3();
void sphereTraceAllocatorFreeVector3(void* pIndex);