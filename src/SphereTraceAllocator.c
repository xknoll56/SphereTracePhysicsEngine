#pragma once

#include "SphereTraceAllocator.h"
#include "SphereTraceLists.h"
#include "SphereTraceCollider.h"



#define ST_INDEX_POOL_DEFAULT_SIZE 1000
#define ST_FUNCTION_POOL_DEFAULT_SIZE 20
#define ST_INDEX_ALLOCATOR_DEFAULT_SIZE 5

static ST_Allocator indexListAllocator;
static ST_Allocator vector3ListAllocator;
static ST_Allocator vector3Allocator;
static ST_Allocator callbackFunctionAllocator;
static ST_Allocator contactEntryAllocator;

ST_FreeStack sphereTraceAllocatorFreeStackConstruct(ST_Index capacity, void* pArr, ST_Index objectSize)
{
	ST_FreeStack stack;
	stack.pStack = (ST_Index*)malloc(capacity * sizeof(ST_Index));
	for (ST_Index i = 0; i < capacity; i++)
	{
		stack.pStack[i] = (ST_Index)pArr+i*objectSize;
	}
	stack.head = 0;
	stack.capacity = capacity;
	return stack;
}

ST_Index* sphereTraceAllocatorFreeStackPurchase(ST_FreeStack* const pFreeStack)
{
	if (pFreeStack->head < pFreeStack->capacity)
	{
		ST_Index* freeIndex = pFreeStack->pStack[pFreeStack->head];
		pFreeStack->head++;
		return freeIndex;
	}
}


void sphereTraceAllocatorFreeStackSell(ST_FreeStack* const pFreeStack, ST_Index* pIndex)
{
	if (pFreeStack->head > 0)
	{
		pFreeStack->head--;
		pFreeStack->pStack[pFreeStack->head] = pIndex;
	}
}


ST_ObjectPool sphereTraceAllocatorObjectPoolConstruct(ST_Index capacity, ST_Index objectSize)
{
	ST_ObjectPool objectPool;
	objectPool.objectSize = objectSize;
	objectPool.data = malloc(capacity * objectSize);
	objectPool.freeStack = sphereTraceAllocatorFreeStackConstruct(capacity, objectPool.data, objectSize);
	objectPool.capacity = capacity;
	objectPool.occupied = 0;
	return objectPool;
}

void* sphereTraceAllocatorObjectPoolAllocateObject(ST_ObjectPool* const pObjectPool)
{
	if (pObjectPool->occupied < pObjectPool->capacity)
	{
		ST_Index* pIndex = sphereTraceAllocatorFreeStackPurchase(&pObjectPool->freeStack);
		pObjectPool->occupied++;
		return pIndex;
	}
	else
		return NULL;
}

void sphereTraceAllocatorObjectPoolFreeObject(ST_ObjectPool* const pObjectPool, void* pObject)
{
	if (pObjectPool->occupied > 0)
	{
		sphereTraceAllocatorFreeStackSell(&pObjectPool->freeStack, pObject);
		pObjectPool->occupied--;
	}
}

ST_Allocator sphereTraceAllocatorConstruct(ST_Index poolCapacity, ST_Index objectSize, ST_Index maxPools)
{
	ST_Allocator allocator;
	allocator.curPool = 0;
	allocator.numObjects = 0;
	allocator.maxPools = maxPools;
	allocator.objectPools = (ST_ObjectPool**)malloc(allocator.maxPools * sizeof(ST_ObjectPool*));
	for (int i = 0; i < maxPools; i++)
	{
		allocator.objectPools[i] = (ST_ObjectPool*)malloc(sizeof(ST_ObjectPool));
		*allocator.objectPools[i] = sphereTraceAllocatorObjectPoolConstruct(poolCapacity, objectSize);
	}
	return allocator;
}

void* sphereTraceAllocatorAllocateObject(ST_Allocator* const pAllocator)
{
	pAllocator->numObjects++;
	if (pAllocator->numObjects % pAllocator->objectPools[0]->capacity == 0)
		pAllocator->curPool++;
	if (pAllocator->curPool >= pAllocator->maxPools)
	{
		ST_Index newMax = pAllocator->maxPools * 2;
		void* dataPointers = (ST_ObjectPool**)realloc(pAllocator->objectPools, newMax * sizeof(ST_ObjectPool*));
		if (dataPointers)
		{
			pAllocator->objectPools = dataPointers;
			for (int i = pAllocator->maxPools; i < newMax; i++)
			{
				pAllocator->objectPools[i] = (ST_ObjectPool*)malloc(sizeof(ST_ObjectPool));
				*pAllocator->objectPools[i] = sphereTraceAllocatorObjectPoolConstruct(pAllocator->objectPools[0]->capacity,
					pAllocator->objectPools[0]->objectSize);
			}
			pAllocator->maxPools = newMax;
		}
		else
		{
			return NULL;
		}
	}
	return sphereTraceAllocatorObjectPoolAllocateObject(pAllocator->objectPools[pAllocator->curPool]);
}

void sphereTraceAllocatorFreeObject(ST_Allocator* const pAllocator, void* pObject)
{
	if (pAllocator->numObjects > 0)
	{
		pAllocator->numObjects--;
		sphereTraceAllocatorObjectPoolFreeObject(pAllocator->objectPools[pAllocator->curPool], pObject);
		if (pAllocator->objectPools[pAllocator->curPool]->occupied == 0 && pAllocator->curPool>0)
			pAllocator->curPool--;
	}
}


void sphereTraceAllocatorInitialize()
{
	indexListAllocator = sphereTraceAllocatorConstruct(ST_INDEX_POOL_DEFAULT_SIZE, sizeof(ST_IndexListData), ST_INDEX_ALLOCATOR_DEFAULT_SIZE);
	vector3ListAllocator = sphereTraceAllocatorConstruct(ST_INDEX_POOL_DEFAULT_SIZE, sizeof(ST_Vector3ListData), ST_INDEX_ALLOCATOR_DEFAULT_SIZE);
	vector3Allocator = sphereTraceAllocatorConstruct(ST_INDEX_POOL_DEFAULT_SIZE, sizeof(ST_Vector3), ST_INDEX_ALLOCATOR_DEFAULT_SIZE);
	callbackFunctionAllocator = sphereTraceAllocatorConstruct(ST_FUNCTION_POOL_DEFAULT_SIZE, sizeof(ST_CallbackFunction), ST_INDEX_ALLOCATOR_DEFAULT_SIZE);
	contactEntryAllocator = sphereTraceAllocatorConstruct(ST_INDEX_POOL_DEFAULT_SIZE, sizeof(ST_SphereContactEntry), ST_INDEX_ALLOCATOR_DEFAULT_SIZE);
}

ST_IndexListData* sphereTraceAllocatorAllocateIndexListData()
{
	return sphereTraceAllocatorAllocateObject(&indexListAllocator);
}

void sphereTraceAllocatorFreeIndexListData(void* pIndex)
{
	sphereTraceAllocatorFreeObject(&indexListAllocator, pIndex);
}

ST_Vector3ListData* sphereTraceAllocatorAllocateVector3ListData()
{
	return sphereTraceAllocatorAllocateObject(&vector3ListAllocator);
}

void sphereTraceAllocatorFreeVector3ListData(void* pIndex)
{
	sphereTraceAllocatorFreeObject(&vector3ListAllocator, pIndex);
}

ST_Vector3 *sphereTraceAllocatorAllocateVector3()
{
	return sphereTraceAllocatorObjectPoolAllocateObject(&vector3Allocator);
}

void sphereTraceAllocatorFreeVector3(void* pIndex)
{
	sphereTraceAllocatorFreeObject(&vector3Allocator, pIndex);
}


ST_CallbackFunction* sphereTraceAllocatorAllocateCallbackFunction()
{
	return sphereTraceAllocatorAllocateObject(&callbackFunctionAllocator);
}

void sphereTraceAllocatorFreeCallbackFunction(void* pIndex)
{
	sphereTraceAllocatorFreeObject(&callbackFunctionAllocator, pIndex);
}

ST_SphereContactEntry* sphereTraceAllocatorAllocateContactEntry()
{
	return sphereTraceAllocatorAllocateObject(&contactEntryAllocator);
}


void sphereTraceAllocatorFreeContactEntry(void* pIndex)
{
	sphereTraceAllocatorFreeObject(&contactEntryAllocator, pIndex);
}