#pragma once
#include <stdlib.h>
#include "SphereTraceMath.h"

typedef uintptr_t ST_Index;

typedef struct ST_IndexListData
{
	ST_Index value;
	struct ST_IndexListData* pNext;
} ST_IndexListData;

typedef struct ST_IndexList
{
	ST_Index count;
	ST_IndexListData* pFirst;
} ST_IndexList;

ST_IndexList sphereTraceIndexListConstruct();

void sphereTraceIndexListAddFirst(ST_IndexList* const pIntList, ST_Index value);

void sphereTraceIndexListAddLast(ST_IndexList* const pIntList, ST_Index value);


b32 sphereTraceIndexListAddUnique(ST_IndexList* const pIntList, ST_Index value);


void sphereTraceIndexListRemoveFirstInstance(ST_IndexList* const pIntList, ST_Index value);


void sphereTraceIndexListFree(ST_IndexList* const pIntList);

b32 sphereTraceIndexListContains(const ST_IndexList* const pIntList, ST_Index value);

ST_IndexList sphereTraceIndexListConstructForDeletedValues(const ST_IndexList* const pOldIntList, const ST_IndexList* const pNewIntList);

void sphereTraceIndexListPrint(const ST_IndexList* const pIntList);

//all sorted list functions, assumes sorted in ascending order
void sphereTraceSortedIndexListAdd(ST_IndexList* const pIntList, ST_Index value);
b32 sphereTraceSortedIndexListAddUnique(ST_IndexList* const pIntList, ST_Index value);
void sphereTraceSortedIndexListRemove(ST_IndexList* const pIntList, ST_Index value);
b32 sphereTraceSortedIndexListContains(const ST_IndexList* const pIntList, ST_Index value);

typedef struct ST_Vector3ListData
{
	ST_Vector3 value;
	struct ST_Vector3ListData* pNext;
} ST_Vector3ListData;

typedef struct ST_Vector3List
{
	int count;
	ST_Vector3ListData* pFirst;
} ST_Vector3List;

ST_Vector3List sphereTraceVector3ListConstruct();

void sphereTraceVector3ListAddLast(ST_Vector3List* const pVector3List, ST_Vector3 value);

void sphereTraceVector3ListAddFirst(ST_Vector3List* const pVector3List, ST_Vector3 value);

void sphereTraceVector3ListRemoveFirst(ST_Vector3List* const pVector3List);

void sphereTraceVector3ListRemoveLast(ST_Vector3List* const pVector3List);

b32 sphereTraceVector3ListAddUnique(ST_Vector3List* const pVector3List, ST_Vector3 value);

void sphereTraceVector3ListRemoveFirstInstance(ST_Vector3List* const pVector3List, ST_Vector3 value);

void sphereTraceVector3ListFree(ST_Vector3List* const pVector3List);

b32 sphereTraceVector3ListContains(const ST_Vector3List* const pVector3List, ST_Vector3 value);


void sphereTraceVector3ListPrint(const ST_Vector3List* const pVector3List);

ST_Vector3 sphereTraceVector3ListAverage(const ST_Vector3List* const pVector3List);

void sphereTraceVector3ListMoveOffset(const ST_Vector3List* const pVector3List, ST_Vector3 offset);