#pragma once
#include <stdlib.h>
#include "SphereTraceMath.h"

typedef struct ST_IntListData
{
	int value;
	struct ST_IntListData* pNext;
} ST_IntListData;

typedef struct ST_IntList
{
	int count;
	ST_IntListData* pFirst;
} ST_IntList;

ST_IntList sphereTraceIntListConstruct();

void sphereTraceIntListAddFirst(ST_IntList* const pIntList, int value);

void sphereTraceIntListAddLast(ST_IntList* const pIntList, int value);

b32 sphereTraceIntListAddUnique(ST_IntList* const pIntList, int value);

void sphereTraceIntListRemoveFirstInstance(ST_IntList* const pIntList, int value);

void sphereTraceIntListFree(ST_IntList* const pIntList);

b32 sphereTraceIntListContains(const ST_IntList* const pIntList, int value);

ST_IntList sphereTraceIntListConstructForDeletedValues(const ST_IntList* const pOldIntList, const ST_IntList* const pNewIntList);

void sphereTraceIntListPrint(const ST_IntList* const pIntList);


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