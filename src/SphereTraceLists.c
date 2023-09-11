

#include "SphereTraceLists.h"
#include "SphereTraceMath.h"
#include "SphereTraceGlobals.h"

ST_IntList sphereTraceIntListConstruct()
{
	ST_IntList intList;
	intList.count = 0;
	intList.pFirst = NULL;
	return intList;
}

void sphereTraceIntListAddFirst(ST_IntList* const pIntList, int value)
{
	ST_IntListData* pFirst = pIntList->pFirst;
	pIntList->pFirst = (ST_IntListData*)malloc(sizeof(ST_IntListData));
	pIntList->pFirst->value = value;
	pIntList->pFirst->pNext = pFirst;
	pIntList->count++;

}

void sphereTraceIntListAddLast(ST_IntList* const pIntList, int value)
{
	if (pIntList->pFirst == NULL)
	{
		pIntList->pFirst = (ST_IntListData*)malloc(sizeof(ST_IntListData));
		pIntList->pFirst->value = value;
		pIntList->pFirst->pNext = NULL;
		pIntList->count++;
	}
	else
	{
		ST_IntListData* pData = pIntList->pFirst;
		for (int i = 0; i < pIntList->count - 1; i++)
		{
			pData = pData->pNext;
		}
		pData->pNext = (ST_IntListData*)malloc(sizeof(ST_IntListData));
		pData->pNext->value = value;
		pData->pNext->pNext = NULL;
		pIntList->count++;
	}
}

b32 sphereTraceIntListAddUnique(ST_IntList* const pIntList, int value)
{
	if (pIntList->pFirst == NULL)
	{
		pIntList->pFirst = (ST_IntListData*)malloc(sizeof(ST_IntListData));
		pIntList->pFirst->value = value;
		pIntList->pFirst->pNext = NULL;
		pIntList->count++;
		return 1;
	}
	else
	{
		ST_IntListData* pData = pIntList->pFirst;
		for (int i = 0; i < pIntList->count - 1; i++)
		{
			if (pData != NULL)
			{
				if (pData->value == value)
					return 0;
			}
			pData = pData->pNext;
		}
		if (pData != NULL)
		{
			if (pData->value == value)
				return 0;
		}
		pData->pNext = (ST_IntListData*)malloc(sizeof(ST_IntListData));
		pData->pNext->value = value;
		pData->pNext->pNext = NULL;
		pIntList->count++;
		return 1;
	}
}

void sphereTraceIntListRemoveFirstInstance(ST_IntList* const pIntList, int value)
{
	if (pIntList->count == 1)
	{
		if (pIntList->pFirst->value == value)
			free(pIntList->pFirst);
		pIntList->pFirst = NULL;
		pIntList->count--;
	}
	else if (pIntList->count > 1)
	{
		ST_IntListData* pData = pIntList->pFirst;
		ST_IntListData* pNext = pIntList->pFirst->pNext;
		if (pIntList->pFirst->value == value)
		{
			free(pIntList->pFirst);
			pIntList->pFirst = pNext;
			pIntList->count--;
			return;
		}
		for (int i = 0; i < pIntList->count - 1; i++)
		{
			if (pNext->value == value)
			{
				pData->pNext = pNext->pNext;
				free(pNext);
				pIntList->count--;
				return;
			}
			pData = pNext;
			pNext = pNext->pNext;
		}
	}
}

void sphereTraceIntListFree(ST_IntList* const pIntList)
{
	ST_IntListData* pData = pIntList->pFirst;
	ST_IntListData* pNext;
	for (int i = 0; i < pIntList->count; i++)
	{
		if (pData != NULL)
		{
			pNext = pData->pNext;
			free(pData);
			pData = pNext;
		}
	}
	pIntList->pFirst = NULL;
	pIntList->count = 0;
}

b32 sphereTraceIntListContains(const ST_IntList* const pIntList, int value)
{
	ST_IntListData* data = pIntList->pFirst;
	for (int i = 0; i < pIntList->count; i++)
	{
		if (data != NULL)
		{
			if (data->value == value)
				return 1;
			data = data->pNext;
		}
	}
	return 0;
}

ST_IntList sphereTraceIntListConstructForDeletedValues(const ST_IntList* const pOldIntList, const ST_IntList* const pNewIntList)
{
	ST_IntList deletedIntList = sphereTraceIntListConstruct();
	ST_IntListData* data = pOldIntList->pFirst;
	for (int i = 0; i < pOldIntList->count; i++)
	{
		if (data != NULL)
		{
			if (!sphereTraceIntListContains(pNewIntList, data->value))
				sphereTraceIntListAddUnique(&deletedIntList, data->value);
			data = data->pNext;
		}
	}
	return deletedIntList;
}

void sphereTraceIntListPrint(const ST_IntList* const pIntList)
{
	ST_IntListData* data = pIntList->pFirst;
	for (int i = 0; i < pIntList->count; i++)
	{
		printf("%i:%i,", i, data->value);
		data = data->pNext;
	}
	printf("\n");
}


ST_Vector3List sphereTraceVector3ListConstruct()
{
	ST_Vector3List vector3List;
	vector3List.count = 0;
	vector3List.pFirst = NULL;
	return vector3List;
}

void sphereTraceVector3ListAddLast(ST_Vector3List* const pVector3List, ST_Vector3 value)
{
	if (pVector3List->pFirst == NULL)
	{
		pVector3List->pFirst = (ST_Vector3ListData*)malloc(sizeof(ST_Vector3ListData));
		pVector3List->pFirst->value = value;
		pVector3List->pFirst->pNext = NULL;
		pVector3List->count++;
	}
	else
	{
		ST_Vector3ListData* pData = pVector3List->pFirst;
		for (int i = 0; i < pVector3List->count - 1; i++)
		{
			pData = pData->pNext;
		}
		pData->pNext = (ST_Vector3ListData*)malloc(sizeof(ST_Vector3ListData));
		pData->pNext->value = value;
		pData->pNext->pNext = NULL;
		pVector3List->count++;
	}
}

void sphereTraceVector3ListAddFirst(ST_Vector3List* const pVector3List, ST_Vector3 value)
{
	ST_Vector3ListData* pFirst = pVector3List->pFirst;
	pVector3List->pFirst = (ST_Vector3ListData*)malloc(sizeof(ST_Vector3ListData));
	pVector3List->pFirst->value = value;
	pVector3List->pFirst->pNext = pFirst;
	pVector3List->count++;
}

void sphereTraceVector3ListRemoveFirst(ST_Vector3List* const pVector3List)
{
	if (pVector3List->pFirst != NULL)
	{
		ST_Vector3ListData* pNext = pVector3List->pFirst->pNext;
		free(pVector3List->pFirst);
		pVector3List->pFirst = pNext;
		pVector3List->count--;
	}
}

void sphereTraceVector3ListRemoveLast(ST_Vector3List* const pVector3List)
{
	if (pVector3List->pFirst != NULL)
	{
		ST_Vector3ListData* pLast = pVector3List->pFirst;
		for (int i = 0; i < pVector3List->count - 1; i++)
		{
			pLast = pLast->pNext;
		}
		free(pLast);
		pLast = NULL;
		pVector3List->count--;
	}
}

b32 sphereTraceVector3ListAddUnique(ST_Vector3List* const pVector3List, ST_Vector3 value)
{
	if (pVector3List->pFirst == NULL)
	{
		pVector3List->pFirst = (ST_Vector3ListData*)malloc(sizeof(ST_Vector3ListData));
		pVector3List->pFirst->value = value;
		pVector3List->pFirst->pNext = NULL;
		pVector3List->count++;
		return 1;
	}
	else
	{
		ST_Vector3ListData* pData = pVector3List->pFirst;
		for (int i = 0; i < pVector3List->count - 1; i++)
		{
			if (pData != NULL)
			{
				if (sphereTraceVector3Equal(pData->value, value))
					return 0;
			}
			pData = pData->pNext;
		}
		if (pData != NULL)
		{
			if (sphereTraceVector3Equal(pData->value, value))
				return 0;
		}
		pData->pNext = (ST_Vector3ListData*)malloc(sizeof(ST_Vector3ListData));
		pData->pNext->value = value;
		pData->pNext->pNext = NULL;
		pVector3List->count++;
		return 1;
	}
}

void sphereTraceVector3ListRemoveFirstInstance(ST_Vector3List* const pVector3List, ST_Vector3 value)
{
	if (pVector3List->count == 1)
	{
		if (sphereTraceVector3Equal(pVector3List->pFirst->value, value))
			free(pVector3List->pFirst);
		pVector3List->pFirst = NULL;
		pVector3List->count--;
	}
	else if (pVector3List->count > 1)
	{
		ST_Vector3ListData* pData = pVector3List->pFirst;
		ST_Vector3ListData* pNext = pVector3List->pFirst->pNext;
		if (sphereTraceVector3Equal(pVector3List->pFirst->value, value))
		{
			free(pVector3List->pFirst);
			pVector3List->pFirst = pNext;
			pVector3List->count--;
			return;
		}
		for (int i = 0; i < pVector3List->count - 1; i++)
		{
			if (sphereTraceVector3Equal(pNext->value, value))
			{
				pData->pNext = pNext->pNext;
				free(pNext);
				pVector3List->count--;
				return;
			}
			pData = pNext;
			pNext = pNext->pNext;
		}
	}
}

void sphereTraceVector3ListFree(ST_Vector3List* const pVector3List)
{
	ST_Vector3ListData* pData = pVector3List->pFirst;
	ST_Vector3ListData* pNext;
	for (int i = 0; i < pVector3List->count; i++)
	{
		if (pData != NULL)
		{
			pNext = pData->pNext;
			free(pData);
			pData = pNext;
		}
	}
	pVector3List->pFirst = NULL;
	pVector3List->count = 0;
}

b32 sphereTraceVector3ListContains(const ST_Vector3List* const pVector3List, ST_Vector3 value)
{
	ST_Vector3ListData* data = pVector3List->pFirst;
	for (int i = 0; i < pVector3List->count; i++)
	{
		if (data != NULL)
		{
			if (sphereTraceVector3Equal(data->value, value))
				return 1;
			data = data->pNext;
		}
	}
	return 0;
}


void sphereTraceVector3ListPrint(const ST_Vector3List* const pVector3List)
{
	ST_Vector3ListData* data = pVector3List->pFirst;
	for (int i = 0; i < pVector3List->count; i++)
	{
		printf("%i:%i,", i, data->value);
		data = data->pNext;
	}
	printf("\n");
}

ST_Vector3 sphereTraceVector3ListAverage(const ST_Vector3List* const pVector3List)
{
	ST_Vector3 avg = gVector3Zero;
	ST_Vector3ListData* pild = pVector3List->pFirst;
	for (int i = 0; i < pVector3List->count; i++)
	{
		avg = sphereTraceVector3Add(avg, pild->value);
		pild = pild->pNext;
	}
	return sphereTraceVector3Scale(avg, 1.0f/pVector3List->count);
}

void sphereTraceVector3ListMoveOffset(const ST_Vector3List* const pVector3List, ST_Vector3 offset)
{
	ST_Vector3ListData* pild = pVector3List->pFirst;
	for (int i = 0; i < pVector3List->count; i++)
	{
		sphereTraceVector3AddByRef(&pild->value, offset);
		pild = pild->pNext;
	}
}