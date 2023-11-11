#include "SphereTraceHashTable.h"

ST_HashTable sphereTraceHashTableConstruct(ST_Index count, ST_Index (*hashFunction)(), ST_HashArgumentType argumentType)
{
	ST_HashTable table;
	table.count = count;
	table.lists = (ST_IndexList*)malloc(count * sizeof(ST_IndexList));
	table.hashFunction = hashFunction;
	table.argumentType = argumentType;
	return table;
}

void sphereTraceHashTableAddUnique(ST_HashTable* const pHashTable, ST_Index id, ...)
{
	va_list list;
	switch (pHashTable->argumentType)
	{
	case ST_HASH_ARGUMENT_VECTOR3:
	{
		va_start(list, 1);
		ST_Vector3 arg = va_arg(list, ST_Vector3);
		va_end(list);
		ST_Index index = pHashTable->hashFunction(arg);
		sphereTraceIndexListAddUnique(&pHashTable->lists[index], id);
	}
	break;
	case ST_HASH_ARGUMENT_INDEX:
	{
		va_start(list, 1);
		ST_Index arg = va_arg(list, ST_Index);
		va_end(list);
		ST_Index index = pHashTable->hashFunction(arg);
		sphereTraceIndexListAddUnique(&pHashTable->lists[index], id);
	}
	break;
	}
}

void sphereTraceHashTableRemove(ST_HashTable* const pHashTable, ST_Index id)
{

}