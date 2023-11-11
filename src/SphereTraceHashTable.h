#pragma once
#include "SphereTraceMath.h"
#include "SphereTraceLists.h"
#include <stdarg.h>

typedef enum ST_HashArgumentType
{
	ST_HASH_ARGUMENT_VECTOR3 = 0,
	ST_HASH_ARGUMENT_INDEX = 1
} ST_HashArgumentType;

typedef struct ST_HashTable
{
	ST_Index count;
	ST_IndexList* lists;
	ST_Index (*hashFunction)();
	ST_IndexList activeLists;
	ST_HashArgumentType argumentType;
} ST_HashTable;


ST_HashTable sphereTraceHashTableConstruct(ST_Index count, ST_Index (*hashFunction)(), ST_HashArgumentType argumentType);

void sphereTraceHashTableAddUnique(ST_HashTable* const pHashTable, ST_Index id, ...);

void sphereTraceHashTableRemove(ST_HashTable* const pHashTable, ST_Index id);