#pragma once
#include <stdlib.h>

typedef uintptr_t ST_Index;

typedef struct ST_Tag
{
	char* data;
	ST_Index tagLength;
} ST_Tag;


ST_Tag sphereTraceTagConstruct(const char* tag);

void sphereTraceTagFree(ST_Tag* pTag);