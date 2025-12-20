#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#include <stdlib.h>

typedef uintptr_t ST_Index;

#define ST_TAG_SIZE 64
typedef struct ST_Tag
{
	char data[ST_TAG_SIZE];
	ST_Index tagLength;
} ST_Tag;


ST_Tag sphereTraceTagConstruct(const char* tag);

#ifdef __cplusplus
}
#endif
