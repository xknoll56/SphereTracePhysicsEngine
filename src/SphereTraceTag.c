#include "SphereTraceTag.h"
#include <string.h>

ST_Tag sphereTraceTagConstruct(const char* tag)
{
	ST_Tag ret;
	rsize_t len = strlen(tag);
	ret.data = malloc(len+1);
	memcpy(ret.data, tag, len);
	ret.data[len] = '\0';
	ret.tagLength = len;
	return ret;
}

void sphereTraceTagFree(ST_Tag* pTag)
{
	free(pTag->data);
}