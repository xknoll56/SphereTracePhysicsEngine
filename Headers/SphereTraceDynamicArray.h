#pragma once
#include <stdlib.h>


typedef struct ST_VectorArrayPointers
{
	void** data;
	int count;
	int capacity;
} ST_VectorArrayPointers;

typedef enum ST_ContactInfoType
{
	CONTACT_SPHERE_PLANE = 0,
	CONTACT_SPHERE_SPHERE = 1,
	CONTACT_SPHERE_TRIANGLE = 2

} ST_ContactInfoType;
typedef struct ST_VectorArrayContactInfo
{
	void* contactInfos;
	int count;
	int capacity;
	ST_ContactInfoType type;
} ST_VectorArrayContactInfo;



ST_VectorArrayPointers sphereTraceVectorArrayPointersConstruct(int capacity)
{
	ST_VectorArrayPointers vectorArrayPointers;
	vectorArrayPointers.data = (void**)malloc(capacity * sizeof(void*));
	vectorArrayPointers.count = 0;
	vectorArrayPointers.capacity = capacity;
	return vectorArrayPointers;
}

//ST_SpherePlaneContactInfoVectorArray spherePlaneContac

void sphereTraceVectorArrayPointersPushBack(ST_VectorArrayPointers* const pVectorArrayPointer, void* const pPointer)
{
	if (pVectorArrayPointer->count + 1 >= pVectorArrayPointer->capacity)
	{
		if (pVectorArrayPointer->capacity == 0)
			pVectorArrayPointer->capacity++;
		pVectorArrayPointer->capacity *= 2;
		void** data = (void**)realloc((void**)pVectorArrayPointer->data, pVectorArrayPointer->capacity * sizeof(void*));
		if (data != NULL)
		{
			pVectorArrayPointer->data = data;
		}
	}
	(void*)pVectorArrayPointer->data[pVectorArrayPointer->count] = pPointer;
	pVectorArrayPointer->count++;
}

void sphereTraceVectorArrayPointersClear(ST_VectorArrayPointers* const pVectorArrayPointers)
{
	free(pVectorArrayPointers->data);
	pVectorArrayPointers->count = 0;
	pVectorArrayPointers->capacity = 0;
}

void sphereTracePointerVectorArrayRemoveByIndex(ST_VectorArrayPointers* const pVectorArrayPointer, void* const pPointer)
{

}


ST_VectorArrayContactInfo sphereTracePointerVectorArrayContactInfoConstruct(int capacity, ST_ContactInfoType type)
{
	ST_VectorArrayContactInfo vectorArrayContactInfo;
	switch (type)
	{
	case CONTACT_SPHERE_PLANE:
		vectorArrayContactInfo.contactInfos = (ST_SpherePlaneContactInfo*)malloc(capacity * sizeof(ST_SpherePlaneContactInfo));
		break;
	case CONTACT_SPHERE_SPHERE:
		vectorArrayContactInfo.contactInfos = (ST_SphereSphereContactInfo*)malloc(capacity * sizeof(ST_SphereSphereContactInfo));
		break;
	case CONTACT_SPHERE_TRIANGLE:
		vectorArrayContactInfo.contactInfos = (ST_SphereTriangleContactInfo*)malloc(capacity * sizeof(ST_SphereTriangleContactInfo));
		break;
	}
	vectorArrayContactInfo.count = 0;
	vectorArrayContactInfo.capacity = capacity;
	vectorArrayContactInfo.type = type;
	return vectorArrayContactInfo;
}

void sphereTracePointerVectorArrayContactInfoPushBackSpherePlane(ST_VectorArrayContactInfo* pVectorArrayContactInfo, ST_SpherePlaneContactInfo contactInfo)
{
	if (pVectorArrayContactInfo->count + 1 >= pVectorArrayContactInfo->capacity)
	{
		if (pVectorArrayContactInfo->capacity == 0)
			pVectorArrayContactInfo->capacity++;
		pVectorArrayContactInfo->capacity *= 2;
		void* data;
		data = (ST_SpherePlaneContactInfo*)realloc(pVectorArrayContactInfo->contactInfos, pVectorArrayContactInfo->capacity * sizeof(ST_SpherePlaneContactInfo));
		if (data != NULL)
		{
			pVectorArrayContactInfo->contactInfos = data;
		}
	}
	ST_SpherePlaneContactInfo* pSpherePlaneContactInfoVectorArray = pVectorArrayContactInfo->contactInfos;
	pSpherePlaneContactInfoVectorArray[pVectorArrayContactInfo->count] = contactInfo;
	pVectorArrayContactInfo->count++;
}

ST_SpherePlaneContactInfo sphereTracePointerVectorArrayContactInfoGetSpherePlane(const ST_VectorArrayContactInfo* const pVectorArrayContactInfo, int index)
{
	if (index < pVectorArrayContactInfo->count)
	{
		ST_SpherePlaneContactInfo* pSpherePlaneContactInfoVectorArray = pVectorArrayContactInfo->contactInfos;
		return pSpherePlaneContactInfoVectorArray[index];
	}
}

void sphereTracePointerVectorArrayContactInfoPushBackSphereTriangle(ST_VectorArrayContactInfo* pVectorArrayContactInfo, ST_SphereTriangleContactInfo contactInfo)
{
	if (pVectorArrayContactInfo->count + 1 >= pVectorArrayContactInfo->capacity)
	{
		if (pVectorArrayContactInfo->capacity == 0)
			pVectorArrayContactInfo->capacity++;
		pVectorArrayContactInfo->capacity *= 2;
		void* data;
		data = (ST_SphereTriangleContactInfo*)realloc(pVectorArrayContactInfo->contactInfos, pVectorArrayContactInfo->capacity * sizeof(ST_SphereTriangleContactInfo));
		if (data != NULL)
		{
			pVectorArrayContactInfo->contactInfos = data;
		}
	}
	ST_SphereTriangleContactInfo* pSphereTriangleContactInfoVectorArray = pVectorArrayContactInfo->contactInfos;
	pSphereTriangleContactInfoVectorArray[pVectorArrayContactInfo->count] = contactInfo;
	pVectorArrayContactInfo->count++;
}

ST_SphereTriangleContactInfo sphereTracePointerVectorArrayContactInfoGetSphereTriangle(const ST_VectorArrayContactInfo* const pVectorArrayContactInfo, int index)
{
	if (index < pVectorArrayContactInfo->count)
	{
		ST_SphereTriangleContactInfo* pSphereTriangleContactInfoVectorArray = pVectorArrayContactInfo->contactInfos;
		return pSphereTriangleContactInfoVectorArray[index];
	}
}


void sphereTracePointerVectorArrayContactInfoPushBackSphereSphere(ST_VectorArrayContactInfo* pVectorArrayContactInfo, ST_SphereSphereContactInfo contactInfo)
{
	if (pVectorArrayContactInfo->count + 1 >= pVectorArrayContactInfo->capacity)
	{
		if (pVectorArrayContactInfo->capacity == 0)
			pVectorArrayContactInfo->capacity++;
		pVectorArrayContactInfo->capacity *= 2;
		void* data;
		data = (ST_SphereSphereContactInfo*)realloc(pVectorArrayContactInfo->contactInfos, pVectorArrayContactInfo->capacity * sizeof(ST_SphereSphereContactInfo));
		if (data != NULL)
		{
			pVectorArrayContactInfo->contactInfos = data;
		}
	}
	ST_SphereSphereContactInfo* pSphereSphereContactInfoVectorArray = pVectorArrayContactInfo->contactInfos;
	pSphereSphereContactInfoVectorArray[pVectorArrayContactInfo->count] = contactInfo;
	pVectorArrayContactInfo->count++;
}

ST_SphereSphereContactInfo sphereTracePointerVectorArrayContactInfoGetSphereSphere(const ST_VectorArrayContactInfo* const pVectorArrayContactInfo, int index)
{
	if (index < pVectorArrayContactInfo->count)
	{
		ST_SphereSphereContactInfo* pSphereSphereContactInfoVectorArray = pVectorArrayContactInfo->contactInfos;
		return pSphereSphereContactInfoVectorArray[index];
	}
}

void sphereTracePointerVectorArrayContactInfoFree(ST_VectorArrayContactInfo* const pVectorArrayContactInfo)
{
	free(pVectorArrayContactInfo->contactInfos);
	pVectorArrayContactInfo->capacity = 0;
	pVectorArrayContactInfo->count = 0;
}

