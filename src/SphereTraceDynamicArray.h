#pragma once

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
	CONTACT_SPHERE_TRIANGLE = 2,
	CONTACT_SPHERE_TERRAIN_TRIANGLE = 3

} ST_ContactInfoType;
typedef struct ST_VectorArrayContactInfo
{
	void* contactInfos;
	int count;
	int capacity;
	ST_ContactInfoType type;
} ST_VectorArrayContactInfo;



ST_VectorArrayPointers sphereTraceVectorArrayPointersConstruct(int capacity);

//ST_SpherePlaneContactInfoVectorArray spherePlaneContac

void sphereTraceVectorArrayPointersPushBack(ST_VectorArrayPointers* const pVectorArrayPointer, void* const pPointer);

void sphereTraceVectorArrayPointersClear(ST_VectorArrayPointers* const pVectorArrayPointers);

void sphereTracePointerVectorArrayRemoveByIndex(ST_VectorArrayPointers* const pVectorArrayPointer, void* const pPointer);


ST_VectorArrayContactInfo sphereTracePointerVectorArrayContactInfoConstruct(int capacity, ST_ContactInfoType type);

void sphereTracePointerVectorArrayContactInfoPushBack(ST_VectorArrayContactInfo* pVectorArrayContactInfo, void* contactInfo, ST_ContactInfoType type);

void* sphereTracePointerVectorArrayContactInfoGet(const ST_VectorArrayContactInfo* const pVectorArrayContactInfo, int index, ST_ContactInfoType type);

void sphereTracePointerVectorArrayContactInfoFree(ST_VectorArrayContactInfo* const pVectorArrayContactInfo);

