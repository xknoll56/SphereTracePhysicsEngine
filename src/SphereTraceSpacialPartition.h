#pragma once

#include "SphereTraceMath.h"
#include "SphereTraceLists.h"
#include "SphereTraceCollider.h"

#define SPACIAL_PARTITION_STATIC_DIMENSION 10
#define SPACIAL_PARTITION_STATIC_SIZE SPACIAL_PARTITION_STATIC_DIMENSION*SPACIAL_PARTITION_STATIC_DIMENSION*SPACIAL_PARTITION_STATIC_DIMENSION
typedef struct ST_SpacialPartitionBucket
{
	ST_Vector3 centroid;
	ST_AABB aabb;
	//ST_VectorArrayPointers planeColliderPointers;
	ST_IntList planeColliderIndices;
	ST_IntList sphereColliderIndices;
	ST_IntList uniformTerrainColliderIndices;
	int containerIndex;
} ST_SpacialPartitionBucket;

typedef struct ST_SpacialPartitiononDynamicContainer
{
	ST_SpacialPartitionBucket* buckets;
	float partitionSize;
	int count;
	int capacity;
} ST_SpacialPartitiononDynamicContainer;

typedef struct ST_SpacialPartitionStaticContainer
{
	ST_SpacialPartitionBucket buckets[SPACIAL_PARTITION_STATIC_SIZE];
	ST_SpacialPartitionBucket outsideBucket;
	float partitionSize;
	int count;
	int capacity;
	ST_AABB aabb;
}ST_SpacialPartitionStaticContainer;

ST_SpacialPartitiononDynamicContainer sphereTraceSpacialPartitionStaticHorizontalCreate(float partitionSize);

ST_SpacialPartitionStaticContainer sphereTraceSpacialPartitionStaticConstruct(float partitionSize);

int sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, ST_Vector3 position);

ST_IntList sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, const ST_AABB* const aabb);

ST_IntList sphereTraceSpacialPartitionStaticUpdateBucketIndicesFromAABBAndReturnDeletedBucketIndices(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, const ST_AABB* const aabb, ST_IntList* const intList);

ST_Vector3 sphereTraceSpacialPartitionStaticGetNearestBucketIntersectionFromPositionAndDirection(const ST_SpacialPartitionBucket* const pCurrentBucket, ST_Vector3 start, ST_Vector3 dir, ST_Vector3* const incomingDirection);