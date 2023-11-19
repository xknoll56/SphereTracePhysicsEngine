#pragma once

#include "SphereTraceMath.h"
#include "SphereTraceLists.h"
#include "SphereTraceCollider.h"

#define SPACIAL_PARTITION_STATIC_DIMENSION 10
#define SPACIAL_PARTITION_STATIC_SIZE SPACIAL_PARTITION_STATIC_DIMENSION*SPACIAL_PARTITION_STATIC_DIMENSION*SPACIAL_PARTITION_STATIC_DIMENSION

typedef struct ST_OctTreeNode
{
	ST_AABB aabb;
	b32 hasChildren;
	struct ST_OctTreeNode* children[8];
	ST_IndexList colliderList;
} ST_OctTreeNode;

typedef enum ST_OctTreNodeLocation
{
	ST_LEFT_DOWN_BACK = 0,
	ST_RIGHT_DOWN_BACK = 1,
	ST_LEFT_DOWN_FORWARD = 2,
	ST_RIGHT_DOWN_FORWARD = 3,
	ST_LEFT_UP_BACK = 4,
	ST_RIGHT_UP_BACK = 5,
	ST_LEFT_UP_FORWARD = 6,
	ST_RIGHT_UP_FORWARD = 7
} ST_OctTreNodeLocation;

typedef struct ST_OctTree
{
	ST_OctTreeNode root;
	ST_Index depth;
	ST_IndexList masterColliderList;
} ST_OctTree;

ST_OctTreeNode sphereTraceOctTreeNodeConstruct(ST_AABB aabb);
void sphereTraceOctTreeNodeSetChildAABBByIndex(ST_OctTreeNode* const pNode, ST_Index i, ST_AABB* paabb);
void sphereTraceOctTreeNodePopulateChildren(ST_OctTreeNode* const pNode);

typedef struct ST_SpacialPartitionBucket
{
	ST_Vector3 centroid;
	ST_AABB aabb;
	//ST_VectorArrayPointers planeColliderPointers;
	ST_IndexList planeColliderIndices;
	ST_IndexList triangleColliderIndices;
	ST_IndexList sphereColliderIndices;
	ST_IndexList bowlColliderIndices;
	ST_IndexList pipeColliderIndices;
	ST_IndexList uniformTerrainColliderIndices;
	ST_Index containerIndex;
} ST_SpacialPartitionBucket;

typedef struct ST_SpacialPartitiononDynamicContainer
{
	ST_SpacialPartitionBucket* buckets;
	float partitionSize;
	ST_Index count;
	ST_Index capacity;
} ST_SpacialPartitiononDynamicContainer;

typedef struct ST_SpacialPartitionStaticContainer
{
	ST_SpacialPartitionBucket* buckets;
	ST_SpacialPartitionBucket outsideBucket;
	float partitionSize;
	ST_Index count;
	ST_Index capacity;
	ST_AABB aabb;
}ST_SpacialPartitionStaticContainer;

//typedef struct ST_SpacialPartititonHeightNode
//{
//	float height;
//	struct ST_SpacialPartititonHeightNode* pChildren[4];
//} ST_SpacialPartititonHeightNode;
//typedef struct ST_SpacialPartitionHeightTree
//{
//	ST_SpacialPartititonHeightNode headNode;
//	ST_Index numEntries;
//	ST_Index depth;
//} ST_SpacialPartitionHeightTree;

//ST_SpacialPartitiononDynamicContainer sphereTraceSpacialPartitionStaticHorizontalCreate(float partitionSize);

ST_SpacialPartitionStaticContainer sphereTraceSpacialPartitionStaticConstruct(float partitionSize);

ST_SpacialPartitionBucket sphereTraceSpacialPartitionGetBucketWithIndex(ST_SpacialPartitionStaticContainer* const pSpacialPartitionStaticContainer, ST_Index bucketIndex);

ST_Index sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, ST_Vector3 position);

ST_IndexList sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, const ST_AABB* const aabb);

ST_IndexList sphereTraceSpacialPartitionStaticUpdateBucketIndicesFromAABBAndReturnDeletedBucketIndices(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, const ST_AABB* const aabb, ST_IndexList* const intList);

ST_Vector3 sphereTraceSpacialPartitionStaticGetNearestBucketIntersectionFromPositionAndDirection(const ST_SpacialPartitionBucket* const pCurrentBucket, ST_Vector3 start, ST_Vector3 dir, ST_Vector3* const incomingDirection);