#pragma once

#include "SphereTraceMath.h"
#include "SphereTraceLists.h"
#include "SphereTraceCollider.h"

#define SPACIAL_PARTITION_STATIC_DIMENSION 10
#define SPACIAL_PARTITION_STATIC_SIZE SPACIAL_PARTITION_STATIC_DIMENSION*SPACIAL_PARTITION_STATIC_DIMENSION*SPACIAL_PARTITION_STATIC_DIMENSION
#define ST_OCT_TREE_POINT_RADIUS_SQUARED 0.33f

typedef struct ST_OctTreeNode
{
	ST_AABB aabb;
	b32 hasChildren;
	struct ST_OctTreeNode* children[8];
	ST_IndexList colliderList;
	ST_Vector3 point;
} ST_OctTreeNode;


typedef struct ST_OctTree
{
	ST_OctTreeNode* root;
	ST_Index depth;
} ST_OctTree;

ST_OctTreeNode sphereTraceOctTreeNodeConstruct(ST_AABB aabb);
void sphereTraceOctTreeNodeSetChildAABBByIndex(ST_OctTreeNode* const pNode, ST_Index i, ST_AABB* paabb);
void sphereTraceOctTreeNodePopulateChildren(ST_OctTreeNode* const pNode);

ST_OctTree sphereTraceOctTreeConstruct(ST_AABB aabb);

//will return pointers to all oct-tree leaf nodes that intersect with this aabb
ST_IndexList sphereTraceOctTreeSampleIntersectionLeafs(ST_OctTree* const pOctTree, ST_AABB* const paabb);
//will return pointers to all colliders intersecting in the tree
ST_IndexList sphereTraceOctTreeSampleIntersectionColliders(ST_OctTree* const pOctTree, ST_Collider* const pCollider);
//will insert point into a node, there can only be one point in each node,
//when another point is placed within a node, if it is within a certain
//radius it will not generate another node
void sphereTraceOctTreeInsertPoint(ST_OctTree* pTree, ST_Vector3 point);
void sphereTraceOctTreeInsertCollider(ST_OctTree* pTree, ST_Collider* pCollider);
ST_Index sphereTraceOctTreeGetMaxCollidersOnLeaf(ST_OctTree* pTree, ST_OctTreeNode** ppNodeWithMax);
void sphereTraceOctTreeDisectNodesWithMinColliders(ST_OctTree* pTree, ST_Index minColliders);

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