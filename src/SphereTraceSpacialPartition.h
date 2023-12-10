#pragma once

#include "SphereTraceMath.h"
#include "SphereTraceLists.h"
#include "SphereTraceCollider.h"

#define SPACIAL_PARTITION_STATIC_DIMENSION 10
#define SPACIAL_PARTITION_STATIC_SIZE SPACIAL_PARTITION_STATIC_DIMENSION*SPACIAL_PARTITION_STATIC_DIMENSION*SPACIAL_PARTITION_STATIC_DIMENSION
#define ST_OCT_TREE_POINT_RADIUS_SQUARED 1.0f
#define ST_OCT_TREE_SMALLEST_OCTANT_SIZE 1.0f
#define ST_OCT_TREE_SKIN_WIDTH 1.0e-6f
#define ST_OCT_TREE_MAX_DEPTH 7


typedef struct ST_OctTreeNode
{
	ST_AABB aabb;
	b32 hasChildren;
	struct ST_OctTreeNode* children[8];
	struct ST_OctTreeNode* pParant;
	ST_IndexList colliderEntries;
	ST_Index depth;
	float boundingRadius;
} ST_OctTreeNode;


typedef struct ST_OctTree
{
	ST_OctTreeNode* root;
	ST_Index depth;
	ST_Index maxDepth;
	ST_Index gridIndex;
} ST_OctTree;

ST_OctTreeNode sphereTraceOctTreeNodeConstruct(ST_AABB aabb, ST_Index depth, ST_OctTreeNode* pParent);
void sphereTraceOctTreeNodeFree(ST_OctTreeNode* const pNode);
void sphereTraceOctTreeNodeSetChildAABBByIndex(ST_OctTreeNode* const pNode, ST_Index i, ST_AABB* paabb);
void sphereTraceOctTreeNodePopulateChildren(ST_OctTreeNode* const pNode, ST_Index bucketIndex);
//ST_Index sphereTraceOctTreeNodeCountObjectsBelow(ST_OctTreeNode* const pNode);
ST_Index sphereTraceOctTreeNodeGetColliderCountBelow(ST_OctTreeNode* const pNode, ST_IndexList* pUniqueCollider, ST_IndexList* pNodesBelow);

ST_OctTree sphereTraceOctTreeConstruct(ST_AABB aabb, ST_Index gridIndex);

//will return pointers to all oct-tree leaf nodes that intersect with this aabb
ST_IndexList sphereTraceOctTreeSampleIntersectionLeafs(ST_OctTree* const pOctTree, ST_AABB* const paabb);
//will sample both colliders and leafs and add whichever items are not on the current sorted lists
void sphereTraceOctTreeReSampleIntersectionLeafsAndColliders(ST_OctTree* const pOctTree, ST_AABB* const paabb,
	ST_IndexList* const pLeafs, ST_IndexList* const pColliders, b32 sampleDynamicColliders, b32 sampleStaticColliders);
//void sphereTraceOctTreeReSampleIntersectionLeafsAndSphereColliders(ST_OctTree* const pOctTree, ST_AABB* const paabb, ST_IndexList* const pLeafs, ST_IndexList* const pColliders);
//
void sphereTraceOctTreeSampleIntersectionLeafsAndCollidersFromPerspective(ST_OctTree* const pOctTree, ST_Vector3 from, ST_Direction dir, float fov, float f, ST_IndexList* const pLeafs, ST_IndexList* const pColliders);
//will return pointers to all colliders intersecting in the tree
//ST_IndexList sphereTraceOctTreeSampleIntersectionColliders(ST_OctTree* const pOctTree, ST_Collider* const pCollider);
//will insert point into a node, there can only be one point in each node,
//when another point is placed within a node, if it is within a certain
//radius it will not generate another node
ST_OctTreeNode* sphereTraceOctTreeSampleLeafNode(ST_OctTree* pTree, ST_Vector3 point);
//inserts an object id with a pointer to an aabb and returns the list of leaf nodes its added to
//void sphereTraceOctTreeInsertObject(ST_OctTree* pTree, ST_SpaceObjectEntry* pObjectEntry, b32 restructureTree);
//will allocate an object entry then insert it with a collider
void sphereTraceOctTreeInsertCollider(ST_OctTree* pTree, ST_Collider* pCollider, b32 restructureTree);
void sphereTraceOctTreeRemoveCollider(ST_OctTree* pTree, ST_Collider* pCollider, b32 restructureTree);
void sphereTraceOctTreeReInsertCollider(ST_OctTree* pTree, ST_Collider* pCollider, b32 restructureTree);
b32 sphereTraceOctTreeRayTrace(ST_Vector3 from, ST_Direction dir, float maxDist, const ST_OctTree* const pTree, ST_RayTraceData* const pRayTraceData);

//typedef struct ST_SpacialPartitionBucket
//{
//	ST_Vector3 centroid;
//	ST_AABB aabb;
//	//ST_VectorArrayPointers planeColliderPointers;
//	ST_IndexList planeColliderIndices;
//	ST_IndexList triangleColliderIndices;
//	ST_IndexList sphereColliderIndices;
//	ST_IndexList uniformTerrainColliderIndices;
//	ST_Index containerIndex;
//} ST_SpacialPartitionBucket;

typedef struct ST_OctTreeGrid
{
	ST_OctTree* treeBuckets;
	ST_IndexList outsideColliders;
	ST_AABB worldaabb;
	ST_Vector3 bucketHalfExtents;
	float minDim;
	ST_Index xBuckets;
	ST_Index yBuckets;
	ST_Index zBuckets;
	ST_Index capacity;
} ST_OctTreeGrid;

//typedef struct ST_SpacialPartitionStaticContainer
//{
//	ST_SpacialPartitionBucket* buckets;
//	ST_SpacialPartitionBucket outsideBucket;
//	float partitionSize;
//	ST_Index count;
//	ST_Index capacity;
//	ST_AABB aabb;
//}ST_SpacialPartitionStaticContainer;

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

//ST_SpacialPartitionStaticContainer sphereTraceSpacialPartitionStaticConstruct(float partitionSize);
//
//ST_SpacialPartitionBucket sphereTraceSpacialPartitionGetBucketWithIndex(ST_SpacialPartitionStaticContainer* const pSpacialPartitionStaticContainer, ST_Index bucketIndex);
//
//ST_Index sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, ST_Vector3 position);
//
//ST_IndexList sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, const ST_AABB* const aabb);
//
//ST_IndexList sphereTraceSpacialPartitionStaticUpdateBucketIndicesFromAABBAndReturnDeletedBucketIndices(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, const ST_AABB* const aabb, ST_IndexList* const intList);
//
//ST_Vector3 sphereTraceSpacialPartitionStaticGetNearestBucketIntersectionFromPositionAndDirection(const ST_SpacialPartitionBucket* const pCurrentBucket, ST_Vector3 start, ST_Vector3 dir, ST_Vector3* const incomingDirection);

ST_OctTreeGrid sphereTraceOctTreeGridConstruct(ST_AABB worldaabb, ST_Vector3 bucketHalfExtents);

ST_Index sphereTraceOctTreeGridGetBucketIndexFromPosition(const ST_OctTreeGrid* const pOctTreeGrid, ST_Vector3 position);

ST_IndexList sphereTraceOctTreeGridGetBucketIndicesFromAABB(const ST_OctTreeGrid* const pOctTreeGrid, const ST_AABB* const aabb);

ST_IndexList sphereTraceOctTreeGridGetBucketIndicesFromAABBCorners(const ST_OctTreeGrid* const pOctTreeGrid, const ST_AABB* const aabb);

ST_IndexList sphereTraceOctTreeGridGetBucketIndicesFromRayTrace(ST_Vector3 start, ST_Direction dir, float maxDist, const ST_OctTreeGrid* const pGrid, ST_RayTraceData* const pData);

void sphereTraceOctTreeGridInsertCollider(ST_OctTreeGrid* const pGrid, ST_Collider* pCollider, b32 restructureTrees);

void sphereTraceOctTreeGridInsertDynamicCollider(ST_OctTreeGrid* const pGrid, ST_Collider* pCollider, b32 restructureTrees);

void sphereTraceOctTreeGridRemoveCollider(ST_OctTreeGrid* const pGrid, ST_Collider* pCollider, b32 restructureTrees);

void sphereTraceOctTreeGridReInsertCollider(ST_OctTreeGrid* const pGrid, ST_Collider* pCollider, b32 restructureTrees);

void sphereTraceOctTreeGridReSampleIntersectionLeafsAndColliders(ST_OctTreeGrid* const pGrid, ST_AABB* const paabb,
	ST_IndexList* const pLeafs, ST_IndexList* const pColliders, b32 sampleDynamicColliders, b32 sampleStaticColliders);

void sphereTraceOctTreeGridSampleIntersectionLeafsAndCollidersFromPerspective(ST_OctTreeGrid* const pGrid, ST_Vector3 from, 
	ST_Direction dir, float fov, float f, ST_IndexList* const pLeafs, ST_IndexList* const pColliders);