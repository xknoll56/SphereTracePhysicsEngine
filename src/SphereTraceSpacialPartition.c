#include "SphereTraceSpacialPartition.h"
#include "SphereTraceGlobals.h"

ST_OctTreeNode sphereTraceOctTreeNodeConstruct(ST_AABB aabb)
{
	ST_OctTreeNode node;
	node.aabb = aabb;
	node.hasChildren = ST_FALSE;
	node.colliderList = sphereTraceIndexListConstruct();
}

void sphereTraceOctTreeNodeSetChildAABBByIndex(ST_OctTreeNode* const pNode, ST_Index i, ST_AABB* paabb)
{
	paabb->halfExtents = sphereTraceVector3Scale(pNode->aabb.halfExtents, 0.5f);
	switch (i)
	{
	case ST_LEFT_DOWN_BACK:
		paabb->lowExtent = pNode->aabb.lowExtent;
		break;
	case ST_RIGHT_DOWN_BACK:
		paabb->lowExtent = sphereTraceVector3Add(pNode->aabb.lowExtent, sphereTraceVector3Construct(pNode->aabb.halfExtents.x, 0.0f, 0.0f));
		break;
	case ST_LEFT_DOWN_FORWARD:
		paabb->lowExtent = sphereTraceVector3Add(pNode->aabb.lowExtent, sphereTraceVector3Construct(0.0f, 0.0f, pNode->aabb.halfExtents.z));
		break;
	case ST_RIGHT_DOWN_FORWARD:
		paabb->lowExtent = sphereTraceVector3Add(pNode->aabb.lowExtent, sphereTraceVector3Construct(pNode->aabb.halfExtents.x, 0.0f, pNode->aabb.halfExtents.z));
		break;
	case ST_LEFT_UP_BACK:
		paabb->lowExtent = sphereTraceVector3Add(pNode->aabb.lowExtent, sphereTraceVector3Construct(0.0f, pNode->aabb.halfExtents.y, 0.0f));
		break;
	case ST_RIGHT_UP_BACK:
		paabb->lowExtent = sphereTraceVector3Add(pNode->aabb.lowExtent, sphereTraceVector3Construct(pNode->aabb.halfExtents.x, pNode->aabb.halfExtents.y, 0.0f));
		break;
	case ST_LEFT_UP_FORWARD:
		paabb->lowExtent = sphereTraceVector3Add(pNode->aabb.lowExtent, sphereTraceVector3Construct(0.0f, pNode->aabb.halfExtents.y, pNode->aabb.halfExtents.z));
		break;
	case ST_RIGHT_UP_FORWARD:
		paabb->lowExtent = sphereTraceVector3Add(pNode->aabb.lowExtent, sphereTraceVector3Construct(pNode->aabb.halfExtents.x, pNode->aabb.halfExtents.y, pNode->aabb.halfExtents.z));
		break;
	}
	paabb->highExtent = sphereTraceVector3Add(paabb->lowExtent, pNode->aabb.halfExtents);
	paabb->center = sphereTraceVector3Add(paabb->lowExtent, paabb->halfExtents);
}

void sphereTraceOctTreeNodePopulateChildren(ST_OctTreeNode* const pNode)
{
	for (ST_Index i = 0; i < 8; i++)
	{
		pNode->children[i] = sphereTraceAllocatorAllocateOctTreeNode();
		sphereTraceOctTreeNodeSetChildAABBByIndex(pNode, i, &pNode->children[i]->aabb);
		pNode->children[i]->hasChildren = ST_FALSE;
	}
	pNode->hasChildren = ST_TRUE;
}

void sphereTraceOctTreeNodeAABBIntersectionWithChildren(ST_OctTreeNode* const pNode, ST_AABB* paabb, b32 intersections[8])
{
	if (pNode->hasChildren)
	{
		ST_Vector3 minDists = sphereTraceVector3Add(pNode->children[0]->aabb.halfExtents, paabb->halfExtents);
		b32 intersectionsByCoord[6];

		if (fabsf(pNode->children[ST_LEFT_DOWN_BACK]->aabb.center.x - paabb->center.x) <= minDists.x)
			intersectionsByCoord[0] = 1;
		else
			intersectionsByCoord[0] = 0;

		if(fabsf(pNode->children[ST_RIGHT_DOWN_BACK]->aabb.center.x - paabb->center.x) <= minDists.x)
			intersectionsByCoord[1] = 1;
		else
			intersectionsByCoord[1] = 1;

		if(fabsf(pNode->children[ST_LEFT_DOWN_BACK]->aabb.center.y - paabb->center.y) <= minDists.y)
			intersectionsByCoord[2] = 1;
		else
			intersectionsByCoord[2] = 0;
		
		if(fabsf(pNode->children[ST_LEFT_UP_BACK]->aabb.center.y - paabb->center.y) <= minDists.y)
			intersectionsByCoord[3] = 1;
		else
			intersectionsByCoord[3] = 0;

		if(fabsf(pNode->children[ST_LEFT_DOWN_BACK]->aabb.center.z - paabb->center.z) <= minDists.z)
			intersectionsByCoord[4] = 1;
		else
			intersectionsByCoord[4] = 0;

		if(fabsf(pNode->children[ST_LEFT_DOWN_FORWARD]->aabb.center.z - paabb->center.z) <= minDists.z)
			intersectionsByCoord[5] = 1;
		else
			intersectionsByCoord[5] = 0;

		intersections[ST_LEFT_DOWN_BACK] = intersectionsByCoord[0] && intersectionsByCoord[2] && intersectionsByCoord[4];
		intersections[ST_RIGHT_DOWN_BACK] = intersectionsByCoord[1] && intersectionsByCoord[2] && intersectionsByCoord[4];
		intersections[ST_LEFT_DOWN_FORWARD] = intersectionsByCoord[0] && intersectionsByCoord[2] && intersectionsByCoord[5];
		intersections[ST_RIGHT_DOWN_FORWARD] = intersectionsByCoord[1] && intersectionsByCoord[2] && intersectionsByCoord[5];
		intersections[ST_LEFT_UP_BACK] = intersectionsByCoord[0] && intersectionsByCoord[3] && intersectionsByCoord[4];
		intersections[ST_RIGHT_UP_BACK] = intersectionsByCoord[1] && intersectionsByCoord[3] && intersectionsByCoord[4];
		intersections[ST_LEFT_UP_FORWARD] = intersectionsByCoord[0] && intersectionsByCoord[3] && intersectionsByCoord[5];
		intersections[ST_RIGHT_UP_FORWARD] = intersectionsByCoord[1] && intersectionsByCoord[3] && intersectionsByCoord[5];
	}
}

//ST_SpacialPartitiononDynamicContainer sphereTraceSpacialPartitionStaticHorizontalCreate(float partitionSize)
//{
//	ST_SpacialPartitiononDynamicContainer spacialPartitionContainer;
//	spacialPartitionContainer.buckets = (ST_SpacialPartitionBucket*)malloc(sizeof(ST_SpacialPartitionBucket));
//	spacialPartitionContainer.buckets[0].centroid = gVector3Zero;
//	spacialPartitionContainer.buckets[0].containerIndex = 0;
//	spacialPartitionContainer.buckets[0].planeColliderIndices = sphereTraceIndexListConstruct();
//	spacialPartitionContainer.buckets[0].sphereColliderIndices = sphereTraceIndexListConstruct();
//	spacialPartitionContainer.buckets[0].bowlColliderIndices = sphereTraceIndexListConstruct();
//	spacialPartitionContainer.buckets[0].pipeColliderIndices = sphereTraceIndexListConstruct();
//	spacialPartitionContainer.count = 1;
//	spacialPartitionContainer.capacity = 1;
//	return spacialPartitionContainer;
//}


ST_SpacialPartitionStaticContainer sphereTraceSpacialPartitionStaticConstruct(float partitionSize)
{
	ST_SpacialPartitionStaticContainer spacialPartitionContainer;
	spacialPartitionContainer.partitionSize = partitionSize;
	spacialPartitionContainer.buckets = malloc(SPACIAL_PARTITION_STATIC_SIZE * sizeof(ST_SpacialPartitionStaticContainer));
	for (ST_Index z = 0; z < SPACIAL_PARTITION_STATIC_DIMENSION; z++)
	{
		for (ST_Index y = 0; y < SPACIAL_PARTITION_STATIC_DIMENSION; y++)
		{
			for (ST_Index x = 0; x < SPACIAL_PARTITION_STATIC_DIMENSION; x++)
			{
				float centroidX = -partitionSize * 0.5f + ((float)x - SPACIAL_PARTITION_STATIC_DIMENSION / 2 + 1) * partitionSize;
				float centroidY = -partitionSize * 0.5f + ((float)y - SPACIAL_PARTITION_STATIC_DIMENSION / 2 + 1) * partitionSize;
				float centroidZ = -partitionSize * 0.5f + ((float)z - SPACIAL_PARTITION_STATIC_DIMENSION / 2 + 1) * partitionSize;
				ST_SpacialPartitionBucket bucket;
				bucket.centroid = sphereTraceVector3Construct(centroidX, centroidY, centroidZ);
				bucket.containerIndex = z * SPACIAL_PARTITION_STATIC_DIMENSION * SPACIAL_PARTITION_STATIC_DIMENSION + y * SPACIAL_PARTITION_STATIC_DIMENSION + x;
				bucket.planeColliderIndices = sphereTraceIndexListConstruct();
				bucket.triangleColliderIndices = sphereTraceIndexListConstruct();
				bucket.sphereColliderIndices = sphereTraceIndexListConstruct();
				bucket.bowlColliderIndices = sphereTraceIndexListConstruct();
				bucket.pipeColliderIndices = sphereTraceIndexListConstruct();
				bucket.uniformTerrainColliderIndices = sphereTraceIndexListConstruct();
				bucket.aabb.halfExtents = sphereTraceVector3Construct(partitionSize * 0.5f, partitionSize * 0.5f, partitionSize * 0.5f);
				bucket.aabb.lowExtent = sphereTraceVector3Subtract(bucket.centroid, bucket.aabb.halfExtents);
				bucket.aabb.highExtent = sphereTraceVector3Add(bucket.centroid, bucket.aabb.halfExtents);
				spacialPartitionContainer.buckets[bucket.containerIndex] = bucket;
			}
		}
	}
	spacialPartitionContainer.aabb.halfExtents = sphereTraceVector3Construct((SPACIAL_PARTITION_STATIC_DIMENSION / 2) * partitionSize, (SPACIAL_PARTITION_STATIC_DIMENSION / 2) * partitionSize, (SPACIAL_PARTITION_STATIC_DIMENSION / 2) * partitionSize);
	spacialPartitionContainer.aabb.lowExtent = sphereTraceVector3Add(spacialPartitionContainer.buckets[0].centroid, sphereTraceVector3Construct(-partitionSize * 0.5f, -partitionSize * 0.5f, -partitionSize * 0.5f));
	spacialPartitionContainer.aabb.highExtent = sphereTraceVector3Add(spacialPartitionContainer.buckets[SPACIAL_PARTITION_STATIC_SIZE - 1].centroid, sphereTraceVector3Construct(partitionSize * 0.5f, partitionSize * 0.5f, partitionSize * 0.5f));
	spacialPartitionContainer.count = SPACIAL_PARTITION_STATIC_SIZE;
	spacialPartitionContainer.capacity = SPACIAL_PARTITION_STATIC_SIZE;
	spacialPartitionContainer.outsideBucket.planeColliderIndices = sphereTraceIndexListConstruct();
	spacialPartitionContainer.outsideBucket.triangleColliderIndices = sphereTraceIndexListConstruct();
	spacialPartitionContainer.outsideBucket.sphereColliderIndices = sphereTraceIndexListConstruct();
	spacialPartitionContainer.outsideBucket.uniformTerrainColliderIndices = sphereTraceIndexListConstruct();
	spacialPartitionContainer.outsideBucket.bowlColliderIndices = sphereTraceIndexListConstruct();
	spacialPartitionContainer.outsideBucket.pipeColliderIndices = sphereTraceIndexListConstruct();
	return spacialPartitionContainer;
}

ST_SpacialPartitionBucket sphereTraceSpacialPartitionGetBucketWithIndex(ST_SpacialPartitionStaticContainer* const pSpacialPartitionStaticContainer, ST_Index bucketIndex)
{
	ST_SpacialPartitionBucket bucket;
	if (bucketIndex == -1)
	{
		bucket = pSpacialPartitionStaticContainer->outsideBucket;
	}
	else
	{
		bucket = pSpacialPartitionStaticContainer->buckets[bucketIndex];
	}
	return bucket;
}

ST_Index sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, ST_Vector3 position)
{
	if (sphereTraceColliderAABBIsPointInside(&pSpacialPartitionContainer->aabb, position))
	{
		ST_Index z = (ST_Index)((position.z + pSpacialPartitionContainer->aabb.halfExtents.z) / pSpacialPartitionContainer->partitionSize);
		ST_Index y = (ST_Index)((position.y + pSpacialPartitionContainer->aabb.halfExtents.y) / pSpacialPartitionContainer->partitionSize);
		ST_Index x = (ST_Index)((position.x + pSpacialPartitionContainer->aabb.halfExtents.x) / pSpacialPartitionContainer->partitionSize);
		ST_Index index = z * SPACIAL_PARTITION_STATIC_DIMENSION * SPACIAL_PARTITION_STATIC_DIMENSION + y * SPACIAL_PARTITION_STATIC_DIMENSION + x;
		if (index >= 0 && index < pSpacialPartitionContainer->count)
			return index;
		else
			return -1;
	}
	else
		return -1;
}

ST_IndexList sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, const ST_AABB* const aabb)
{
	ST_IndexList intList = sphereTraceIndexListConstruct();
	float xExtent = fminf(aabb->highExtent.x, pSpacialPartitionContainer->aabb.highExtent.x);
	float yExtent = fminf(aabb->highExtent.y, pSpacialPartitionContainer->aabb.highExtent.y);
	float zExtent = fminf(aabb->highExtent.z, pSpacialPartitionContainer->aabb.highExtent.z);
	ST_Vector3 start = aabb->lowExtent;
	float xIncrement = fminf(aabb->halfExtents.x, pSpacialPartitionContainer->partitionSize);
	if (xIncrement == 0.0f)
		xIncrement = pSpacialPartitionContainer->partitionSize;
	if (start.x < pSpacialPartitionContainer->aabb.lowExtent.x)
	{
		start.x = pSpacialPartitionContainer->aabb.lowExtent.x;
	}
	float yIncrement = fminf(aabb->halfExtents.y, pSpacialPartitionContainer->partitionSize);
	if (yIncrement == 0.0f)
		yIncrement = pSpacialPartitionContainer->partitionSize;
	if ( start.y < pSpacialPartitionContainer->aabb.lowExtent.y)
	{
		start.y = pSpacialPartitionContainer->aabb.lowExtent.y;
	}
	float zIncrement = fminf(aabb->halfExtents.z, pSpacialPartitionContainer->partitionSize);
	if (zIncrement == 0.0f)
		zIncrement = pSpacialPartitionContainer->partitionSize;
	if (start.z < pSpacialPartitionContainer->aabb.lowExtent.z)
	{
		start.z = pSpacialPartitionContainer->aabb.lowExtent.z;
	}

	for (float x = start.x; x <= xExtent; x += xIncrement)
	{
		for (float y = start.y; y <= yExtent; y += yIncrement)
		{
			for (float z = start.z; z <= zExtent; z += zIncrement)
			{
				sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(x, y, z)));
			}
		}
	}
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3){aabb->highExtent.x, aabb->lowExtent.y, aabb->lowExtent.z}));
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->lowExtent.x, aabb->highExtent.y, aabb->lowExtent.z }));
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->lowExtent.x, aabb->lowExtent.y, aabb->highExtent.z }));
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->highExtent.x, aabb->lowExtent.y, aabb->highExtent.z }));
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->highExtent.x, aabb->highExtent.y, aabb->lowExtent.z }));
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->lowExtent.x, aabb->highExtent.y, aabb->highExtent.z }));
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, aabb->highExtent));
	//
	return intList;
}

ST_IndexList sphereTraceSpacialPartitionStaticUpdateBucketIndicesFromAABBAndReturnDeletedBucketIndices(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, const ST_AABB* const aabb, ST_IndexList* const intList)
{
	ST_IndexList intListToRemove = sphereTraceIndexListConstruct();
	for (float x = aabb->lowExtent.x; x <= aabb->highExtent.x; x += pSpacialPartitionContainer->partitionSize)
	{
		for (float y = aabb->lowExtent.y; y <= aabb->highExtent.y; y += pSpacialPartitionContainer->partitionSize)
		{
			for (float z = aabb->lowExtent.z; z <= aabb->highExtent.z; z += pSpacialPartitionContainer->partitionSize)
			{
				int bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(x, y, z));
				sphereTraceIndexListAddUnique(intList, bucket);
			}
		}
	}
	int bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->highExtent.x, aabb->lowExtent.y, aabb->lowExtent.z));
	sphereTraceIndexListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->lowExtent.x, aabb->highExtent.y, aabb->lowExtent.z));
	sphereTraceIndexListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->lowExtent.x, aabb->lowExtent.y, aabb->highExtent.z));
	sphereTraceIndexListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->highExtent.x, aabb->lowExtent.y, aabb->highExtent.z));
	sphereTraceIndexListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->highExtent.x, aabb->highExtent.y, aabb->lowExtent.z));
	sphereTraceIndexListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->lowExtent.x, aabb->highExtent.y, aabb->highExtent.z));
	sphereTraceIndexListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, aabb->highExtent);
	sphereTraceIndexListAddUnique(intList, bucket);

	return intListToRemove;
}

ST_Vector3 sphereTraceSpacialPartitionStaticGetNearestBucketIntersectionFromPositionAndDirection(const ST_SpacialPartitionBucket* const pCurrentBucket, ST_Vector3 start, ST_Vector3 dir, ST_Vector3* const incomingDirection)
{
	//ST_Vector3 t = gVector3Max;
	float t = FLT_MAX;
	if (dir.x > 0.0f)
	{
		float tentative = (pCurrentBucket->aabb.highExtent.x - start.x) / dir.x;
		if (tentative < t && tentative>0.0f)
		{
			t = tentative;
			*incomingDirection = gVector3Right;
		}
	}
	else if (dir.x < 0.0f)
	{
		float tentative = (pCurrentBucket->aabb.lowExtent.x - start.x) / dir.x;
		if (tentative < t && tentative>0.0f)
		{
			t = tentative;
			*incomingDirection = gVector3Left;
		}
	}

	if (dir.y > 0.0f)
	{
		float tentative = (pCurrentBucket->aabb.highExtent.y - start.y) / dir.y;
		if (tentative < t && tentative>0.0f)
		{
			t = tentative;
			*incomingDirection = gVector3Up;
		}
	}
	else if (dir.y < 0.0f)
	{
		float tentative = (pCurrentBucket->aabb.lowExtent.y - start.y) / dir.y;
		if (tentative < t && tentative>0.0f)
		{
			t = tentative;
			*incomingDirection = gVector3Down;
		}
	}

	if (dir.z > 0.0f)
	{
		float tentative = (pCurrentBucket->aabb.highExtent.z - start.z) / dir.z;
		if (tentative < t && tentative>0.0f)
		{
			t = tentative;
			*incomingDirection = gVector3Forward;
		}
	}
	else if (dir.z < 0.0f)
	{
		float tentative = (pCurrentBucket->aabb.lowExtent.z - start.z) / dir.z;
		if (tentative < t && tentative>0.0f)
		{
			t = tentative;
			*incomingDirection = gVector3Back;
		}
	}

	//float tMin = fminf(fminf(t.x, t.y), t.z) + FLT_MIN;
	ST_Vector3 intersection = sphereTraceVector3AddAndScale(start, dir, t);
	return intersection;
}
