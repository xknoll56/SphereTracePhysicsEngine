#include "SphereTraceSpacialPartition.h"
#include "SphereTraceGlobals.h"

ST_SpacialPartitiononDynamicContainer sphereTraceSpacialPartitionStaticHorizontalCreate(float partitionSize)
{
	ST_SpacialPartitiononDynamicContainer spacialPartitionContainer;
	spacialPartitionContainer.buckets = (ST_SpacialPartitionBucket*)malloc(sizeof(ST_SpacialPartitionBucket));
	spacialPartitionContainer.buckets[0].centroid = gVector3Zero;
	spacialPartitionContainer.buckets[0].containerIndex = 0;
	spacialPartitionContainer.buckets[0].planeColliderIndices = sphereTraceIndexListConstruct();
	spacialPartitionContainer.buckets[0].sphereColliderIndices = sphereTraceIndexListConstruct();
	spacialPartitionContainer.buckets[0].bowlColliderIndices = sphereTraceIndexListConstruct();
	spacialPartitionContainer.buckets[0].pipeColliderIndices = sphereTraceIndexListConstruct();
	spacialPartitionContainer.count = 1;
	spacialPartitionContainer.capacity = 1;
	return spacialPartitionContainer;
}


ST_SpacialPartitionStaticContainer sphereTraceSpacialPartitionStaticConstruct(float partitionSize)
{
	ST_SpacialPartitionStaticContainer spacialPartitionContainer;
	spacialPartitionContainer.partitionSize = partitionSize;
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
				bucket.sphereColliderIndices = sphereTraceIndexListConstruct();
				bucket.bowlColliderIndices = sphereTraceIndexListConstruct();
				bucket.pipeColliderIndices = sphereTraceIndexListConstruct();
				bucket.uniformTerrainColliderIndices = sphereTraceIndexListConstruct();
				bucket.aabb.halfExtents = sphereTraceVector3Construct(partitionSize * 0.5f, partitionSize * 0.5f, partitionSize * 0.5f);
				bucket.aabb.leftDownBackTransformedVertex = sphereTraceVector3Subtract(bucket.centroid, bucket.aabb.halfExtents);
				bucket.aabb.rightTopForwardsTransformedVertex = sphereTraceVector3Add(bucket.centroid, bucket.aabb.halfExtents);
				spacialPartitionContainer.buckets[bucket.containerIndex] = bucket;
			}
		}
	}
	spacialPartitionContainer.aabb.halfExtents = sphereTraceVector3Construct((SPACIAL_PARTITION_STATIC_DIMENSION / 2) * partitionSize, (SPACIAL_PARTITION_STATIC_DIMENSION / 2) * partitionSize, (SPACIAL_PARTITION_STATIC_DIMENSION / 2) * partitionSize);
	spacialPartitionContainer.aabb.leftDownBackTransformedVertex = sphereTraceVector3Add(spacialPartitionContainer.buckets[0].centroid, sphereTraceVector3Construct(-partitionSize * 0.5f, -partitionSize * 0.5f, -partitionSize * 0.5f));
	spacialPartitionContainer.aabb.rightTopForwardsTransformedVertex = sphereTraceVector3Add(spacialPartitionContainer.buckets[SPACIAL_PARTITION_STATIC_SIZE - 1].centroid, sphereTraceVector3Construct(partitionSize * 0.5f, partitionSize * 0.5f, partitionSize * 0.5f));
	spacialPartitionContainer.count = SPACIAL_PARTITION_STATIC_SIZE;
	spacialPartitionContainer.capacity = SPACIAL_PARTITION_STATIC_SIZE;
	spacialPartitionContainer.outsideBucket.planeColliderIndices = sphereTraceIndexListConstruct();
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
	float xExtent = fminf(aabb->rightTopForwardsTransformedVertex.x, pSpacialPartitionContainer->aabb.rightTopForwardsTransformedVertex.x);
	float yExtent = fminf(aabb->rightTopForwardsTransformedVertex.y, pSpacialPartitionContainer->aabb.rightTopForwardsTransformedVertex.y);
	float zExtent = fminf(aabb->rightTopForwardsTransformedVertex.z, pSpacialPartitionContainer->aabb.rightTopForwardsTransformedVertex.z);
	ST_Vector3 start = aabb->leftDownBackTransformedVertex;
	float xIncrement = fminf(aabb->halfExtents.x, pSpacialPartitionContainer->partitionSize);
	if (xIncrement == 0.0f)
		xIncrement = pSpacialPartitionContainer->partitionSize;
	if (start.x < pSpacialPartitionContainer->aabb.leftDownBackTransformedVertex.x)
	{
		start.x = pSpacialPartitionContainer->aabb.leftDownBackTransformedVertex.x;
	}
	float yIncrement = fminf(aabb->halfExtents.y, pSpacialPartitionContainer->partitionSize);
	if (yIncrement == 0.0f)
		yIncrement = pSpacialPartitionContainer->partitionSize;
	if ( start.y < pSpacialPartitionContainer->aabb.leftDownBackTransformedVertex.y)
	{
		start.y = pSpacialPartitionContainer->aabb.leftDownBackTransformedVertex.y;
	}
	float zIncrement = fminf(aabb->halfExtents.z, pSpacialPartitionContainer->partitionSize);
	if (zIncrement == 0.0f)
		zIncrement = pSpacialPartitionContainer->partitionSize;
	if (start.z < pSpacialPartitionContainer->aabb.leftDownBackTransformedVertex.z)
	{
		start.z = pSpacialPartitionContainer->aabb.leftDownBackTransformedVertex.z;
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
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3){aabb->rightTopForwardsTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->leftDownBackTransformedVertex.z}));
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->leftDownBackTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->leftDownBackTransformedVertex.z }));
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->leftDownBackTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z }));
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->rightTopForwardsTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z }));
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->rightTopForwardsTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->leftDownBackTransformedVertex.z }));
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->leftDownBackTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z }));
	//sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, aabb->rightTopForwardsTransformedVertex));
	//
	return intList;
}

ST_IndexList sphereTraceSpacialPartitionStaticUpdateBucketIndicesFromAABBAndReturnDeletedBucketIndices(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, const ST_AABB* const aabb, ST_IndexList* const intList)
{
	ST_IndexList intListToRemove = sphereTraceIndexListConstruct();
	for (float x = aabb->leftDownBackTransformedVertex.x; x <= aabb->rightTopForwardsTransformedVertex.x; x += pSpacialPartitionContainer->partitionSize)
	{
		for (float y = aabb->leftDownBackTransformedVertex.y; y <= aabb->rightTopForwardsTransformedVertex.y; y += pSpacialPartitionContainer->partitionSize)
		{
			for (float z = aabb->leftDownBackTransformedVertex.z; z <= aabb->rightTopForwardsTransformedVertex.z; z += pSpacialPartitionContainer->partitionSize)
			{
				int bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(x, y, z));
				sphereTraceIndexListAddUnique(intList, bucket);
			}
		}
	}
	int bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->rightTopForwardsTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->leftDownBackTransformedVertex.z));
	sphereTraceIndexListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->leftDownBackTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->leftDownBackTransformedVertex.z));
	sphereTraceIndexListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->leftDownBackTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z));
	sphereTraceIndexListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->rightTopForwardsTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z));
	sphereTraceIndexListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->rightTopForwardsTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->leftDownBackTransformedVertex.z));
	sphereTraceIndexListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->leftDownBackTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z));
	sphereTraceIndexListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, aabb->rightTopForwardsTransformedVertex);
	sphereTraceIndexListAddUnique(intList, bucket);

	return intListToRemove;
}

ST_Vector3 sphereTraceSpacialPartitionStaticGetNearestBucketIntersectionFromPositionAndDirection(const ST_SpacialPartitionBucket* const pCurrentBucket, ST_Vector3 start, ST_Vector3 dir, ST_Vector3* const incomingDirection)
{
	//ST_Vector3 t = gVector3Max;
	float t = FLT_MAX;
	if (dir.x > 0.0f)
	{
		float tentative = (pCurrentBucket->aabb.rightTopForwardsTransformedVertex.x - start.x) / dir.x;
		if (tentative < t && tentative>0.0f)
		{
			t = tentative;
			*incomingDirection = gVector3Right;
		}
	}
	else if (dir.x < 0.0f)
	{
		float tentative = (pCurrentBucket->aabb.leftDownBackTransformedVertex.x - start.x) / dir.x;
		if (tentative < t && tentative>0.0f)
		{
			t = tentative;
			*incomingDirection = gVector3Left;
		}
	}

	if (dir.y > 0.0f)
	{
		float tentative = (pCurrentBucket->aabb.rightTopForwardsTransformedVertex.y - start.y) / dir.y;
		if (tentative < t && tentative>0.0f)
		{
			t = tentative;
			*incomingDirection = gVector3Up;
		}
	}
	else if (dir.y < 0.0f)
	{
		float tentative = (pCurrentBucket->aabb.leftDownBackTransformedVertex.y - start.y) / dir.y;
		if (tentative < t && tentative>0.0f)
		{
			t = tentative;
			*incomingDirection = gVector3Down;
		}
	}

	if (dir.z > 0.0f)
	{
		float tentative = (pCurrentBucket->aabb.rightTopForwardsTransformedVertex.z - start.z) / dir.z;
		if (tentative < t && tentative>0.0f)
		{
			t = tentative;
			*incomingDirection = gVector3Forward;
		}
	}
	else if (dir.z < 0.0f)
	{
		float tentative = (pCurrentBucket->aabb.leftDownBackTransformedVertex.z - start.z) / dir.z;
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
