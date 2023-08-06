#pragma once
#define SPACIAL_PARTITION_STATIC_DIMENSION 10
#define SPACIAL_PARTITION_STATIC_SIZE SPACIAL_PARTITION_STATIC_DIMENSION*SPACIAL_PARTITION_STATIC_DIMENSION*SPACIAL_PARTITION_STATIC_DIMENSION
typedef struct ST_SpacialPartitionBucket
{
	ST_Vector3 centroid;
	ST_AABB aabb;
	//ST_VectorArrayPointers planeColliderPointers;
	ST_IntList planeColliderIndices;
	ST_IntList sphereColliderIndices;
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

ST_SpacialPartitiononDynamicContainer sphereTraceSpacialPartitionStaticHorizontalCreate(float partitionSize)
{
	ST_SpacialPartitiononDynamicContainer spacialPartitionContainer;
	spacialPartitionContainer.buckets = (ST_SpacialPartitiononDynamicContainer*)malloc(sizeof(ST_SpacialPartitiononDynamicContainer));
	spacialPartitionContainer.buckets[0].centroid = gVector3Zero;
	spacialPartitionContainer.buckets[0].containerIndex = 0;
	spacialPartitionContainer.buckets[0].planeColliderIndices = sphereTraceIntListConstruct();
	spacialPartitionContainer.buckets[0].sphereColliderIndices = sphereTraceIntListConstruct();
	spacialPartitionContainer.count = 1;
	spacialPartitionContainer.capacity = 1;
}


ST_SpacialPartitionStaticContainer sphereTraceSpacialPartitionStaticConstruct(float partitionSize)
{
	ST_SpacialPartitionStaticContainer spacialPartitionContainer;
	spacialPartitionContainer.partitionSize = partitionSize;
	for (int z = 0; z < SPACIAL_PARTITION_STATIC_DIMENSION; z++)
	{
		for (int y = 0; y < SPACIAL_PARTITION_STATIC_DIMENSION; y++)
		{
			for (int x = 0; x < SPACIAL_PARTITION_STATIC_DIMENSION; x++)
			{
				float centroidX = -partitionSize * 0.5f + (x - SPACIAL_PARTITION_STATIC_DIMENSION/2+1) * partitionSize;
				float centroidY = -partitionSize * 0.5f + (y - SPACIAL_PARTITION_STATIC_DIMENSION/2+1) * partitionSize;
				float centroidZ = -partitionSize * 0.5f + (z - SPACIAL_PARTITION_STATIC_DIMENSION/2+1) * partitionSize;
				ST_SpacialPartitionBucket bucket;
				bucket.centroid = (ST_Vector3){ centroidX, centroidY, centroidZ };
				bucket.containerIndex = z * SPACIAL_PARTITION_STATIC_DIMENSION* SPACIAL_PARTITION_STATIC_DIMENSION + y * SPACIAL_PARTITION_STATIC_DIMENSION + x;
				bucket.planeColliderIndices = sphereTraceIntListConstruct();
				bucket.sphereColliderIndices = sphereTraceIntListConstruct();
				bucket.aabb.halfExtents = (ST_Vector3){ partitionSize * 0.5f, partitionSize * 0.5f, partitionSize * 0.5f };
				bucket.aabb.leftDownBackTransformedVertex = sphereTraceVector3Subtract(bucket.centroid, bucket.aabb.halfExtents);
				bucket.aabb.rightTopForwardsTransformedVertex = sphereTraceVector3Add(bucket.centroid, bucket.aabb.halfExtents);
				spacialPartitionContainer.buckets[bucket.containerIndex] = bucket;
			}
		}
	}
	spacialPartitionContainer.aabb.halfExtents = (ST_Vector3){ (SPACIAL_PARTITION_STATIC_DIMENSION / 2)*partitionSize , (SPACIAL_PARTITION_STATIC_DIMENSION / 2) * partitionSize , (SPACIAL_PARTITION_STATIC_DIMENSION / 2) * partitionSize };
	spacialPartitionContainer.aabb.leftDownBackTransformedVertex = sphereTraceVector3Add(spacialPartitionContainer.buckets[0].centroid, (ST_Vector3) { -partitionSize * 0.5f, -partitionSize * 0.5f, -partitionSize * 0.5f });
	spacialPartitionContainer.aabb.rightTopForwardsTransformedVertex = sphereTraceVector3Add(spacialPartitionContainer.buckets[SPACIAL_PARTITION_STATIC_SIZE-1].centroid, (ST_Vector3) { partitionSize * 0.5f, partitionSize * 0.5f, partitionSize * 0.5f });
	spacialPartitionContainer.count = SPACIAL_PARTITION_STATIC_SIZE;
	spacialPartitionContainer.capacity = SPACIAL_PARTITION_STATIC_SIZE;
	spacialPartitionContainer.outsideBucket.planeColliderIndices = sphereTraceIntListConstruct();
	spacialPartitionContainer.outsideBucket.sphereColliderIndices = sphereTraceIntListConstruct();
	return spacialPartitionContainer;
}

int sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, ST_Vector3 position)
{
	if (sphereTraceColliderAABBIsPointInside(&pSpacialPartitionContainer->aabb, position))
	{
		int z = (int)((position.z + pSpacialPartitionContainer->aabb.halfExtents.z) / pSpacialPartitionContainer->partitionSize);
		int y = (int)((position.y + pSpacialPartitionContainer->aabb.halfExtents.y) / pSpacialPartitionContainer->partitionSize);
		int x = (int)((position.x + pSpacialPartitionContainer->aabb.halfExtents.x) / pSpacialPartitionContainer->partitionSize);
		return z * SPACIAL_PARTITION_STATIC_DIMENSION * SPACIAL_PARTITION_STATIC_DIMENSION + y * SPACIAL_PARTITION_STATIC_DIMENSION + x;
	}
	else
		return -1;
}

ST_IntList sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, const ST_AABB* const aabb)
{
	ST_IntList intList = sphereTraceIntListConstruct();
	float xExtent = aabb->rightTopForwardsTransformedVertex.x + pSpacialPartitionContainer->partitionSize;
	float yExtent = aabb->rightTopForwardsTransformedVertex.y + pSpacialPartitionContainer->partitionSize;
	float zExtent = aabb->rightTopForwardsTransformedVertex.z + pSpacialPartitionContainer->partitionSize;

	for (float x = aabb->leftDownBackTransformedVertex.x; x < xExtent; x += pSpacialPartitionContainer->partitionSize)
	{
		for (float y = aabb->leftDownBackTransformedVertex.y; y < yExtent; y += pSpacialPartitionContainer->partitionSize)
		{
			for (float z = aabb->leftDownBackTransformedVertex.z; z < zExtent; z += pSpacialPartitionContainer->partitionSize)
			{
				sphereTraceIntListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { x, y, z }));
			}
		}
	}
	//sphereTraceIntListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3){aabb->rightTopForwardsTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->leftDownBackTransformedVertex.z}));
	//sphereTraceIntListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->leftDownBackTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->leftDownBackTransformedVertex.z }));
	//sphereTraceIntListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->leftDownBackTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z }));
	//sphereTraceIntListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->rightTopForwardsTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z }));
	//sphereTraceIntListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->rightTopForwardsTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->leftDownBackTransformedVertex.z }));
	//sphereTraceIntListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->leftDownBackTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z }));
	//sphereTraceIntListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, aabb->rightTopForwardsTransformedVertex));
	//
	return intList;
}

ST_IntList sphereTraceSpacialPartitionStaticUpdateBucketIndicesFromAABBAndReturnDeletedBucketIndices(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, const ST_AABB* const aabb, ST_IntList* const intList)
{
	ST_IntList intListToRemove = sphereTraceIntListConstruct();
	for (float x = aabb->leftDownBackTransformedVertex.x; x <= aabb->rightTopForwardsTransformedVertex.x; x += pSpacialPartitionContainer->partitionSize)
	{
		for (float y = aabb->leftDownBackTransformedVertex.y; y <= aabb->rightTopForwardsTransformedVertex.y; y += pSpacialPartitionContainer->partitionSize)
		{
			for (float z = aabb->leftDownBackTransformedVertex.z; z <= aabb->rightTopForwardsTransformedVertex.z; z += pSpacialPartitionContainer->partitionSize)
			{
				int bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { x, y, z });
				sphereTraceIntListAddUnique(intList, bucket);
			}
		}
	}
	int bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->rightTopForwardsTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->leftDownBackTransformedVertex.z });
	sphereTraceIntListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->leftDownBackTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->leftDownBackTransformedVertex.z });
	sphereTraceIntListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->leftDownBackTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z });
	sphereTraceIntListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->rightTopForwardsTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z });
	sphereTraceIntListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->rightTopForwardsTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->leftDownBackTransformedVertex.z });
	sphereTraceIntListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, (ST_Vector3) { aabb->leftDownBackTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z });
	sphereTraceIntListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, aabb->rightTopForwardsTransformedVertex);
	sphereTraceIntListAddUnique(intList, bucket);

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

//void spacialPartitionHorizontalInsertPlane(ST_SpacialPartionDynamicContainer* const pSpacialPartitionContainer, const ST_PlaneCollider* const pPlaneCollider)
//{
//
//}
