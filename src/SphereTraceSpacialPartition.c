#include "SphereTraceSpacialPartition.h"

extern const ST_Vector3 gVector3Up;
extern const ST_Vector3 gVector3Right;
extern const ST_Vector3 gVector3Forward;
extern const ST_Vector3 gVector3Zero;
extern const ST_Vector3 gVector3One;
extern const ST_Vector3 gVector3Max;
extern const ST_Vector3 gVector3Left;
extern const ST_Vector3 gVector3Down;
extern const ST_Vector3 gVector3Back;
extern const ST_Vector4 gVector4Zero;
extern const ST_Vector4 gVector4One;
extern const ST_Vector4 gVector4ColorRed;
extern const ST_Vector4 gVector4ColorGreen;
extern const ST_Vector4 gVector4ColorBlue;
extern const ST_Quaternion gQuaternionIdentity;

ST_SpacialPartitiononDynamicContainer sphereTraceSpacialPartitionStaticHorizontalCreate(float partitionSize)
{
	ST_SpacialPartitiononDynamicContainer spacialPartitionContainer;
	spacialPartitionContainer.buckets = (ST_SpacialPartitionBucket*)malloc(sizeof(ST_SpacialPartitionBucket));
	spacialPartitionContainer.buckets[0].centroid = gVector3Zero;
	spacialPartitionContainer.buckets[0].containerIndex = 0;
	spacialPartitionContainer.buckets[0].planeColliderIndices = sphereTraceIntListConstruct();
	spacialPartitionContainer.buckets[0].sphereColliderIndices = sphereTraceIntListConstruct();
	spacialPartitionContainer.count = 1;
	spacialPartitionContainer.capacity = 1;
	return spacialPartitionContainer;
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
				float centroidX = -partitionSize * 0.5f + (x - SPACIAL_PARTITION_STATIC_DIMENSION / 2 + 1) * partitionSize;
				float centroidY = -partitionSize * 0.5f + (y - SPACIAL_PARTITION_STATIC_DIMENSION / 2 + 1) * partitionSize;
				float centroidZ = -partitionSize * 0.5f + (z - SPACIAL_PARTITION_STATIC_DIMENSION / 2 + 1) * partitionSize;
				ST_SpacialPartitionBucket bucket;
				bucket.centroid = sphereTraceVector3Construct(centroidX, centroidY, centroidZ);
				bucket.containerIndex = z * SPACIAL_PARTITION_STATIC_DIMENSION * SPACIAL_PARTITION_STATIC_DIMENSION + y * SPACIAL_PARTITION_STATIC_DIMENSION + x;
				bucket.planeColliderIndices = sphereTraceIntListConstruct();
				bucket.sphereColliderIndices = sphereTraceIntListConstruct();
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
				sphereTraceIntListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(x, y, z)));
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
				int bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(x, y, z));
				sphereTraceIntListAddUnique(intList, bucket);
			}
		}
	}
	int bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->rightTopForwardsTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->leftDownBackTransformedVertex.z));
	sphereTraceIntListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->leftDownBackTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->leftDownBackTransformedVertex.z));
	sphereTraceIntListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->leftDownBackTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z));
	sphereTraceIntListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->rightTopForwardsTransformedVertex.x, aabb->leftDownBackTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z));
	sphereTraceIntListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->rightTopForwardsTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->leftDownBackTransformedVertex.z));
	sphereTraceIntListAddUnique(intList, bucket);
	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->leftDownBackTransformedVertex.x, aabb->rightTopForwardsTransformedVertex.y, aabb->rightTopForwardsTransformedVertex.z));
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
