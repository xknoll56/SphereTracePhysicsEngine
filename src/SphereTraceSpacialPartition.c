#include "SphereTraceSpacialPartition.h"
#include "SphereTraceGlobals.h"

ST_OctTreeNode sphereTraceOctTreeNodeConstruct(ST_AABB aabb, ST_Index depth, ST_OctTreeNode* pParent)
{
	ST_OctTreeNode node;
	node.aabb = aabb;
	node.hasChildren = ST_FALSE;
	node.colliderEntries = sphereTraceIndexListConstruct();
	node.depth = depth;
	node.boundingRadius = sphereTraceAABBGetBoundingRadius(&aabb);
	node.pParant = pParent;
	return node;
}
void sphereTraceOctTreeNodeFree(ST_OctTreeNode* const pNode)
{
	sphereTraceIndexListFree(&pNode->colliderEntries);
	sphereTraceAllocatorFreeOctTreeNode(pNode);
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

b32 sphereTraceOctTreeNodeAABBIntersectionWithChildren(ST_OctTreeNode* const pNode, ST_AABB* paabb, b32 intersections[8])
{
	if (pNode->hasChildren)
	{
		ST_Vector3 minDists = sphereTraceVector3Add(pNode->children[0]->aabb.halfExtents, paabb->halfExtents);
		b32 intersectionsByCoord[6];

		if (sphereTraceAbs(pNode->children[ST_LEFT_DOWN_BACK]->aabb.center.x - paabb->center.x) <= minDists.x)
			intersectionsByCoord[0] = 1;
		else
			intersectionsByCoord[0] = 0;

		if (sphereTraceAbs(pNode->children[ST_RIGHT_DOWN_BACK]->aabb.center.x - paabb->center.x) <= minDists.x)
			intersectionsByCoord[1] = 1;
		else
			intersectionsByCoord[1] = 0;

		if (sphereTraceAbs(pNode->children[ST_LEFT_DOWN_BACK]->aabb.center.y - paabb->center.y) <= minDists.y)
			intersectionsByCoord[2] = 1;
		else
			intersectionsByCoord[2] = 0;

		if (sphereTraceAbs(pNode->children[ST_LEFT_UP_BACK]->aabb.center.y - paabb->center.y) <= minDists.y)
			intersectionsByCoord[3] = 1;
		else
			intersectionsByCoord[3] = 0;

		if (sphereTraceAbs(pNode->children[ST_LEFT_DOWN_BACK]->aabb.center.z - paabb->center.z) <= minDists.z)
			intersectionsByCoord[4] = 1;
		else
			intersectionsByCoord[4] = 0;

		if (sphereTraceAbs(pNode->children[ST_LEFT_DOWN_FORWARD]->aabb.center.z - paabb->center.z) <= minDists.z)
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
		return 1;
	}
	return 0;
}

//same with intersect with children except there are no children yet so more calculations need to be done
void sphereTraceOctTreeNodeAABBIntersectionWithFutureChildren(ST_OctTreeNode* const pNode, ST_AABB* paabb, b32 intersections[8])
{
	ST_Vector3 childHalfExtents = sphereTraceVector3Scale(pNode->aabb.halfExtents, 0.5f);
	ST_Vector3 minDists = sphereTraceVector3Add(childHalfExtents, paabb->halfExtents);
	ST_Vector3 childLefDownBackCenter = sphereTraceVector3Add(pNode->aabb.lowExtent, childHalfExtents);
	b32 intersectionsByCoord[6];

	if (sphereTraceAbs(childLefDownBackCenter.x - paabb->center.x) <= minDists.x)
		intersectionsByCoord[0] = 1;
	else
		intersectionsByCoord[0] = 0;

	if (sphereTraceAbs((childLefDownBackCenter.x+pNode->aabb.halfExtents.x) - paabb->center.x) <= minDists.x)
		intersectionsByCoord[1] = 1;
	else
		intersectionsByCoord[1] = 0;

	if (sphereTraceAbs(childLefDownBackCenter.y - paabb->center.y) <= minDists.y)
		intersectionsByCoord[2] = 1;
	else
		intersectionsByCoord[2] = 0;

	if (sphereTraceAbs((childLefDownBackCenter.y + pNode->aabb.halfExtents.y) - paabb->center.y) <= minDists.y)
		intersectionsByCoord[3] = 1;
	else
		intersectionsByCoord[3] = 0;

	if (sphereTraceAbs(childLefDownBackCenter.z - paabb->center.z) <= minDists.z)
		intersectionsByCoord[4] = 1;
	else
		intersectionsByCoord[4] = 0;

	if (sphereTraceAbs((childLefDownBackCenter.z + pNode->aabb.halfExtents.z) - paabb->center.z) <= minDists.z)
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

b32 sphereTraceOctTreeNodeIsInPerspective(ST_OctTreeNode* const pNode, ST_Vector3 from, ST_Direction dir, float fov, float far)
{
	ST_Vector3 dirFromLineToCenter;
	float distFromLineToCenter;
	float dirDist;
	ST_Vector3 dp;
	float distFromLineToCenterMax;
	dp = sphereTraceVector3Subtract(pNode->aabb.center, from);
	dirDist = sphereTraceVector3Dot(dir.v, dp);
	if (dirDist + pNode->boundingRadius > 0.0f)
	{
		if (dirDist - pNode->boundingRadius<= far)
		{
			distFromLineToCenterMax = sphereTraceAbs(tanf(fov*0.5f) * dirDist);
			dirFromLineToCenter = sphereTraceVector3Normalize(sphereTraceVector3Cross(sphereTraceVector3Cross(dir.v, dp), dir.v));
			distFromLineToCenter = sphereTraceAbs(sphereTraceVector3Dot(dirFromLineToCenter, dp));
			if ((distFromLineToCenter - pNode->boundingRadius) <= distFromLineToCenterMax)
			{
				return ST_TRUE;
			}
		}
	}
	return ST_FALSE;
}


void sphereTraceOctTreeNodeReLeafColliders(ST_OctTreeNode* const pNode, ST_Index bucketIndex)
{
	ST_IndexList colliderList = pNode->colliderEntries;
	ST_IndexListData* pild = colliderList.pFirst;
	b32 childIntersections[8];
	ST_Collider* pCollider;
	for (ST_Index i = 0; i < colliderList.count; i++)
	{
		pCollider = pild->value;
		sphereTraceSortedIndexListRemove(&pCollider->pLeafBucketLists[bucketIndex], pNode);
		if (sphereTraceOctTreeNodeAABBIntersectionWithChildren(pNode, &pCollider->aabb, childIntersections))
		{
			for (int i = 0; i < 8; i++)
			{
				if (childIntersections[i])
				{
					sphereTraceSortedIndexListAddUnique(&pNode->children[i]->colliderEntries, pCollider);
					sphereTraceSortedIndexListAddUnique(&pCollider->pLeafBucketLists[bucketIndex], pNode->children[i]);
				}
			}
		}
		pild = pild->pNext;
	}
	sphereTraceIndexListFree(&pNode->colliderEntries);
}




void sphereTraceOctTreeNodePopulateChildren(ST_OctTreeNode* const pNode, ST_Index bucketIndex)
{
	ST_AABB aabb = gAABBOne;
	for (ST_Index i = 0; i < 8; i++)
	{
		pNode->children[i] = sphereTraceAllocatorAllocateOctTreeNode();
		sphereTraceOctTreeNodeSetChildAABBByIndex(pNode, i, &aabb);
		*pNode->children[i] = sphereTraceOctTreeNodeConstruct(aabb, pNode->depth+1, pNode);
	}
	pNode->hasChildren = ST_TRUE;
	sphereTraceOctTreeNodeReLeafColliders(pNode, bucketIndex);
}

//if there is no reduction in the number of objects on a single child
//than i'm going to assume no benfit is had, although there could still
//be a benefit in further releafificaation
b32 sphereTraceOctTreeNodeDoesReLeafProvideBenefit(ST_OctTreeNode* const pNode, ST_Index maxIntersectionCountPerObject)
{
	b32 childIntersections[8];
	ST_IndexListData* pild = pNode->colliderEntries.pFirst;
	ST_Collider* pCollider;
	ST_Index intersectionCount = 0;
	for (int i = 0; i < pNode->colliderEntries.count; i++)
	{
		intersectionCount = 0;
		pCollider = pild->value;
		sphereTraceOctTreeNodeAABBIntersectionWithFutureChildren(pNode, &pCollider->aabb, childIntersections);
		for (int j = 0; j < 8; j++)
		{
			if (childIntersections[j])
			{
				intersectionCount++;
			}
		}
		if (intersectionCount > maxIntersectionCountPerObject)
			return ST_FALSE;

		pild = pild->pNext;
	}
	return ST_TRUE;
}


float sphereTraceOctTreeNodeCalculateVolumePercent(ST_OctTreeNode* const pNode, ST_AABB* const paabb)
{
	ST_AABB intersectionRegion = gAABBOne;
	sphereTraceColliderAABBIntersectAABBIntersectionRegion(&pNode->aabb, paabb, &intersectionRegion);
	return sphereTraceAABBGetVolume(&intersectionRegion) / sphereTraceAABBGetVolume(&pNode->aabb);
}

//if all colliders are above the minimum volume percent, its likely innefficient to keep disecting
b32 sphereTraceOctTreeNodeVolumeRequirements(ST_OctTreeNode* const pNode, float maxVolumePercentOfNode, float minVolumeOfSelf)
{
	ST_IndexListData* pild = pNode->colliderEntries.pFirst;
	ST_Collider* pCollider;
	float commulativeVolume = 0.0f;
	float volumeOfNode = sphereTraceAABBGetVolume(&pNode->aabb);
	float maxVolumeOfSelf = 0.0f;
	for (ST_Index i = 0; i < pNode->colliderEntries.count; i++)
	{
		pCollider = pild->value;
		ST_AABB intersectionRegion = gAABBOne;
		sphereTraceColliderAABBIntersectAABBIntersectionRegion(&pNode->aabb, &pCollider->aabb, &intersectionRegion);
		commulativeVolume += sphereTraceAABBGetVolume(&intersectionRegion);
		float volumeOfSelf = sphereTraceAABBGetVolume(&intersectionRegion) / sphereTraceAABBGetVolume(&pCollider->aabb);
		if (volumeOfSelf > maxVolumeOfSelf)
		{
			maxVolumeOfSelf = volumeOfSelf;
		}
		pild = pild->pNext;
	}
	float volumeRatio = commulativeVolume / volumeOfNode;
	if (volumeRatio >= maxVolumePercentOfNode)
		return ST_FALSE;
	if (maxVolumeOfSelf <= minVolumeOfSelf)
		return ST_FALSE;
	return ST_TRUE;
}


ST_OctTreeNode* sphereTraceOctTreeNodePointIntersectionWithChildren(ST_OctTreeNode* const pNode, ST_Vector3 point)
{

	//we already know its in the aabb
	//if its not its gonna return the nearest octant
	ST_Vector3 mid = pNode->children[ST_LEFT_DOWN_BACK]->aabb.highExtent;
	if (point.x < mid.x)
	{
		if (point.y < mid.y)
		{
			if (point.z < mid.z)
			{
				return pNode->children[ST_LEFT_DOWN_BACK];
			}
			else
			{
				return pNode->children[ST_LEFT_DOWN_FORWARD];
			}
		}
		else
		{
			if (point.z < mid.z)
			{
				return pNode->children[ST_LEFT_UP_BACK];
			}
			else
			{
				return pNode->children[ST_LEFT_UP_FORWARD];
			}
		}
	}
	else
	{
		if (point.y < mid.y)
		{
			if (point.z < mid.z)
			{
				return pNode->children[ST_RIGHT_DOWN_BACK];
			}
			else
			{
				return pNode->children[ST_RIGHT_DOWN_FORWARD];
			}
		}
		else
		{
			if (point.z < mid.z)
			{
				return pNode->children[ST_RIGHT_UP_BACK];
			}
			else
			{
				return pNode->children[ST_RIGHT_UP_FORWARD];
			}
		}
	}

}

ST_Octant sphereTraceOctTreeNodePointToChildOctant(ST_OctTreeNode* const pNode, ST_Vector3 point)
{

	//we already know its in the aabb
	//if its not its gonna return the nearest octant
	ST_Vector3 mid = pNode->children[ST_LEFT_DOWN_BACK]->aabb.highExtent;
	if (point.x < mid.x)
	{
		if (point.y < mid.y)
		{
			if (point.z < mid.z)
			{
				return ST_LEFT_DOWN_BACK;
			}
			else
			{
				return ST_LEFT_DOWN_FORWARD;
			}
		}
		else
		{
			if (point.z < mid.z)
			{
				return ST_LEFT_UP_BACK;
			}
			else
			{
				return ST_LEFT_UP_FORWARD;
			}
		}
	}
	else
	{
		if (point.y < mid.y)
		{
			if (point.z < mid.z)
			{
				return ST_RIGHT_DOWN_BACK;
			}
			else
			{
				return ST_RIGHT_DOWN_FORWARD;
			}
		}
		else
		{
			if (point.z < mid.z)
			{
				return ST_RIGHT_UP_BACK;
			}
			else
			{
				return ST_RIGHT_UP_FORWARD;
			}
		}
	}
}

//this ensures added safety when raytracing so a octant is not skipped
ST_Octant sphereTraceOctTreeNodePointToChildOctantWithDir(ST_OctTreeNode* const pNode, ST_Vector3 point, ST_Direction dir)
{

	//we already know its in the aabb
	//if its not its gonna return the nearest octant
	ST_Vector3 mid = pNode->children[ST_LEFT_DOWN_BACK]->aabb.highExtent;
	b32 onMidX = sphereTraceAbs(point.x - mid.x)< ST_OCT_TREE_SKIN_WIDTH;
	b32 onMidY = sphereTraceAbs(point.y - mid.y)<ST_OCT_TREE_SKIN_WIDTH;
	b32 onMidZ = sphereTraceAbs(point.z - mid.z)<ST_OCT_TREE_SKIN_WIDTH;
	ST_Octant toReturn;
	if (point.x < mid.x)
	{
		if (point.y < mid.y)
		{
			if (point.z < mid.z)
			{
				toReturn= ST_LEFT_DOWN_BACK;
			}
			else
			{
				toReturn = ST_LEFT_DOWN_FORWARD;
			}
		}
		else
		{
			if (point.z < mid.z)
			{
				toReturn = ST_LEFT_UP_BACK;
			}
			else
			{
				toReturn = ST_LEFT_UP_FORWARD;
			}
		}
	}
	else
	{
		if (point.y < mid.y)
		{
			if (point.z < mid.z)
			{
				toReturn = ST_RIGHT_DOWN_BACK;
			}
			else
			{
				toReturn = ST_RIGHT_DOWN_FORWARD;
			}
		}
		else
		{
			if (point.z < mid.z)
			{
				toReturn = ST_RIGHT_UP_BACK;
			}
			else
			{
				toReturn = ST_RIGHT_UP_FORWARD;
			}
		}
	}
	if (onMidX)
	{
		if (dir.v.x >= 0)
		{
			if (!sphereTraceOctantIsOnRightSide(toReturn))
			{
				toReturn = sphereTraceOctantGetNextFromDirection(toReturn, ST_DIRECTION_RIGHT);
			}
		}
		else
		{
			if (sphereTraceOctantIsOnRightSide(toReturn))
			{
				toReturn = sphereTraceOctantGetNextFromDirection(toReturn, ST_DIRECTION_LEFT);
			}
		}
	}

	if (onMidY)
	{
		if (dir.v.y >= 0)
		{
			if (!sphereTraceOctantIsOnUpSide(toReturn))
			{
				toReturn = sphereTraceOctantGetNextFromDirection(toReturn, ST_DIRECTION_UP);
			}
		}
		else
		{
			if (sphereTraceOctantIsOnUpSide(toReturn))
			{
				toReturn = sphereTraceOctantGetNextFromDirection(toReturn, ST_DIRECTION_DOWN);
			}
		}
	}

	if (onMidZ)
	{
		if (dir.v.z >= 0)
		{
			if (!sphereTraceOctantIsOnForwardSide(toReturn))
			{
				toReturn = sphereTraceOctantGetNextFromDirection(toReturn, ST_DIRECTION_FORWARD);
			}
		}
		else
		{
			if (sphereTraceOctantIsOnForwardSide(toReturn))
			{
				toReturn = sphereTraceOctantGetNextFromDirection(toReturn, ST_DIRECTION_BACK);
			}
		}
	}
	return toReturn;
}

ST_Index sphereTraceOctTreeCalculateMaxDepth(ST_OctTree* pTree)
{
	float minDim = sphereTraceMin(sphereTraceMin(pTree->root->aabb.halfExtents.x, pTree->root->aabb.halfExtents.y), pTree->root->aabb.halfExtents.z);
	ST_Index maxDepth = 1;
	while (minDim >= ST_OCT_TREE_SMALLEST_OCTANT_SIZE)
	{
		maxDepth++;
		minDim /= 2.0f;
	}
	if (maxDepth > ST_OCT_TREE_MAX_DEPTH)
		maxDepth = ST_OCT_TREE_MAX_DEPTH;
	return maxDepth;
}

ST_OctTree sphereTraceOctTreeConstruct(ST_AABB aabb, ST_Index gridIndex)
{
	ST_OctTree octTree;
	octTree.depth = 0;
	octTree.root = sphereTraceAllocatorAllocateOctTreeNode();
	*octTree.root = sphereTraceOctTreeNodeConstruct(aabb, 0, NULL);
	octTree.maxDepth = sphereTraceOctTreeCalculateMaxDepth(&octTree);
	octTree.gridIndex = gridIndex;
	return octTree;
}

void sphereTraceOctTreeNodeSampleIntersectionLeafsRecursive(ST_OctTreeNode* const pNode, ST_IndexList* pIntersections, ST_AABB* const paabb)
{
	if (!pNode->hasChildren)
	{
		//were guarenteed not to be adding the same nodes, and sorting them does not matter
		//sphereTraceSortedIndexListAddUnique(pIntersections, pNode);
		sphereTraceIndexListAddFirst(pIntersections, pNode);
	}
	else
	{
		b32 childIntersections[8];
		sphereTraceOctTreeNodeAABBIntersectionWithChildren(pNode, paabb, childIntersections);
		for (int i = 0; i < 8; i++)
		{
			if (childIntersections[i])
			{
				sphereTraceOctTreeNodeSampleIntersectionLeafsRecursive(pNode->children[i], pIntersections, paabb);
			}
		}
	}
}

ST_IndexList sphereTraceOctTreeSampleIntersectionLeafs(ST_OctTree* const pOctTree, ST_AABB* const paabb)
{
	ST_IndexList intersections = sphereTraceIndexListConstruct();
	sphereTraceOctTreeNodeSampleIntersectionLeafsRecursive(pOctTree->root, &intersections, paabb);
	return intersections;
}

void sphereTraceOctTreeNodeReSampleIntersectionLeafsAndCollidersRecursive(ST_OctTreeNode* const pNode, ST_AABB* const paabb, 
	ST_IndexList* const pLeafs, ST_IndexList* const pColliders, b32 sampleDynamicColliders, b32 sampleStaticColliders)
{
	if (!pNode->hasChildren)
	{
		//were guarenteed not to be adding the same nodes, and sorting them does not matter
		//if pointer to index list is null dont bother collecting the leafs
		if(pLeafs!=NULL)
			sphereTraceSortedIndexListAddUnique(pLeafs, pNode);
		ST_IndexListData* pild = pNode->colliderEntries.pFirst;
		ST_Collider* pCollider;
		for (int i = 0; i < pNode->colliderEntries.count; i++)
		{
			pCollider = pild->value;
			if (sphereTraceColliderAABBIntersectAABB(paabb, &pCollider->aabb))
			{
				if((pCollider->isDynamic && sampleDynamicColliders) || (!pCollider->isDynamic && sampleStaticColliders))
					sphereTraceSortedIndexListAddUnique(pColliders, pCollider);
			}
			pild = pild->pNext;
		}
	}
	else
	{
		b32 childIntersections[8];
		sphereTraceOctTreeNodeAABBIntersectionWithChildren(pNode, paabb, childIntersections);
		for (int i = 0; i < 8; i++)
		{
			if (childIntersections[i])
			{
				sphereTraceOctTreeNodeReSampleIntersectionLeafsAndCollidersRecursive(pNode->children[i],  paabb, pLeafs, pColliders, sampleDynamicColliders, sampleStaticColliders);
			}
		}
	}
}

void sphereTraceOctTreeReSampleIntersectionLeafsAndColliders(ST_OctTree* const pOctTree, ST_AABB* const paabb, ST_IndexList* const pLeafs, 
	ST_IndexList* const pColliders, b32 sampleDynamicColliders, b32 sampleStaticColliders)
{
	sphereTraceOctTreeNodeReSampleIntersectionLeafsAndCollidersRecursive(pOctTree->root, paabb, pLeafs, pColliders, sampleDynamicColliders, sampleStaticColliders);
}

void sphereTraceOctTreeNodeSampleIntersectionLeafsAndCollidersFromPerspectiveRecursive(ST_OctTreeNode* const pNode, ST_Vector3 from, ST_Direction dir, float fov, float far, ST_IndexList* const pLeafs, ST_IndexList* const pColliders)
{
	if (!pNode->hasChildren)
	{
		//were guarenteed not to be adding the same nodes, and sorting them does not matter
		sphereTraceIndexListAddFirst(pLeafs, pNode);

		//if null is passed in for colliders than skip
		if (pColliders != NULL)
		{
			sphereTraceSortedIndexListMergeUnique(&pNode->colliderEntries, pColliders);
		}
	}
	else
	{
		for (int i = 0; i < 8; i++)
		{
			if (sphereTraceOctTreeNodeIsInPerspective(pNode->children[i], from, dir, fov, far))
			{
				sphereTraceOctTreeNodeSampleIntersectionLeafsAndCollidersFromPerspectiveRecursive(pNode->children[i], from, dir, fov, far, pLeafs, pColliders);
			}
		}
	}
}

//leafs will not be sorted
void sphereTraceOctTreeSampleIntersectionLeafsAndCollidersFromPerspective(ST_OctTree* const pOctTree, ST_Vector3 from, ST_Direction dir, float fov, float f, ST_IndexList* const pLeafs, ST_IndexList* const pColliders)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	sphereTraceOctTreeNodeSampleIntersectionLeafsAndCollidersFromPerspectiveRecursive(pOctTree->root, from, dir, fov, f, pLeafs, pColliders);
}


//ST_IndexList sphereTraceOctTreeSampleIntersectionColliders(ST_OctTree* const pOctTree, ST_Collider* const pCollider)
//{
//	ST_IndexList toReturn = sphereTraceIndexListConstruct();
//	ST_IndexList nodeList = sphereTraceOctTreeSampleIntersectionLeafs(pOctTree, &pCollider->aabb);
//	ST_IndexListData* pNodeListData = nodeList.pFirst;
//	ST_IndexListData* pColliderListData;
//	ST_OctTreeNode* pNode;
//	ST_Collider* pOtherCollider;
//	ST_AABB intersectionAABB;
//	for (int i = 0; i < nodeList.count; i++)
//	{
//		ST_OctTreeNode* pNode = pNodeListData->value;
//		pColliderListData = pNode->objectIds.pFirst;
//		for (int j = 0; j < pNode->objectIds.count; j++)
//		{
//			pOtherCollider = pColliderListData->value;
//			if (sphereTraceColliderAABBIntersectAABB(&pCollider->aabb, &pOtherCollider->aabb))
//			{
//				sphereTraceSortedIndexListAddUnique(&toReturn, pOtherCollider);
//			}
//			pColliderListData = pColliderListData->pNext;
//		}
//		pNodeListData = pNodeListData->pNext;
//	}
//	sphereTraceIndexListFree(&nodeList);
//	return toReturn;
//}



ST_OctTreeNode* sphereTraceOctTreeNodeSampleLeafNodeRecursive(ST_OctTreeNode* pNode, ST_Vector3 point)
{
	//if the node has a child, simply move on to the child node
	if (pNode->hasChildren)
	{
		sphereTraceOctTreeNodeSampleLeafNodeRecursive(sphereTraceOctTreeNodePointIntersectionWithChildren(pNode, point), point);
	}
	else
	{
		return pNode;
	}
}

ST_OctTreeNode* sphereTraceOctTreeSampleLeafNode(ST_OctTree* pTree, ST_Vector3 point)
{
	if (sphereTraceColliderAABBContainsPoint(pTree->root, point))
	{
		return sphereTraceOctTreeNodeSampleLeafNodeRecursive(pTree->root, point);
	}
	return NULL;
}

void sphereTraceOctTreeNodeInsertColliderRecursive(ST_OctTreeNode* const pNode, ST_Collider* const pCollider, ST_Index maxDepth, b32 hasBeenAdded, b32 restructureTree, ST_Index bucketIndex)
{
	if (pNode->hasChildren)
	{
		b32 childIntersections[8];
		if (sphereTraceOctTreeNodeAABBIntersectionWithChildren(pNode, &pCollider->aabb, childIntersections))
		{
			for (int i = 0; i < 8; i++)
			{
				if (childIntersections[i])
				{
					sphereTraceOctTreeNodeInsertColliderRecursive(pNode->children[i], pCollider, maxDepth, hasBeenAdded, restructureTree, bucketIndex);
				}
			}
		}
	}
	else
	{
		//if it hasn't been added add it, then let the children disect recursively 
		if (!hasBeenAdded)
		{
			sphereTraceSortedIndexListAddUnique(&pNode->colliderEntries, pCollider);
			hasBeenAdded = ST_TRUE;
		}
		if (restructureTree && pNode->colliderEntries.count > 1 && pNode->depth < maxDepth
			&& sphereTraceOctTreeNodeVolumeRequirements(pNode, 0.80f, 0.20f))
		{
			sphereTraceOctTreeNodePopulateChildren(pNode, bucketIndex);
			sphereTraceOctTreeNodeInsertColliderRecursive(pNode, pCollider, maxDepth, hasBeenAdded, restructureTree, bucketIndex);
		}
		else
		{
			//its a leaf node, so add it to the object entry
			sphereTraceSortedIndexListAddUnique(&pCollider->pLeafBucketLists[bucketIndex], pNode);
		}
	}
}


void sphereTraceOctTreeInsertCollider(ST_OctTree* pTree, ST_Collider* pCollider, b32 restructureTree)
{
	sphereTraceOctTreeNodeInsertColliderRecursive(pTree->root, pCollider, pTree->maxDepth, ST_FALSE, restructureTree, pTree->gridIndex);
}

ST_Index sphereTraceOctTreeNodeCountUniqueCollidersBelowRecursive(ST_OctTreeNode* const pNode, ST_IndexList* pUniqueColliders, ST_IndexList* pNodesBelow)
{
	sphereTraceSortedIndexListAddUnique(pNodesBelow, pNode);
	if (!pNode->hasChildren)
	{
		sphereTraceSortedIndexListMergeUnique(&pNode->colliderEntries, pUniqueColliders);
		//ST_IndexListData* pild = pNode->objectEntries.pFirst;
		//for (int i = 0; i < pNode->objectEntries.count; i++)
		//{
		//	sphereTraceSortedIndexListAddUnique(pUniqueObjectIds, pild->value);
		//	pild = pild->pNext;
		//}
	}
	else
	{
		for (int i = 0; i < 8; i++)
		{
			sphereTraceOctTreeNodeCountUniqueCollidersBelowRecursive(pNode->children[i], pUniqueColliders, pNodesBelow);
		}
	}
}

ST_Index sphereTraceOctTreeNodeGetColliderCountBelow(ST_OctTreeNode* const pNode, ST_IndexList* pUniqueColliders, ST_IndexList* pNodesBelow)
{
	if (pNode->hasChildren)
	{
		for (int i = 0; i < 8; i++)
		{
			sphereTraceOctTreeNodeCountUniqueCollidersBelowRecursive(pNode->children[i], pUniqueColliders, pNodesBelow);
		}
		return pUniqueColliders->count;
	}
	else
		return pNode->colliderEntries.count;
}

void sphereTraceOctTreeNodeCollapseUpwardUntilMinimumColliders(ST_OctTreeNode* pNode, ST_Index minObjects, ST_IndexList* pCollapsedNodes, ST_IndexList* pUniqueColliders)
{
	ST_IndexList uniqueCollidersCheck = sphereTraceIndexListConstruct();
	ST_IndexList collapsedNodesCheck = sphereTraceIndexListConstruct();

	ST_IndexListData* pild;
	ST_Index count = sphereTraceOctTreeNodeGetColliderCountBelow(pNode, &uniqueCollidersCheck, &collapsedNodesCheck);
	while (count <= minObjects && pNode != NULL)
	{
		sphereTraceSortedIndexListMergeUnique(&uniqueCollidersCheck, pUniqueColliders);
		sphereTraceSortedIndexListMergeUnique(&collapsedNodesCheck, pCollapsedNodes);
		sphereTraceIndexListReset(&uniqueCollidersCheck);
		sphereTraceIndexListReset(&collapsedNodesCheck);
		pNode->hasChildren = ST_FALSE;
		pNode = pNode->pParant;
		if(pNode)
			count = sphereTraceOctTreeNodeGetColliderCountBelow(pNode, &uniqueCollidersCheck, &collapsedNodesCheck);
	}
	sphereTraceIndexListFree(&uniqueCollidersCheck);
	sphereTraceIndexListFree(&collapsedNodesCheck);
}


//removes the object but will not restructure the oct tree
void sphereTraceOctTreeRemoveCollider(ST_OctTree* pTree, ST_Collider* pCollider, b32 restructureTree)
{
	ST_OctTreeNode* pNode;
	ST_IndexListData* pild = pCollider->pLeafBucketLists[pTree->gridIndex].pFirst;
	for (ST_Index i = 0; i < pCollider->pLeafBucketLists[pTree->gridIndex].count; i++)
	{
		pNode = pild->value;
		sphereTraceSortedIndexListRemove(&pNode->colliderEntries, pCollider);
		pild = pild->pNext;
	}

	if (restructureTree)
	{
		ST_IndexList collapsedNodes = sphereTraceIndexListConstruct();
		ST_IndexList uniqueColliders = sphereTraceIndexListConstruct();
		pild = pCollider->pLeafBucketLists[pTree->gridIndex].pFirst;
		for (ST_Index i = 0; i < pCollider->pLeafBucketLists[pTree->gridIndex].count; i++)
		{
			pNode = pild->value;
			if (!sphereTraceSortedIndexListContains(&collapsedNodes, pNode))
			{
				sphereTraceOctTreeNodeCollapseUpwardUntilMinimumColliders(pNode, 1, &collapsedNodes, &uniqueColliders);
			}
			pild = pild->pNext;
		}


		//releaf all remaining objects
		pild = uniqueColliders.pFirst;
		ST_Collider* pAffectedCollider;
		ST_IndexListData* pild2;
		for (ST_Index i = 0; i < uniqueColliders.count; i++)
		{
			pAffectedCollider = pild->value;
			sphereTraceIndexListFree(&pAffectedCollider->pLeafBucketLists[pTree->gridIndex]);
			pAffectedCollider->pLeafBucketLists[pTree->gridIndex] = sphereTraceOctTreeSampleIntersectionLeafs(pTree, &pAffectedCollider->aabb);
			pild2 = pAffectedCollider->pLeafBucketLists[pTree->gridIndex].pFirst;
			for (ST_Index j = 0; j < pAffectedCollider->pLeafBucketLists[pTree->gridIndex].count; j++)
			{
				pNode = pild2->value;
				sphereTraceIndexListAddUnique(&pNode->colliderEntries, pAffectedCollider);
				pild2 = pild2->pNext;
			}
			pild = pild->pNext;
		}
		sphereTraceIndexListFree(&uniqueColliders);

		//free all collapsed nodes
		pild = collapsedNodes.pFirst;;
		for (ST_Index i = 0; i < collapsedNodes.count; i++)
		{
			pNode = pild->value;
			sphereTraceOctTreeNodeFree(pNode);
			pild = pild->pNext;
		}
		sphereTraceIndexListFree(&collapsedNodes);
	}
	sphereTraceIndexListFree(&pCollider->pLeafBucketLists[pTree->gridIndex]);
	//sphereTraceAllocatorFreeOctTreeObjectEntry(pObjectEntry);
}



void sphereTraceOctTreeReInsertCollider(ST_OctTree* pTree, ST_Collider* pCollider, b32 restructureTree)
{
	sphereTraceOctTreeRemoveCollider(pTree, pCollider, restructureTree);
	sphereTraceOctTreeInsertCollider(pTree, pCollider, restructureTree);
}

//b32 sphereTraceOctTreeNodeRayTraceRecursive(ST_Octant childOctant, ST_Vector3 from, ST_Direction dir, float maxDist, const ST_OctTreeNode* const pNode, ST_RayTraceData* const pRayTraceData)
//{
//	if (pNode->hasChildren)
//	{
//		//ST_Octant childOctant = sphereTraceOctTreeNodePointToChildOctant(pNode, from);
//		//if (sphereTraceOctTreeNodeRayTraceRecursive(from, dir, maxDist, pNode->children[childOctant], pRayTraceData))
//		//	return 1;
//		ST_RayTraceData traceOutRTD;
//		ST_IndexList checkList = sphereTraceIndexListConstruct();
//		while (childOctant != ST_OCTANT_NONE)
//		{
//			if (sphereTraceColliderAABBRayTraceOut(from, dir, &pNode->children[childOctant]->aabb, &traceOutRTD))
//			{
//				childOctant = sphereTraceOctantGetNextFromDirection(childOctant, traceOutRTD.directionType);
//				if (childOctant != ST_OCTANT_NONE && traceOutRTD.distance <= maxDist)
//					sphereTraceIndexListAddLast(&checkList, childOctant);
//			}
//		}
//		ST_IndexListData* pild = checkList.pFirst;
//		for (int i = 0; i < checkList.count; i++)
//		{
//
//			pild = pild->pNext;
//		}
//	}
//	else
//	{
//		if (sphereTraceColliderListRayTrace(pRayTraceData->startPoint, dir, &pNode->colliderEntries, pRayTraceData))
//			return 1;
//	}
//}

b32 sphereTraceOctTreeNodeRayTraceRecursive(ST_Vector3 from, ST_Direction dir, float distTravelled, float maxDist, const ST_OctTreeNode* const pNode, ST_RayTraceData* const pRayTraceData)
{
	if (pNode->hasChildren)
	{
		ST_Octant childOctant = sphereTraceOctTreeNodePointToChildOctantWithDir(pNode, from, dir);
		if (sphereTraceOctTreeNodeRayTraceRecursive(from, dir, distTravelled, maxDist, pNode->children[childOctant], pRayTraceData))
			return 1;
		ST_RayTraceData traceOutRTD;
		while (childOctant !=ST_OCTANT_NONE)
		{
			if (sphereTraceColliderAABBRayTraceOut(from, dir, &pNode->children[childOctant]->aabb, &traceOutRTD))
			{
				childOctant = sphereTraceOctantGetNextFromDirection(childOctant, traceOutRTD.directionType);
				if (childOctant == ST_OCTANT_NONE)
					return 0;
				distTravelled += traceOutRTD.distance;
				if (distTravelled > maxDist)
					return 0;
				from = traceOutRTD.contact.point;
				if (sphereTraceOctTreeNodeRayTraceRecursive(from, dir, distTravelled, maxDist, pNode->children[childOctant], pRayTraceData))
					return 1;

			}
		}
	}
	else
	{
		if (sphereTraceColliderListRayTrace(pRayTraceData->startPoint, dir, &pNode->colliderEntries, pRayTraceData))
			return 1;
	}
}


b32 sphereTraceOctTreeRayTrace(ST_Vector3 from, ST_Direction dir, float maxDist, const ST_OctTree* const pTree, ST_RayTraceData* const pRayTraceData)
{

	if (sphereTraceColliderAABBRayTrace(from, dir, &pTree->root->aabb, pRayTraceData))
	{
		if (pTree->root->hasChildren)
		{
			if (pRayTraceData->distance <= maxDist)
			{
				pRayTraceData->startPoint = from;
				return sphereTraceOctTreeNodeRayTraceRecursive(pRayTraceData->contact.point, dir, pRayTraceData->distance, maxDist, pTree->root, pRayTraceData);
			}
			else
			{
				return 0;
			}
		}
		else
		{
			return sphereTraceColliderListRayTrace(from, dir, &pTree->root->colliderEntries, pRayTraceData);
		}
	}
	return 0;

}

//ST_Index sphereTraceOctTreeGetMaxCollidersOnLeaf(ST_OctTree* pTree, ST_OctTreeNode** ppNodeWithMax)
//{
//	ST_IndexList nodeList = sphereTraceOctTreeSampleIntersectionLeafs(pTree, &pTree->root->aabb);
//	ST_OctTreeNode* pNode;
//	ST_IndexListData* pild;
//	pild = nodeList.pFirst;
//	ST_Index maxColliders = 0;
//	for (int j = 0; j < nodeList.count; j++)
//	{
//		pNode = pild->value;
//		if (pNode->objectIds.count > maxColliders)
//		{
//			maxColliders = pNode->objectIds.count;
//			*ppNodeWithMax = pNode;
//		}
//		pild = pild->pNext;
//	}
//	sphereTraceIndexListFree(&nodeList);
//	return maxColliders;
//}

//void sphereTraceOctTreeDisectNodesWithMinColliders(ST_OctTree* pTree, ST_Index minColliders)
//{
//	ST_IndexList nodeList = sphereTraceOctTreeSampleIntersectionLeafs(pTree, &pTree->root->aabb);
//	ST_OctTreeNode* pNode;
//	ST_IndexListData* pild;
//	pild = nodeList.pFirst;
//	for (int j = 0; j < nodeList.count; j++)
//	{
//		pNode = pild->value;
//		if (pNode->colliderList.count >= minColliders)
//		{
//			if(pNode->depth<pTree->maxDepth)
//				sphereTraceOctTreeNodePopulateChildren(pNode);
//		}
//		pild = pild->pNext;
//	}
//	sphereTraceIndexListFree(&nodeList);
//}
//
//void sphereTraceOctTreeDisectNodesUntilIneffective(ST_OctTree* pTree, ST_Index maxIterations)
//{
//	ST_OctTreeNode* pNode;
//	ST_Index maxCollidersOnLeaf = sphereTraceOctTreeGetMaxCollidersOnLeaf(pTree, &pNode);
//	ST_Index newMaxCollidersOnLeaf = maxCollidersOnLeaf;
//	ST_Index iters = 0;
//	do
//	{
//		maxCollidersOnLeaf = newMaxCollidersOnLeaf;
//		sphereTraceOctTreeDisectNodesWithMinColliders(pTree, maxCollidersOnLeaf);
//		newMaxCollidersOnLeaf = sphereTraceOctTreeGetMaxCollidersOnLeaf(pTree, &pNode);
//		iters++;
//	} 
//	while (newMaxCollidersOnLeaf < maxCollidersOnLeaf || iters < maxIterations);
//}

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


//ST_SpacialPartitionStaticContainer sphereTraceSpacialPartitionStaticConstruct(float partitionSize)
//{
//	ST_SpacialPartitionStaticContainer spacialPartitionContainer;
//	spacialPartitionContainer.partitionSize = partitionSize;
//	spacialPartitionContainer.buckets = malloc(SPACIAL_PARTITION_STATIC_SIZE * sizeof(ST_SpacialPartitionStaticContainer));
//	for (ST_Index z = 0; z < SPACIAL_PARTITION_STATIC_DIMENSION; z++)
//	{
//		for (ST_Index y = 0; y < SPACIAL_PARTITION_STATIC_DIMENSION; y++)
//		{
//			for (ST_Index x = 0; x < SPACIAL_PARTITION_STATIC_DIMENSION; x++)
//			{
//				float centroidX = -partitionSize * 0.5f + ((float)x - SPACIAL_PARTITION_STATIC_DIMENSION / 2 + 1) * partitionSize;
//				float centroidY = -partitionSize * 0.5f + ((float)y - SPACIAL_PARTITION_STATIC_DIMENSION / 2 + 1) * partitionSize;
//				float centroidZ = -partitionSize * 0.5f + ((float)z - SPACIAL_PARTITION_STATIC_DIMENSION / 2 + 1) * partitionSize;
//				ST_SpacialPartitionBucket bucket;
//				bucket.centroid = sphereTraceVector3Construct(centroidX, centroidY, centroidZ);
//				bucket.containerIndex = z * SPACIAL_PARTITION_STATIC_DIMENSION * SPACIAL_PARTITION_STATIC_DIMENSION + y * SPACIAL_PARTITION_STATIC_DIMENSION + x;
//				bucket.planeColliderIndices = sphereTraceIndexListConstruct();
//				bucket.triangleColliderIndices = sphereTraceIndexListConstruct();
//				bucket.sphereColliderIndices = sphereTraceIndexListConstruct();
//				bucket.uniformTerrainColliderIndices = sphereTraceIndexListConstruct();
//				bucket.aabb.halfExtents = sphereTraceVector3Construct(partitionSize * 0.5f, partitionSize * 0.5f, partitionSize * 0.5f);
//				bucket.aabb.lowExtent = sphereTraceVector3Subtract(bucket.centroid, bucket.aabb.halfExtents);
//				bucket.aabb.highExtent = sphereTraceVector3Add(bucket.centroid, bucket.aabb.halfExtents);
//				spacialPartitionContainer.buckets[bucket.containerIndex] = bucket;
//			}
//		}
//	}
//	spacialPartitionContainer.aabb.halfExtents = sphereTraceVector3Construct((SPACIAL_PARTITION_STATIC_DIMENSION / 2) * partitionSize, (SPACIAL_PARTITION_STATIC_DIMENSION / 2) * partitionSize, (SPACIAL_PARTITION_STATIC_DIMENSION / 2) * partitionSize);
//	spacialPartitionContainer.aabb.lowExtent = sphereTraceVector3Add(spacialPartitionContainer.buckets[0].centroid, sphereTraceVector3Construct(-partitionSize * 0.5f, -partitionSize * 0.5f, -partitionSize * 0.5f));
//	spacialPartitionContainer.aabb.highExtent = sphereTraceVector3Add(spacialPartitionContainer.buckets[SPACIAL_PARTITION_STATIC_SIZE - 1].centroid, sphereTraceVector3Construct(partitionSize * 0.5f, partitionSize * 0.5f, partitionSize * 0.5f));
//	spacialPartitionContainer.count = SPACIAL_PARTITION_STATIC_SIZE;
//	spacialPartitionContainer.capacity = SPACIAL_PARTITION_STATIC_SIZE;
//	spacialPartitionContainer.outsideBucket.planeColliderIndices = sphereTraceIndexListConstruct();
//	spacialPartitionContainer.outsideBucket.triangleColliderIndices = sphereTraceIndexListConstruct();
//	spacialPartitionContainer.outsideBucket.sphereColliderIndices = sphereTraceIndexListConstruct();
//	spacialPartitionContainer.outsideBucket.uniformTerrainColliderIndices = sphereTraceIndexListConstruct();
//	return spacialPartitionContainer;
//}

ST_OctTreeGrid sphereTraceOctTreeGridConstruct(ST_AABB worldaabb, ST_Vector3 bucketHalfExtents)
{
	ST_OctTreeGrid octTreeGrid;
	octTreeGrid.bucketHalfExtents = bucketHalfExtents;
	octTreeGrid.minDim = sphereTraceMin(bucketHalfExtents.x, sphereTraceMin(bucketHalfExtents.y, bucketHalfExtents.z));
	octTreeGrid.xBuckets = (worldaabb.halfExtents.x / bucketHalfExtents.x);
	if (octTreeGrid.xBuckets * bucketHalfExtents.x > worldaabb.halfExtents.x || octTreeGrid.xBuckets == 0)
		octTreeGrid.xBuckets++;
	octTreeGrid.yBuckets = (worldaabb.halfExtents.y / bucketHalfExtents.y);
	if (octTreeGrid.yBuckets * bucketHalfExtents.y > worldaabb.halfExtents.y || octTreeGrid.yBuckets == 0)
		octTreeGrid.yBuckets++;
	octTreeGrid.zBuckets = (worldaabb.halfExtents.z / bucketHalfExtents.z);
	if (octTreeGrid.zBuckets * bucketHalfExtents.z > worldaabb.halfExtents.z || octTreeGrid.zBuckets == 0)
		octTreeGrid.zBuckets++;
	octTreeGrid.worldaabb = sphereTraceAABBConstruct1(worldaabb.lowExtent,
		sphereTraceVector3Add(worldaabb.lowExtent, sphereTraceVector3Construct(octTreeGrid.xBuckets * (2.0f * bucketHalfExtents.x),
			octTreeGrid.yBuckets * (2.0f * bucketHalfExtents.y), octTreeGrid.zBuckets * (2.0f * bucketHalfExtents.z))));
	octTreeGrid.outsideColliders = sphereTraceIndexListConstruct();
	octTreeGrid.capacity = octTreeGrid.xBuckets * octTreeGrid.yBuckets * octTreeGrid.zBuckets;
	octTreeGrid.treeBuckets = (ST_OctTree*)malloc(sizeof(ST_OctTree) * octTreeGrid.capacity);
	for (ST_Index z = 0; z < octTreeGrid.zBuckets; z++)
	{
		for (ST_Index y = 0; y < octTreeGrid.yBuckets; y++)
		{
			for (ST_Index x = 0; x < octTreeGrid.xBuckets; x++)
			{
				ST_Index i = z * octTreeGrid.xBuckets * octTreeGrid.yBuckets + y * octTreeGrid.xBuckets + x;
				ST_Vector3 low = sphereTraceVector3Add(worldaabb.lowExtent,sphereTraceVector3Construct(x * 2.0f * bucketHalfExtents.x,
					y * 2.0f * bucketHalfExtents.y, z * 2.0f * bucketHalfExtents.z));
				ST_Vector3 high = sphereTraceVector3Add(worldaabb.lowExtent, sphereTraceVector3Construct((x+1) * 2.0f * bucketHalfExtents.x,
					(y+1) * 2.0f * bucketHalfExtents.y, (z+1) * 2.0f * bucketHalfExtents.z));
				octTreeGrid.treeBuckets[i] = sphereTraceOctTreeConstruct(sphereTraceAABBConstruct1(low, high), i);
				//octTreeGrid.treeBuckets[i].gridIndex = i;
			}
		}
	}
	return octTreeGrid;
}

ST_Index sphereTraceOctTreeGridGetBucketIndexFromPosition(const ST_OctTreeGrid* const pOctTreeGrid, ST_Vector3 position)
{
	if (sphereTraceColliderAABBContainsPoint(&pOctTreeGrid->worldaabb, position))
	{
		ST_Index z = (ST_Index)((position.z - pOctTreeGrid->worldaabb.lowExtent.z) / (2.0f*pOctTreeGrid->bucketHalfExtents.z));
		ST_Index y = (ST_Index)((position.y - pOctTreeGrid->worldaabb.lowExtent.y) / (2.0f * pOctTreeGrid->bucketHalfExtents.y));
		ST_Index x = (ST_Index)((position.x - pOctTreeGrid->worldaabb.lowExtent.x) / (2.0f * pOctTreeGrid->bucketHalfExtents.x));
		ST_Index index = z * pOctTreeGrid->xBuckets*pOctTreeGrid->yBuckets + y * pOctTreeGrid->xBuckets + x;
		if (index >= 0 && index < pOctTreeGrid->capacity)
			return index;
		else
			return -1;
	}
	else
		return -1;
}

ST_IndexList sphereTraceOctTreeGridGetBucketIndicesFromAABB(const ST_OctTreeGrid* const pOctTreeGrid, const ST_AABB* const aabb)
{
	ST_IndexList intList = sphereTraceIndexListConstruct();
	float xExtent = aabb->highExtent.x;
	if (xExtent > pOctTreeGrid->worldaabb.highExtent.x)
	{
		xExtent = pOctTreeGrid->worldaabb.highExtent.x;
		sphereTraceSortedIndexListAddUnique(&intList, -1);
	}
	float yExtent = aabb->highExtent.y;
	if (yExtent > pOctTreeGrid->worldaabb.highExtent.y)
	{
		yExtent = pOctTreeGrid->worldaabb.highExtent.y;
		sphereTraceSortedIndexListAddUnique(&intList, -1);
	}
	float zExtent = aabb->highExtent.z;
	if (zExtent > pOctTreeGrid->worldaabb.highExtent.z)
	{
		zExtent = pOctTreeGrid->worldaabb.highExtent.z;
		sphereTraceSortedIndexListAddUnique(&intList, -1);
	}
	ST_Vector3 start = aabb->lowExtent;
	float xIncrement = sphereTraceMin(aabb->halfExtents.x, 2.0f*pOctTreeGrid->bucketHalfExtents.x);
	if (xIncrement == 0.0f)
		xIncrement = 2.0f * pOctTreeGrid->bucketHalfExtents.x;
	if (start.x < pOctTreeGrid->worldaabb.lowExtent.x)
	{
		start.x = pOctTreeGrid->worldaabb.lowExtent.x;
		sphereTraceSortedIndexListAddUnique(&intList, -1);
	}
	float yIncrement = sphereTraceMin(aabb->halfExtents.y, 2.0f * pOctTreeGrid->bucketHalfExtents.y);
	if (yIncrement == 0.0f)
		yIncrement = 2.0f * pOctTreeGrid->bucketHalfExtents.y;
	if (start.y < pOctTreeGrid->worldaabb.lowExtent.y)
	{
		start.y = pOctTreeGrid->worldaabb.lowExtent.y;
		sphereTraceSortedIndexListAddUnique(&intList, -1);
	}
	float zIncrement = sphereTraceMin(aabb->halfExtents.z, 2.0f * pOctTreeGrid->bucketHalfExtents.z);
	if (zIncrement == 0.0f)
		zIncrement = 2.0f * pOctTreeGrid->bucketHalfExtents.z;
	if (start.z < pOctTreeGrid->worldaabb.lowExtent.z)
	{
		start.z = pOctTreeGrid->worldaabb.lowExtent.z;
		sphereTraceSortedIndexListAddUnique(&intList, -1);
	}

	for (float x = start.x; x <= xExtent; x += xIncrement)
	{
		for (float y = start.y; y <= yExtent; y += yIncrement)
		{
			for (float z = start.z; z <= zExtent; z += zIncrement)
			{
				sphereTraceSortedIndexListAddUnique(&intList, sphereTraceOctTreeGridGetBucketIndexFromPosition(pOctTreeGrid, sphereTraceVector3Construct(x, y, z)));
			}
		}
	}
	return intList;
}

//this can be used if an aabb is certain to fit within a grid tree
ST_IndexList sphereTraceOctTreeGridGetBucketIndicesFromAABBCorners(const ST_OctTreeGrid* const pOctTreeGrid, const ST_AABB* const aabb)
{
	ST_IndexList intList = sphereTraceIndexListConstruct();
	sphereTraceSortedIndexListAddUnique(&intList, sphereTraceOctTreeGridGetBucketIndexFromPosition(pOctTreeGrid, aabb->lowExtent));
	sphereTraceSortedIndexListAddUnique(&intList, sphereTraceOctTreeGridGetBucketIndexFromPosition(pOctTreeGrid,
		sphereTraceColliderAABBGetExtentByOctant(aabb, ST_RIGHT_DOWN_BACK)));
	sphereTraceSortedIndexListAddUnique(&intList, sphereTraceOctTreeGridGetBucketIndexFromPosition(pOctTreeGrid,
		sphereTraceColliderAABBGetExtentByOctant(aabb, ST_LEFT_DOWN_FORWARD)));
	sphereTraceSortedIndexListAddUnique(&intList, sphereTraceOctTreeGridGetBucketIndexFromPosition(pOctTreeGrid,
		sphereTraceColliderAABBGetExtentByOctant(aabb, ST_RIGHT_DOWN_FORWARD)));
	sphereTraceSortedIndexListAddUnique(&intList, sphereTraceOctTreeGridGetBucketIndexFromPosition(pOctTreeGrid,
		sphereTraceColliderAABBGetExtentByOctant(aabb, ST_LEFT_UP_BACK)));
	sphereTraceSortedIndexListAddUnique(&intList, sphereTraceOctTreeGridGetBucketIndexFromPosition(pOctTreeGrid,
		sphereTraceColliderAABBGetExtentByOctant(aabb, ST_RIGHT_UP_BACK)));
	sphereTraceSortedIndexListAddUnique(&intList, sphereTraceOctTreeGridGetBucketIndexFromPosition(pOctTreeGrid,
		sphereTraceColliderAABBGetExtentByOctant(aabb, ST_LEFT_UP_FORWARD)));
	sphereTraceSortedIndexListAddUnique(&intList, sphereTraceOctTreeGridGetBucketIndexFromPosition(pOctTreeGrid, aabb->highExtent));
	return intList;
}

ST_Index sphereTraceOctTreeGridGetNextIndexByDirection(const ST_OctTreeGrid* const pGrid, ST_Index curIndex, ST_DirectionType dir)
{
	switch (dir)
	{
	case ST_DIRECTION_RIGHT:
	{
		ST_Index xInd = curIndex % pGrid->xBuckets;
		if (xInd < pGrid->xBuckets - 1)
			return curIndex+1;
		else
			return -1;
	}
		break;
	case ST_DIRECTION_LEFT:
	{
		ST_Index xInd = curIndex % pGrid->xBuckets;
		if (xInd > 0)
			return curIndex-1;
		else
			return -1;
	}
		break;
	case ST_DIRECTION_UP:
	{
		ST_Index yInd = (curIndex % (pGrid->xBuckets * pGrid->yBuckets))/pGrid->xBuckets;
		if (yInd < pGrid->yBuckets - 1)
			return curIndex+=pGrid->xBuckets;
		else
			return -1;
	}
		break;
	case ST_DIRECTION_DOWN:
	{
		ST_Index yInd = (curIndex % (pGrid->xBuckets * pGrid->yBuckets)) / pGrid->xBuckets;
		if (yInd > 0)
			return curIndex -= pGrid->xBuckets;
		else
			return -1;
	}
		break;
	case ST_DIRECTION_FORWARD:
	{
		ST_Index zInd = curIndex / (pGrid->xBuckets * pGrid->yBuckets);
		if (zInd < pGrid->zBuckets - 1)
			return curIndex += pGrid->xBuckets * pGrid->yBuckets;
		else
			return -1;
	}
		break;
	case ST_DIRECTION_BACK:
	{
		ST_Index zInd = curIndex / (pGrid->xBuckets * pGrid->yBuckets);
		if (zInd >0)
			return curIndex -= pGrid->xBuckets * pGrid->yBuckets;
		else
			return -1;
	}
		break;

	}
}

ST_IndexList sphereTraceOctTreeGridGetBucketIndicesFromRayTrace(ST_Vector3 start, ST_Direction dir, float maxDist, const ST_OctTreeGrid* const pGrid, ST_RayTraceData* const pData)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	ST_IndexList indices = sphereTraceIndexListConstruct();
	ST_RayTraceData rtd;
	ST_Vector3 curStart;
	ST_Index breakCond = (ST_Index)-1;
	if (sphereTraceColliderAABBRayTrace(start, dir, &pGrid->worldaabb, pData))
	{
		ST_Index curIndex;
		if(pData->distance==0.0f)
			curIndex = sphereTraceOctTreeGridGetBucketIndexFromPosition(pGrid, pData->contact.point);
		else
		{
			curIndex = sphereTraceOctTreeGridGetBucketIndexFromPosition(pGrid, 
				sphereTraceVector3AddAndScale(pData->contact.point, pData->contact.normal.v, -pGrid->minDim));
		}
		curStart = pData->contact.point;
		float cumDist = pData->distance;
		if (pData->distance > maxDist)
			return indices;
		while (curIndex != breakCond)
		{
			sphereTraceIndexListAddLast(&indices, curIndex);
			if (sphereTraceColliderAABBRayTraceOut(curStart, dir, &pGrid->treeBuckets[curIndex].root->aabb, &rtd))
			{
				curIndex = sphereTraceOctTreeGridGetNextIndexByDirection(pGrid, curIndex, rtd.directionType);
				curStart = rtd.contact.point;
				cumDist += rtd.distance;
				if (cumDist > maxDist)
					return indices;
			}
			else
				return indices;
		}
		sphereTraceIndexListAddLast(&indices, -1);
	}
	return indices;
}



//ST_SpacialPartitionBucket sphereTraceSpacialPartitionGetBucketWithIndex(ST_SpacialPartitionStaticContainer* const pSpacialPartitionStaticContainer, ST_Index bucketIndex)
//{
//	ST_SpacialPartitionBucket bucket;
//	if (bucketIndex == -1)
//	{
//		bucket = pSpacialPartitionStaticContainer->outsideBucket;
//	}
//	else
//	{
//		bucket = pSpacialPartitionStaticContainer->buckets[bucketIndex];
//	}
//	return bucket;
//}
//
//ST_Index sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, ST_Vector3 position)
//{
//	if (sphereTraceColliderAABBContainsPoint(&pSpacialPartitionContainer->aabb, position))
//	{
//		ST_Index z = (ST_Index)((position.z + pSpacialPartitionContainer->aabb.halfExtents.z) / pSpacialPartitionContainer->partitionSize);
//		ST_Index y = (ST_Index)((position.y + pSpacialPartitionContainer->aabb.halfExtents.y) / pSpacialPartitionContainer->partitionSize);
//		ST_Index x = (ST_Index)((position.x + pSpacialPartitionContainer->aabb.halfExtents.x) / pSpacialPartitionContainer->partitionSize);
//		ST_Index index = z * SPACIAL_PARTITION_STATIC_DIMENSION * SPACIAL_PARTITION_STATIC_DIMENSION + y * SPACIAL_PARTITION_STATIC_DIMENSION + x;
//		if (index >= 0 && index < pSpacialPartitionContainer->count)
//			return index;
//		else
//			return -1;
//	}
//	else
//		return -1;
//}
//
//ST_IndexList sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, const ST_AABB* const aabb)
//{
//	ST_IndexList intList = sphereTraceIndexListConstruct();
//	float xExtent = sphereTraceMin(aabb->highExtent.x, pSpacialPartitionContainer->aabb.highExtent.x);
//	float yExtent = sphereTraceMin(aabb->highExtent.y, pSpacialPartitionContainer->aabb.highExtent.y);
//	float zExtent = sphereTraceMin(aabb->highExtent.z, pSpacialPartitionContainer->aabb.highExtent.z);
//	ST_Vector3 start = aabb->lowExtent;
//	float xIncrement = sphereTraceMin(aabb->halfExtents.x, pSpacialPartitionContainer->partitionSize);
//	if (xIncrement == 0.0f)
//		xIncrement = pSpacialPartitionContainer->partitionSize;
//	if (start.x < pSpacialPartitionContainer->aabb.lowExtent.x)
//	{
//		start.x = pSpacialPartitionContainer->aabb.lowExtent.x;
//	}
//	float yIncrement = sphereTraceMin(aabb->halfExtents.y, pSpacialPartitionContainer->partitionSize);
//	if (yIncrement == 0.0f)
//		yIncrement = pSpacialPartitionContainer->partitionSize;
//	if ( start.y < pSpacialPartitionContainer->aabb.lowExtent.y)
//	{
//		start.y = pSpacialPartitionContainer->aabb.lowExtent.y;
//	}
//	float zIncrement = sphereTraceMin(aabb->halfExtents.z, pSpacialPartitionContainer->partitionSize);
//	if (zIncrement == 0.0f)
//		zIncrement = pSpacialPartitionContainer->partitionSize;
//	if (start.z < pSpacialPartitionContainer->aabb.lowExtent.z)
//	{
//		start.z = pSpacialPartitionContainer->aabb.lowExtent.z;
//	}
//
//	for (float x = start.x; x <= xExtent; x += xIncrement)
//	{
//		for (float y = start.y; y <= yExtent; y += yIncrement)
//		{
//			for (float z = start.z; z <= zExtent; z += zIncrement)
//			{
//				sphereTraceIndexListAddUnique(&intList, sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(x, y, z)));
//			}
//		}
//	}
//	return intList;
//}
//
//ST_IndexList sphereTraceSpacialPartitionStaticUpdateBucketIndicesFromAABBAndReturnDeletedBucketIndices(const ST_SpacialPartitionStaticContainer* const pSpacialPartitionContainer, const ST_AABB* const aabb, ST_IndexList* const intList)
//{
//	ST_IndexList intListToRemove = sphereTraceIndexListConstruct();
//	for (float x = aabb->lowExtent.x; x <= aabb->highExtent.x; x += pSpacialPartitionContainer->partitionSize)
//	{
//		for (float y = aabb->lowExtent.y; y <= aabb->highExtent.y; y += pSpacialPartitionContainer->partitionSize)
//		{
//			for (float z = aabb->lowExtent.z; z <= aabb->highExtent.z; z += pSpacialPartitionContainer->partitionSize)
//			{
//				int bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(x, y, z));
//				sphereTraceIndexListAddUnique(intList, bucket);
//			}
//		}
//	}
//	int bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->highExtent.x, aabb->lowExtent.y, aabb->lowExtent.z));
//	sphereTraceIndexListAddUnique(intList, bucket);
//	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->lowExtent.x, aabb->highExtent.y, aabb->lowExtent.z));
//	sphereTraceIndexListAddUnique(intList, bucket);
//	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->lowExtent.x, aabb->lowExtent.y, aabb->highExtent.z));
//	sphereTraceIndexListAddUnique(intList, bucket);
//	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->highExtent.x, aabb->lowExtent.y, aabb->highExtent.z));
//	sphereTraceIndexListAddUnique(intList, bucket);
//	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->highExtent.x, aabb->highExtent.y, aabb->lowExtent.z));
//	sphereTraceIndexListAddUnique(intList, bucket);
//	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, sphereTraceVector3Construct(aabb->lowExtent.x, aabb->highExtent.y, aabb->highExtent.z));
//	sphereTraceIndexListAddUnique(intList, bucket);
//	bucket = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(pSpacialPartitionContainer, aabb->highExtent);
//	sphereTraceIndexListAddUnique(intList, bucket);
//
//	return intListToRemove;
//}
//
//ST_Vector3 sphereTraceSpacialPartitionStaticGetNearestBucketIntersectionFromPositionAndDirection(const ST_SpacialPartitionBucket* const pCurrentBucket, ST_Vector3 start, ST_Vector3 dir, ST_Vector3* const incomingDirection)
//{
//	//ST_Vector3 t = gVector3Max;
//	float t = FLT_MAX;
//	if (dir.x > 0.0f)
//	{
//		float tentative = (pCurrentBucket->aabb.highExtent.x - start.x) / dir.x;
//		if (tentative < t && tentative>0.0f)
//		{
//			t = tentative;
//			*incomingDirection = gVector3Right;
//		}
//	}
//	else if (dir.x < 0.0f)
//	{
//		float tentative = (pCurrentBucket->aabb.lowExtent.x - start.x) / dir.x;
//		if (tentative < t && tentative>0.0f)
//		{
//			t = tentative;
//			*incomingDirection = gVector3Left;
//		}
//	}
//
//	if (dir.y > 0.0f)
//	{
//		float tentative = (pCurrentBucket->aabb.highExtent.y - start.y) / dir.y;
//		if (tentative < t && tentative>0.0f)
//		{
//			t = tentative;
//			*incomingDirection = gVector3Up;
//		}
//	}
//	else if (dir.y < 0.0f)
//	{
//		float tentative = (pCurrentBucket->aabb.lowExtent.y - start.y) / dir.y;
//		if (tentative < t && tentative>0.0f)
//		{
//			t = tentative;
//			*incomingDirection = gVector3Down;
//		}
//	}
//
//	if (dir.z > 0.0f)
//	{
//		float tentative = (pCurrentBucket->aabb.highExtent.z - start.z) / dir.z;
//		if (tentative < t && tentative>0.0f)
//		{
//			t = tentative;
//			*incomingDirection = gVector3Forward;
//		}
//	}
//	else if (dir.z < 0.0f)
//	{
//		float tentative = (pCurrentBucket->aabb.lowExtent.z - start.z) / dir.z;
//		if (tentative < t && tentative>0.0f)
//		{
//			t = tentative;
//			*incomingDirection = gVector3Back;
//		}
//	}
//
//	//float tMin = sphereTraceMin(sphereTraceMin(t.x, t.y), t.z) + FLT_MIN;
//	ST_Vector3 intersection = sphereTraceVector3AddAndScale(start, dir, t);
//	return intersection;
//}

void sphereTraceOctTreeGridInsertCollider(ST_OctTreeGrid* const pGrid, ST_Collider* pCollider, b32 restructureTrees)
{
	ST_IndexList indices = sphereTraceOctTreeGridGetBucketIndicesFromAABB(pGrid, &pCollider->aabb);
	ST_IndexListData* pild = indices.pFirst;
	for (int i = 0; i < indices.count; i++)
	{
		if (pild->value != (ST_Index)-1)
			sphereTraceOctTreeInsertCollider(&pGrid->treeBuckets[pild->value], pCollider, restructureTrees);
		else
			sphereTraceSortedIndexListAddUnique(&pGrid->outsideColliders, pCollider);
		pild = pild->pNext;
	}
	pCollider->bucketIndices = indices;
}

void sphereTraceOctTreeGridRemoveCollider(ST_OctTreeGrid* const pGrid, ST_Collider* pCollider, b32 restructureTrees)
{
	ST_IndexListData* pild = pCollider->bucketIndices.pFirst;
	for (int i = 0; i < pCollider->bucketIndices.count; i++)
	{
		if (pild->value == (ST_Index)-1)
			sphereTraceSortedIndexListRemove(&pGrid->outsideColliders, pCollider);
		else
			sphereTraceOctTreeRemoveCollider(&pGrid->treeBuckets[pild->value], pCollider, restructureTrees);
		pild = pild->pNext;
	}
	sphereTraceIndexListFree(&pCollider->bucketIndices);
}

void sphereTraceOctTreeGridReInsertCollider(ST_OctTreeGrid* const pGrid, ST_Collider* pCollider, b32 restructureTrees)
{
	sphereTraceOctTreeGridRemoveCollider(pGrid, pCollider, restructureTrees);
	sphereTraceOctTreeGridInsertCollider(pGrid, pCollider, restructureTrees);
}

void sphereTraceOctTreeGridReSampleIntersectionLeafsAndColliders(ST_OctTreeGrid* const pGrid, ST_AABB* const paabb,
	ST_IndexList* const pLeafs, ST_IndexList* const pColliders, b32 sampleDynamicColliders, b32 sampleStaticColliders)
{
	ST_IndexList indices = sphereTraceOctTreeGridGetBucketIndicesFromAABB(pGrid, paabb);
	ST_IndexListData* pild = indices.pFirst;
	for (int i = 0; i < indices.count; i++)
	{
		if (pild->value != (ST_Index)-1)
			sphereTraceOctTreeReSampleIntersectionLeafsAndColliders(&pGrid->treeBuckets[pild->value], paabb, pLeafs,
				pColliders, sampleDynamicColliders, sampleStaticColliders);
		else
			sphereTraceSortedIndexListMergeUnique(&pGrid->outsideColliders, pColliders);
		pild = pild->pNext;
	}
	sphereTraceIndexListFree(&indices);
}

//quick hacky way with bounding box
void sphereTraceOctTreeGridSampleIntersectionLeafsAndCollidersFromPerspective(ST_OctTreeGrid* const pGrid, ST_Vector3 from,
	ST_Direction dir, float fov, float f, ST_IndexList* const pLeafs, ST_IndexList* const pColliders)
{
	float width = sphereTraceAbs(tanf(fov * 0.5f) * f);
	ST_AABB sampleaabb = sphereTraceAABBConstruct2(sphereTraceVector3AddAndScale(from, dir.v, f * 0.5f),
		sphereTraceVector3Construct(width, width, width));
	ST_IndexList indices = sphereTraceOctTreeGridGetBucketIndicesFromAABB(pGrid, &sampleaabb);
	ST_IndexListData* pild = indices.pFirst;
	for (int i = 0; i < indices.count; i++)
	{
		if (pild->value != (ST_Index)-1)
			sphereTraceOctTreeSampleIntersectionLeafsAndCollidersFromPerspective(&pGrid->treeBuckets[pild->value], from, dir,
				fov, f, pLeafs, pColliders);
		else
			sphereTraceSortedIndexListMergeUnique(&pGrid->outsideColliders, pColliders);
		pild = pild->pNext;
	}
	sphereTraceIndexListFree(&indices);
}