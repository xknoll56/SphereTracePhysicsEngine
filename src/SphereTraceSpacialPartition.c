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


void sphereTraceOctTreeNodeReLeafColliders(ST_OctTreeNode* const pNode)
{
	ST_IndexList colliderList = pNode->colliderEntries;
	ST_IndexListData* pild = colliderList.pFirst;
	b32 childIntersections[8];
	ST_Collider* pCollider;
	for (ST_Index i = 0; i < colliderList.count; i++)
	{
		pCollider = pild->value;
		sphereTraceSortedIndexListRemove(&pCollider->octTreeLeafs, pNode);
		if (sphereTraceOctTreeNodeAABBIntersectionWithChildren(pNode, &pCollider->aabb, childIntersections))
		{
			for (int i = 0; i < 8; i++)
			{
				if (childIntersections[i])
				{
					sphereTraceSortedIndexListAddUnique(&pNode->children[i]->colliderEntries, pCollider);
					sphereTraceSortedIndexListAddUnique(&pCollider->octTreeLeafs, pNode->children[i]);
				}
			}
		}
		pild = pild->pNext;
	}
	sphereTraceIndexListFree(&pNode->colliderEntries);
}




void sphereTraceOctTreeNodePopulateChildren(ST_OctTreeNode* const pNode)
{
	ST_AABB aabb = gAABBOne;
	for (ST_Index i = 0; i < 8; i++)
	{
		pNode->children[i] = sphereTraceAllocatorAllocateOctTreeNode();
		sphereTraceOctTreeNodeSetChildAABBByIndex(pNode, i, &aabb);
		*pNode->children[i] = sphereTraceOctTreeNodeConstruct(aabb, pNode->depth+1, pNode);
	}
	pNode->hasChildren = ST_TRUE;
	sphereTraceOctTreeNodeReLeafColliders(pNode);
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

ST_OctTree sphereTraceOctTreeConstruct(ST_AABB aabb)
{
	ST_OctTree octTree;
	octTree.depth = 0;
	octTree.root = sphereTraceAllocatorAllocateOctTreeNode();
	*octTree.root = sphereTraceOctTreeNodeConstruct(aabb, 0, NULL);
	octTree.maxDepth = sphereTraceOctTreeCalculateMaxDepth(&octTree);
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

void sphereTraceOctTreeNodeInsertColliderRecursive(ST_OctTreeNode* const pNode, ST_Collider* const pCollider, ST_Index maxDepth, b32 hasBeenAdded, b32 restructureTree)
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
					sphereTraceOctTreeNodeInsertColliderRecursive(pNode->children[i], pCollider, maxDepth, hasBeenAdded, restructureTree);
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
			sphereTraceOctTreeNodePopulateChildren(pNode);
			sphereTraceOctTreeNodeInsertColliderRecursive(pNode, pCollider, maxDepth, hasBeenAdded, restructureTree);
		}
		else
		{
			//its a leaf node, so add it to the object entry
			sphereTraceSortedIndexListAddUnique(&pCollider->octTreeLeafs, pNode);
		}
	}
}


void sphereTraceOctTreeInsertCollider(ST_OctTree* pTree, ST_Collider* pCollider, b32 restructureTree)
{
	sphereTraceOctTreeNodeInsertColliderRecursive(pTree->root, pCollider, pTree->maxDepth, ST_FALSE, restructureTree);
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
	ST_IndexListData* pild = pCollider->octTreeLeafs.pFirst;
	for (ST_Index i = 0; i < pCollider->octTreeLeafs.count; i++)
	{
		pNode = pild->value;
		sphereTraceSortedIndexListRemove(&pNode->colliderEntries, pCollider);
		pild = pild->pNext;
	}

	if (restructureTree)
	{
		ST_IndexList collapsedNodes = sphereTraceIndexListConstruct();
		ST_IndexList uniqueColliders = sphereTraceIndexListConstruct();
		pild = pCollider->octTreeLeafs.pFirst;
		for (ST_Index i = 0; i < pCollider->octTreeLeafs.count; i++)
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
			sphereTraceIndexListFree(&pAffectedCollider->octTreeLeafs);
			pAffectedCollider->octTreeLeafs = sphereTraceOctTreeSampleIntersectionLeafs(pTree, &pAffectedCollider->aabb);
			pild2 = pAffectedCollider->octTreeLeafs.pFirst;
			for (ST_Index j = 0; j < pAffectedCollider->octTreeLeafs.count; j++)
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
	sphereTraceIndexListFree(&pCollider->octTreeLeafs);
	//sphereTraceAllocatorFreeOctTreeObjectEntry(pObjectEntry);
}



void sphereTraceOctTreeReInsertCollider(ST_OctTree* pTree, ST_Collider* pCollider, b32 restructureTree)
{
	sphereTraceOctTreeRemoveCollider(pTree, pCollider, restructureTree);
	sphereTraceOctTreeInsertCollider(pTree, pCollider, restructureTree);
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
	if (sphereTraceColliderAABBContainsPoint(&pSpacialPartitionContainer->aabb, position))
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
	float xExtent = sphereTraceMin(aabb->highExtent.x, pSpacialPartitionContainer->aabb.highExtent.x);
	float yExtent = sphereTraceMin(aabb->highExtent.y, pSpacialPartitionContainer->aabb.highExtent.y);
	float zExtent = sphereTraceMin(aabb->highExtent.z, pSpacialPartitionContainer->aabb.highExtent.z);
	ST_Vector3 start = aabb->lowExtent;
	float xIncrement = sphereTraceMin(aabb->halfExtents.x, pSpacialPartitionContainer->partitionSize);
	if (xIncrement == 0.0f)
		xIncrement = pSpacialPartitionContainer->partitionSize;
	if (start.x < pSpacialPartitionContainer->aabb.lowExtent.x)
	{
		start.x = pSpacialPartitionContainer->aabb.lowExtent.x;
	}
	float yIncrement = sphereTraceMin(aabb->halfExtents.y, pSpacialPartitionContainer->partitionSize);
	if (yIncrement == 0.0f)
		yIncrement = pSpacialPartitionContainer->partitionSize;
	if ( start.y < pSpacialPartitionContainer->aabb.lowExtent.y)
	{
		start.y = pSpacialPartitionContainer->aabb.lowExtent.y;
	}
	float zIncrement = sphereTraceMin(aabb->halfExtents.z, pSpacialPartitionContainer->partitionSize);
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

	//float tMin = sphereTraceMin(sphereTraceMin(t.x, t.y), t.z) + FLT_MIN;
	ST_Vector3 intersection = sphereTraceVector3AddAndScale(start, dir, t);
	return intersection;
}
