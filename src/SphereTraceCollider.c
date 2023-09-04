#include "SphereTraceCollider.h"
#include "SphereTraceColliderPlane.h"
#include "SphereTraceGlobals.h"

ST_Edge sphereTraceEdgeConstruct(ST_Vector3 p1, ST_Vector3 p2)
{
	ST_Edge edge;
	edge.point1 = p1;
	edge.point2 = p2;
	edge.dir = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(p2, p1));
	edge.dist = sphereTraceVector3Distance(p1, p2);
	return edge;
}


b32 sphereTraceColliderAABBIsPointInside(const ST_AABB* const aabb, ST_Vector3 point)
{
	if (point.x >= aabb->leftDownBackTransformedVertex.x && point.x <= aabb->rightTopForwardsTransformedVertex.x)
	{
		if (point.y >= aabb->leftDownBackTransformedVertex.y && point.y <= aabb->rightTopForwardsTransformedVertex.y)
		{
			if (point.z >= aabb->leftDownBackTransformedVertex.z && point.z <= aabb->rightTopForwardsTransformedVertex.z)
			{
				return 1;
			}
		}
	}
	return 0;
}

void sphereTraceColliderAABBSetHalfExtents(ST_AABB* const aabb)
{
	aabb->halfExtents = sphereTraceVector3Subtract(aabb->rightTopForwardsTransformedVertex, sphereTraceVector3Average(aabb->leftDownBackTransformedVertex, aabb->rightTopForwardsTransformedVertex));
}

void sphereTraceColliderAABBResizeAABBToContainAnotherAABB(ST_AABB* const aabbToResize, const ST_AABB* const aabbToContain)
{
	b32 resizeDidHappen = 0;
	if (aabbToContain->leftDownBackTransformedVertex.x < aabbToResize->leftDownBackTransformedVertex.x)
	{
		aabbToResize->leftDownBackTransformedVertex.x = aabbToContain->leftDownBackTransformedVertex.x;
		resizeDidHappen = 1;
	}
	if (aabbToContain->leftDownBackTransformedVertex.y < aabbToResize->leftDownBackTransformedVertex.y)
	{
		aabbToResize->leftDownBackTransformedVertex.y = aabbToContain->leftDownBackTransformedVertex.y;
		resizeDidHappen = 1;
	}
	if (aabbToContain->leftDownBackTransformedVertex.z < aabbToResize->leftDownBackTransformedVertex.z)
	{
		aabbToResize->leftDownBackTransformedVertex.z = aabbToContain->leftDownBackTransformedVertex.z;
		resizeDidHappen = 1;
	}
	if (aabbToContain->rightTopForwardsTransformedVertex.x > aabbToResize->rightTopForwardsTransformedVertex.x)
	{
		aabbToResize->rightTopForwardsTransformedVertex.x = aabbToContain->rightTopForwardsTransformedVertex.x;
		resizeDidHappen = 1;
	}
	if (aabbToContain->rightTopForwardsTransformedVertex.y > aabbToResize->rightTopForwardsTransformedVertex.y)
	{
		aabbToResize->rightTopForwardsTransformedVertex.y = aabbToContain->rightTopForwardsTransformedVertex.y;
		resizeDidHappen = 1;
	}
	if (aabbToContain->rightTopForwardsTransformedVertex.z > aabbToResize->rightTopForwardsTransformedVertex.z)
	{
		aabbToResize->rightTopForwardsTransformedVertex.z = aabbToContain->rightTopForwardsTransformedVertex.z;
		resizeDidHappen = 1;
	}
	if (resizeDidHappen)
	{
		sphereTraceColliderAABBSetHalfExtents(aabbToResize);
	}
}

b32 sphereTraceColliderAABBIntersectAABB(const ST_AABB* const aabb1, const ST_AABB* const aabb2)
{
	if ((aabb1->rightTopForwardsTransformedVertex.x >= aabb2->rightTopForwardsTransformedVertex.x && aabb1->leftDownBackTransformedVertex.x <= aabb2->rightTopForwardsTransformedVertex.x)
		|| (aabb1->rightTopForwardsTransformedVertex.x >= aabb2->leftDownBackTransformedVertex.x && aabb1->leftDownBackTransformedVertex.x <= aabb2->leftDownBackTransformedVertex.x)
		|| (aabb2->rightTopForwardsTransformedVertex.x >= aabb1->rightTopForwardsTransformedVertex.x && aabb2->leftDownBackTransformedVertex.x <= aabb1->rightTopForwardsTransformedVertex.x)
		|| (aabb2->rightTopForwardsTransformedVertex.x >= aabb1->leftDownBackTransformedVertex.x && aabb2->leftDownBackTransformedVertex.x <= aabb1->leftDownBackTransformedVertex.x))
	{
		if ((aabb1->rightTopForwardsTransformedVertex.y >= aabb2->rightTopForwardsTransformedVertex.y && aabb1->leftDownBackTransformedVertex.y <= aabb2->rightTopForwardsTransformedVertex.y)
			|| (aabb1->rightTopForwardsTransformedVertex.y >= aabb2->leftDownBackTransformedVertex.y && aabb1->leftDownBackTransformedVertex.y <= aabb2->leftDownBackTransformedVertex.y)
			|| (aabb2->rightTopForwardsTransformedVertex.y >= aabb1->rightTopForwardsTransformedVertex.y && aabb2->leftDownBackTransformedVertex.y <= aabb1->rightTopForwardsTransformedVertex.y)
			|| (aabb2->rightTopForwardsTransformedVertex.y >= aabb1->leftDownBackTransformedVertex.y && aabb2->leftDownBackTransformedVertex.y <= aabb1->leftDownBackTransformedVertex.y))
		{
			if ((aabb1->rightTopForwardsTransformedVertex.z >= aabb2->rightTopForwardsTransformedVertex.z && aabb1->leftDownBackTransformedVertex.z <= aabb2->rightTopForwardsTransformedVertex.z)
				|| (aabb1->rightTopForwardsTransformedVertex.z >= aabb2->leftDownBackTransformedVertex.z && aabb1->leftDownBackTransformedVertex.z <= aabb2->leftDownBackTransformedVertex.z)
				|| (aabb2->rightTopForwardsTransformedVertex.z >= aabb1->rightTopForwardsTransformedVertex.z && aabb2->leftDownBackTransformedVertex.z <= aabb1->rightTopForwardsTransformedVertex.z)
				|| (aabb2->rightTopForwardsTransformedVertex.z >= aabb1->leftDownBackTransformedVertex.z && aabb2->leftDownBackTransformedVertex.z <= aabb1->leftDownBackTransformedVertex.z))
			{
				return 1;
			}
		}
	}

	return 0;
}

b32 sphereTraceColliderAABBIntersectAABBHorizontally(const ST_AABB* const aabb1, const ST_AABB* const aabb2)
{
	if ((aabb1->rightTopForwardsTransformedVertex.x >= aabb2->rightTopForwardsTransformedVertex.x && aabb1->leftDownBackTransformedVertex.x <= aabb2->rightTopForwardsTransformedVertex.x)
		|| (aabb1->rightTopForwardsTransformedVertex.x >= aabb2->leftDownBackTransformedVertex.x && aabb1->leftDownBackTransformedVertex.x <= aabb2->leftDownBackTransformedVertex.x)
		|| (aabb2->rightTopForwardsTransformedVertex.x >= aabb1->rightTopForwardsTransformedVertex.x && aabb2->leftDownBackTransformedVertex.x <= aabb1->rightTopForwardsTransformedVertex.x)
		|| (aabb2->rightTopForwardsTransformedVertex.x >= aabb1->leftDownBackTransformedVertex.x && aabb2->leftDownBackTransformedVertex.x <= aabb1->leftDownBackTransformedVertex.x))
	{

		if ((aabb1->rightTopForwardsTransformedVertex.z >= aabb2->rightTopForwardsTransformedVertex.z && aabb1->leftDownBackTransformedVertex.z <= aabb2->rightTopForwardsTransformedVertex.z)
			|| (aabb1->rightTopForwardsTransformedVertex.z >= aabb2->leftDownBackTransformedVertex.z && aabb1->leftDownBackTransformedVertex.z <= aabb2->leftDownBackTransformedVertex.z)
			|| (aabb2->rightTopForwardsTransformedVertex.z >= aabb1->rightTopForwardsTransformedVertex.z && aabb2->leftDownBackTransformedVertex.z <= aabb1->rightTopForwardsTransformedVertex.z)
			|| (aabb2->rightTopForwardsTransformedVertex.z >= aabb1->leftDownBackTransformedVertex.z && aabb2->leftDownBackTransformedVertex.z <= aabb1->leftDownBackTransformedVertex.z))
		{
			return 1;
		}
	}

	return 0;
}

b32 sphereTraceColliderAABBIntersectAABBVertically(const ST_AABB* const aabb1, const ST_AABB* const aabb2)
{
	if ((aabb1->rightTopForwardsTransformedVertex.y >= aabb2->rightTopForwardsTransformedVertex.y && aabb1->leftDownBackTransformedVertex.y <= aabb2->rightTopForwardsTransformedVertex.y)
		|| (aabb1->rightTopForwardsTransformedVertex.y >= aabb2->leftDownBackTransformedVertex.y && aabb1->leftDownBackTransformedVertex.y <= aabb2->leftDownBackTransformedVertex.y)
		|| (aabb2->rightTopForwardsTransformedVertex.y >= aabb1->rightTopForwardsTransformedVertex.y && aabb2->leftDownBackTransformedVertex.y <= aabb1->rightTopForwardsTransformedVertex.y)
		|| (aabb2->rightTopForwardsTransformedVertex.y >= aabb1->leftDownBackTransformedVertex.y && aabb2->leftDownBackTransformedVertex.y <= aabb1->leftDownBackTransformedVertex.y))
	{
		return 1;
	}

	return 0;
}

ST_Vector3 sphereTraceColliderAABBMidPoint(const ST_AABB* const aabb)
{
	return sphereTraceVector3Average(aabb->leftDownBackTransformedVertex, aabb->rightTopForwardsTransformedVertex);
}



void sphereTraceColliderResizeAABBWithSpherecast(const ST_SphereTraceData* const pSphereCastData, ST_AABB* const aabb)
{
	ST_Vector3 dp = sphereTraceVector3Subtract(pSphereCastData->sphereCenter, pSphereCastData->rayTraceData.startPoint);

	if (dp.x >= 0.0f)
	{
		if (dp.y >= 0.0f)
		{
			if (dp.z >= 0.0f)
			{
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale(pSphereCastData->rayTraceData.startPoint, gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale(pSphereCastData->sphereCenter, gVector3One, pSphereCastData->radius);
			}
			else
			{
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale(sphereTraceVector3Construct(pSphereCastData->rayTraceData.startPoint.x,
					pSphereCastData->rayTraceData.startPoint.y, pSphereCastData->sphereCenter.z), gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale(sphereTraceVector3Construct(pSphereCastData->sphereCenter.x,
					pSphereCastData->sphereCenter.y, pSphereCastData->rayTraceData.startPoint.z), gVector3One, pSphereCastData->radius);
			}
		}
		else
		{
			if (dp.z >= 0.0f)
			{
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale(sphereTraceVector3Construct(pSphereCastData->rayTraceData.startPoint.x,
					pSphereCastData->sphereCenter.y, pSphereCastData->rayTraceData.startPoint.z), gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale(sphereTraceVector3Construct(pSphereCastData->sphereCenter.x,
					pSphereCastData->rayTraceData.startPoint.y, pSphereCastData->sphereCenter.z), gVector3One, pSphereCastData->radius);
			}
			else
			{
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale(sphereTraceVector3Construct(pSphereCastData->rayTraceData.startPoint.x,
					pSphereCastData->sphereCenter.y, pSphereCastData->sphereCenter.z), gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale(sphereTraceVector3Construct(pSphereCastData->sphereCenter.x,
					pSphereCastData->rayTraceData.startPoint.y, pSphereCastData->rayTraceData.startPoint.z), gVector3One, pSphereCastData->radius);

			}
		}
	}
	else
	{
		if (dp.y >= 0.0f)
		{
			if (dp.z >= 0.0f)
			{
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale(sphereTraceVector3Construct(pSphereCastData->sphereCenter.x,
					pSphereCastData->rayTraceData.startPoint.y, pSphereCastData->rayTraceData.startPoint.z), gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale(sphereTraceVector3Construct(pSphereCastData->rayTraceData.startPoint.x,
					pSphereCastData->sphereCenter.y, pSphereCastData->sphereCenter.z), gVector3One, pSphereCastData->radius);
			}
			else
			{
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale(sphereTraceVector3Construct(pSphereCastData->sphereCenter.x,
					pSphereCastData->rayTraceData.startPoint.y, pSphereCastData->sphereCenter.z), gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale(sphereTraceVector3Construct(pSphereCastData->rayTraceData.startPoint.x,
					pSphereCastData->sphereCenter.y, pSphereCastData->rayTraceData.startPoint.z), gVector3One, pSphereCastData->radius);
			}
		}
		else
		{
			if (dp.z >= 0.0f)
			{
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale(sphereTraceVector3Construct(pSphereCastData->sphereCenter.x,
					pSphereCastData->sphereCenter.y, pSphereCastData->rayTraceData.startPoint.z), gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale(sphereTraceVector3Construct(pSphereCastData->rayTraceData.startPoint.x,
					pSphereCastData->rayTraceData.startPoint.y, pSphereCastData->sphereCenter.z), gVector3One, pSphereCastData->radius);
			}
			else
			{
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale(pSphereCastData->sphereCenter, gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale(pSphereCastData->rayTraceData.startPoint, gVector3One, pSphereCastData->radius);
			}
		}
	}

	aabb->halfExtents = sphereTraceVector3Subtract(aabb->rightTopForwardsTransformedVertex, sphereTraceVector3Average(aabb->rightTopForwardsTransformedVertex, aabb->leftDownBackTransformedVertex));
}



b32 sphereTraceColliderPointSphereTrace(ST_Vector3 from, ST_Direction dir, float radius, ST_Vector3 point, ST_SphereTraceData* const pSphereTraceData)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	ST_Vector3 dp = sphereTraceVector3Subtract(point, from);
	float fwdDist = sphereTraceVector3Dot(dp, dir.v);
	ST_Vector3 dpFwd = sphereTraceVector3Scale(dir.v, fwdDist);
	ST_Vector3 dpRight = sphereTraceVector3Subtract(dp, dpFwd);
	float rightDist = sphereTraceVector3Length(dpRight);
	if (fpclassify(rightDist) == FP_NAN)
	{
		pSphereTraceData->rayTraceData.startPoint = from;
		pSphereTraceData->rayTraceData.hitPoint = point;
		pSphereTraceData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pSphereTraceData->rayTraceData.startPoint, pSphereTraceData->rayTraceData.hitPoint));
		pSphereTraceData->radius = radius;
		pSphereTraceData->sphereCenter = sphereTraceVector3AddAndScale(point, dir.v, -radius);
		pSphereTraceData->rayTraceData.normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(pSphereTraceData->sphereCenter, pSphereTraceData->rayTraceData.hitPoint));
		pSphereTraceData->traceDistance = sphereTraceVector3Distance(pSphereTraceData->sphereCenter, pSphereTraceData->rayTraceData.startPoint);
		return 1;
	}
	else if (rightDist <= radius)
	{
		pSphereTraceData->rayTraceData.startPoint = from;
		pSphereTraceData->rayTraceData.hitPoint = point;
		pSphereTraceData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pSphereTraceData->rayTraceData.startPoint, pSphereTraceData->rayTraceData.hitPoint));
		pSphereTraceData->radius = radius;
		float beta = acosf(rightDist / radius);
		float opposite = sinf(beta) * radius;
		pSphereTraceData->sphereCenter = sphereTraceVector3AddAndScale(sphereTraceVector3AddAndScale(point, dpRight, -1.0f), dir.v, -opposite);
		pSphereTraceData->rayTraceData.normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(pSphereTraceData->sphereCenter, pSphereTraceData->rayTraceData.hitPoint));
		pSphereTraceData->traceDistance = sphereTraceVector3Distance(pSphereTraceData->sphereCenter, pSphereTraceData->rayTraceData.startPoint);
		return 1;
	}
	return 0;
}

b32 sphereTraceColliderEdgeSphereTrace(ST_Vector3 from, ST_Direction dir, float radius, ST_Edge* const pEdge, ST_SphereTraceData* const pSphereTraceData)
{
	ST_Contact contact;
	//dir = sphereTraceVector3Normalize(dir);
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	if (sphereTraceColliderEdgeImposedSphereCollisionTest(pEdge, from, radius, &contact))
	{
		pSphereTraceData->radius = radius;
		pSphereTraceData->rayTraceData.startPoint = from;
		pSphereTraceData->sphereCenter = from;
		pSphereTraceData->rayTraceData.hitPoint = contact.point;
		pSphereTraceData->rayTraceData.normal = contact.normal;
		pSphereTraceData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(from, contact.point));
		pSphereTraceData->traceDistance = 0.0f;
		return 1;
	}
	ST_Vector3 edgeDir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pEdge->point2, pEdge->point1));
	float edgeMaxDist = sphereTraceVector3Distance(pEdge->point2, pEdge->point1);
	ST_Vector3 right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir.v, edgeDir));
	//ST_Vector3 sphereDir = sphereTraceVector3Normalize(sphereTraceVector3Cross(right, edgeDir));
	ST_Vector3 sphereDir = sphereTraceVector3Negative(dir.v);
	float d = fabsf(sphereTraceVector3Dot(edgeDir, dir.v));
	ST_Vector3 fwd = sphereTraceVector3Normalize(sphereTraceVector3Cross(right, dir.v));
	if (d > COLLIDER_TOLERANCE)
		sphereDir = sphereTraceVector3Normalize(sphereTraceVector3Cross(right, edgeDir));
	//if (d < COLLIDER_TOLERANCE)
	//{
	//	sphereDir = sphereTraceVector3Negative(dir);
	//	fwd = sphereDir;
	//}
	float theta = acosf(sphereTraceVector3Dot(edgeDir, dir.v));
	float rightDist;
	rightDist = sphereTraceVector3Dot(sphereTraceVector3Subtract(from, pEdge->point1), right);
	if (fabsf(rightDist) < radius)
	{
		float beta = acosf(rightDist / radius);
		float fwdDist = sinf(beta) * radius;
		ST_Vector3 pointOnEdgeClosestToCenterRaycast = sphereTraceClosestPointOnLineBetweenTwoLines(pEdge->point1, edgeDir, from, dir.v);
		if (sphereTraceVector3Dot(dir.v, sphereTraceVector3Subtract(pointOnEdgeClosestToCenterRaycast, from)) < 0.0f)
			return 0;
		ST_Vector3 pointOnPlaneNearOriginClosestToEdge = sphereTraceClosestPointOnLineBetweenTwoLines(from, right, pEdge->point1, edgeDir);
		pointOnPlaneNearOriginClosestToEdge = sphereTraceVector3AddAndScale(pointOnPlaneNearOriginClosestToEdge, fwd, -fwdDist);
		ST_Vector3 pointOnEdgeTracedFrompointOnPlaneNearOriginClosestToEdge = sphereTraceClosestPointOnLineBetweenTwoLines(pEdge->point1, edgeDir, pointOnPlaneNearOriginClosestToEdge, dir.v);
		float distToPointOnEdgeOrthogonalWithSphereDirectionAndEdge = cosf(theta) * sphereTraceVector3Length(sphereTraceVector3Subtract(pointOnEdgeTracedFrompointOnPlaneNearOriginClosestToEdge, pointOnEdgeClosestToCenterRaycast));
		ST_Vector3 intersection = sphereTraceVector3AddAndScale(pointOnEdgeClosestToCenterRaycast, edgeDir, -distToPointOnEdgeOrthogonalWithSphereDirectionAndEdge);

		fwdDist = fabsf(tanf(theta) * sphereTraceVector3Length(sphereTraceVector3Subtract(pointOnEdgeClosestToCenterRaycast, intersection)));
		ST_Vector3 testPoint2 = sphereTraceClosestPointOnLineBetweenTwoLines(intersection, sphereDir, pointOnEdgeClosestToCenterRaycast, dir.v);
		if (sphereTraceVector3Nan(testPoint2))
		{
			fwdDist = sinf(beta) * radius;
			testPoint2 = sphereTraceVector3AddAndScale(intersection, sphereDir, fwdDist);
		}
		float edgeDist = sphereTraceVector3Dot(sphereTraceVector3Subtract(intersection, pEdge->point1), edgeDir);
		if (edgeDist < 0)
		{
			return sphereTraceColliderPointSphereTrace(from, dir, radius, pEdge->point1, pSphereTraceData);
		}
		else if (edgeDist > edgeMaxDist)
		{
			return sphereTraceColliderPointSphereTrace(from, dir, radius, pEdge->point2, pSphereTraceData);
		}
		else 
		{

			pSphereTraceData->rayTraceData.startPoint = from;
			pSphereTraceData->rayTraceData.hitPoint = intersection;
			pSphereTraceData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pSphereTraceData->rayTraceData.startPoint, pSphereTraceData->rayTraceData.hitPoint));
			pSphereTraceData->radius = radius;
			pSphereTraceData->sphereCenter = sphereTraceVector3Add(sphereTraceVector3AddAndScale(pointOnPlaneNearOriginClosestToEdge, right, rightDist), sphereTraceVector3Subtract(testPoint2, pointOnPlaneNearOriginClosestToEdge));
			pSphereTraceData->rayTraceData.normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(pSphereTraceData->sphereCenter, intersection));
			pSphereTraceData->traceDistance = sphereTraceVector3Distance(pSphereTraceData->sphereCenter, pSphereTraceData->rayTraceData.startPoint);
			return 1;
		}
	}
	return 0;

}
