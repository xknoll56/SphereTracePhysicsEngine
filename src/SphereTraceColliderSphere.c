#include "SphereTraceColliderSphere.h"
#include "SphereTraceGlobals.h"


ST_SphereCollider sphereTraceColliderSphereConstruct(float radius)
{
	ST_SphereCollider sphereCollider;
	sphereCollider.collider = sphereTraceColliderConstruct(COLLIDER_SPHERE, radius);
	sphereCollider.rigidBody = sphereTraceRigidBodyConstruct(1.0f, 1.0f);
	sphereCollider.radius = radius;
	sphereCollider.ignoreCollisions = 0;
	sphereCollider.restingContact = ST_FALSE;
	sphereCollider.prevFrameContacts = sphereTraceIndexListConstruct();
	sphereCollider.collider.aabb.halfExtents = sphereTraceVector3Construct(radius, radius, radius);
	sphereTraceColliderSphereAABBSetTransformedVertices(&sphereCollider);
	return sphereCollider;
}

void sphereTraceColliderSphereSetPosition(ST_SphereCollider* pSphere, ST_Vector3 position)
{
	pSphere->rigidBody.position = position;
	sphereTraceColliderSphereAABBSetTransformedVertices(pSphere);
}

void sphereTraceColliderSphereAABBSetTransformedVertices(ST_SphereCollider* const pSphereCollider)
{
	pSphereCollider->collider.aabb.highExtent = sphereTraceVector3Add(pSphereCollider->rigidBody.position, pSphereCollider->collider.aabb.halfExtents);
	pSphereCollider->collider.aabb.lowExtent = sphereTraceVector3Subtract(pSphereCollider->rigidBody.position, pSphereCollider->collider.aabb.halfExtents);
	pSphereCollider->collider.aabb.center = pSphereCollider->rigidBody.position;
}

//todo, check the closest edge instead
b32 sphereTraceColliderAABBIntersectImposedSphere(const ST_AABB* const aabb, ST_Vector3 imposedPosition, float imposedRadius)
{
	ST_Vector3 dp = sphereTraceVector3Subtract(imposedPosition, sphereTraceColliderAABBMidPoint(aabb));
	if (sphereTraceAbs(sphereTraceVector3Dot(dp, gVector3Right)) > aabb->halfExtents.x + imposedRadius)
		return 0;
	if (sphereTraceAbs(sphereTraceVector3Dot(dp, gVector3Up)) > aabb->halfExtents.y + imposedRadius)
		return 0;
	if (sphereTraceAbs(sphereTraceVector3Dot(dp, gVector3Forward)) > aabb->halfExtents.z + imposedRadius)
		return 0;
	return 1;
}

b32 sphereTraceColliderAABBIntersectSphere(const ST_AABB* const aabb, const ST_SphereCollider* const pSphereCollider)
{
	ST_Vector3 dp = sphereTraceVector3Subtract(pSphereCollider->rigidBody.position, sphereTraceColliderAABBMidPoint(aabb));
	if (sphereTraceAbs(sphereTraceVector3Dot(dp, gVector3Right)) > aabb->halfExtents.x + pSphereCollider->radius)
		return 0;
	if (sphereTraceAbs(sphereTraceVector3Dot(dp, gVector3Up)) > aabb->halfExtents.y + pSphereCollider->radius)
		return 0;
	if (sphereTraceAbs(sphereTraceVector3Dot(dp, gVector3Forward)) > aabb->halfExtents.z + pSphereCollider->radius)
		return 0;
	return 1;
}

b32 sphereTraceColliderInfinitePlaneImposedSphereCollisionTest(ST_Vector3 imposedPosition, float imposedRadius, ST_Direction planeNormal, ST_Vector3 pointOnPlane, ST_SphereContact* const pContact)
{
	ST_RayTraceData rtd;
	sphereTraceColliderInfinitePlaneRayTrace(imposedPosition, planeNormal, planeNormal, pointOnPlane, &rtd);
	if (sphereTraceAbs(rtd.distance) <= imposedRadius)
	{
		pContact->penetrationDistance = imposedRadius - sphereTraceAbs(rtd.distance);
		pContact->point = rtd.contact.point;
		pContact->normal = rtd.contact.normal;
		//if (sphereTraceVector3Dot(sphereTraceVector3Subtract(rtd.hitPoint, imposedPosition), pContact->normal) > 0.0f)
		//{
		//	sphereTraceVector3NegativeByRef(&pContact->normal);
		//}
		return 1;
	}
	return 0;
}


b32 sphereTraceColliderSphereRayTrace(ST_Vector3 start, ST_Direction dir, const ST_SphereCollider* const pSphere, ST_RayTraceData* const pData)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	b32 hits = 0;
	ST_Vector3 p0 = pSphere->rigidBody.position;
	ST_Vector3 negativeDir = sphereTraceVector3Negative(dir.v);
	float dist = sphereTraceVector3Dot(sphereTraceVector3Subtract(p0, start), negativeDir) / sphereTraceVector3Dot(dir.v, negativeDir);

	if (dist >= 0.0f)
	{
		ST_Vector3 intersection = sphereTraceVector3Add(start, sphereTraceVector3Scale(dir.v, dist));
		ST_Vector3 intersectionMinusStart = sphereTraceVector3Subtract(intersection, p0);
		float l2 = sphereTraceVector3Length2(intersectionMinusStart);
		if (l2 <= pSphere->radius * pSphere->radius)
		{
			hits = 1;
			float length = sphereTraceVector3Length(intersectionMinusStart);
			float theta = sphereTraceMin(1.0f, asinf(length / pSphere->radius));
			ST_Vector3 dirComp = sphereTraceVector3Scale(negativeDir, length / tanf(theta));
			if (length == 0.0f)
				dirComp = sphereTraceVector3Scale(negativeDir, pSphere->radius);
			pData->contact.normal = sphereTraceDirectionConstruct(sphereTraceVector3Add(intersectionMinusStart, dirComp), 0);
			pData->contact.point = sphereTraceVector3Add(pData->contact.normal.v, p0);
			//stupid hack
			if (sphereTraceVector3Nan(pData->contact.point))
				return 0;
			pData->startPoint = start;
			pData->distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->contact.point, start));
			sphereTraceDirectionNormalizeIfNotNormalizedByRef(&pData->contact.normal);
		}
	}

	return hits;
}

b32 sphereTraceColliderImposedSphereRayTrace(ST_Vector3 start, ST_Direction dir, ST_Vector3 imposedPosition, float imposedRadius, ST_RayTraceData* const pData)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	b32 hits = 0;
	ST_Vector3 p0 = imposedPosition;
	ST_Vector3 negativeDir = sphereTraceVector3Negative(dir.v);
	float dist = sphereTraceVector3Dot(sphereTraceVector3Subtract(p0, start), negativeDir) / sphereTraceVector3Dot(dir.v, negativeDir);

	if (dist >= 0.0f)
	{
		ST_Vector3 intersection = sphereTraceVector3Add(start, sphereTraceVector3Scale(dir.v, dist));
		ST_Vector3 intersectionMinusStart = sphereTraceVector3Subtract(intersection, p0);
		float length = sphereTraceVector3Length(intersectionMinusStart);
		if (length <= imposedRadius)
		{
			hits = 1;
			
			float theta = asinf(length / imposedRadius);
			ST_Vector3 dirComp = sphereTraceVector3Scale(negativeDir, length / tanf(theta));
			if (length == 0.0f)
				dirComp = sphereTraceVector3Scale(negativeDir, imposedRadius);
			pData->contact.normal = sphereTraceDirectionConstruct(sphereTraceVector3Add(intersectionMinusStart, dirComp), 0);
			pData->contact.point = sphereTraceVector3Add(pData->contact.normal.v, p0);
			//stupid hack
			if (sphereTraceVector3Nan(pData->contact.point))
				return 0;
			pData->startPoint = start;
			pData->distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->contact.point, start));
			sphereTraceDirectionNormalizeIfNotNormalizedByRef(&pData->contact.normal);
		}
	}

	return hits;
}

b32 sphereTraceColliderImposedSphereRayTraceClampLength(ST_Vector3 start, ST_Direction dir, ST_Vector3 imposedPosition, float imposedRadius, ST_RayTraceData* const pData)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	b32 hits = 0;
	ST_Vector3 p0 = imposedPosition;
	ST_Vector3 negativeDir = sphereTraceVector3Negative(dir.v);
	float dist = sphereTraceVector3Dot(sphereTraceVector3Subtract(p0, start), negativeDir) / sphereTraceVector3Dot(dir.v, negativeDir);

	if (dist >= 0.0f)
	{
		ST_Vector3 intersection = sphereTraceVector3Add(start, sphereTraceVector3Scale(dir.v, dist));
		ST_Vector3 intersectionMinusStart = sphereTraceVector3Subtract(intersection, p0);
		float length = sphereTraceVector3Length(intersectionMinusStart);
		if (length > imposedRadius)
			length = imposedRadius;
		hits = 1;

		float theta = asinf(length / imposedRadius);
		ST_Vector3 dirComp = sphereTraceVector3Scale(negativeDir, length / tanf(theta));
		if (length == 0.0f)
			dirComp = sphereTraceVector3Scale(negativeDir, imposedRadius);
		pData->contact.normal = sphereTraceDirectionConstruct(sphereTraceVector3Add(intersectionMinusStart, dirComp), 0);
		pData->contact.point = sphereTraceVector3Add(pData->contact.normal.v, p0);
		//stupid hack
		if (sphereTraceVector3Nan(pData->contact.point))
			return 0;
		pData->startPoint = start;
		pData->distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->contact.point, start));
		sphereTraceDirectionNormalizeIfNotNormalizedByRef(&pData->contact.normal);
	}

	return hits;
}

b32 sphereTraceColliderSphereSphereTrace(ST_Vector3 start, ST_Direction dir, float radius, const ST_SphereCollider* const pSphere, ST_SphereTraceData* const pData)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	ST_Vector3 dp = sphereTraceVector3Subtract(start, pSphere->rigidBody.position);
	float sphereRadiusPlusSphereCastRadius = pSphere->radius + radius;
	float dpDist = sphereTraceVector3Length(dp);
	//ST_Vector3 p0 = sphere->pRigidBody->position;
	if (dpDist <= sphereRadiusPlusSphereCastRadius)
	{
		pData->rayTraceData.startPoint = start;
		pData->radius = radius;
		pData->rayTraceData.contact.normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(dp, 1.0f / dpDist), 1);
		pData->sphereCenter = start;
		pData->rayTraceData.contact.point = sphereTraceVector3AddAndScale(pSphere->rigidBody.position, pData->rayTraceData.contact.normal.v, pSphere->radius);
		pData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->rayTraceData.contact.point, start));
		pData->traceDistance = 0.0f;
		return 1;
	}
	b32 hits = 0;
	ST_Vector3 negativeDir = sphereTraceVector3Negative(dir.v);
	float dist = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphere->rigidBody.position, start), negativeDir) / sphereTraceVector3Dot(dir.v, negativeDir);

	if (dist >= 0.0f)
	{
		ST_Vector3 intersection = sphereTraceVector3Add(start, sphereTraceVector3Scale(dir.v, dist));
		ST_Vector3 intersectionMinusStart = sphereTraceVector3Subtract(intersection, pSphere->rigidBody.position);
		float length = sphereTraceVector3Length(intersectionMinusStart);
		if (length <= sphereRadiusPlusSphereCastRadius)
		{
			hits = 1;
			pData->rayTraceData.startPoint = start;
			pData->radius = radius;
			float theta = acosf(length / sphereRadiusPlusSphereCastRadius);
			//ST_Vector3 contactPointDir =;
			pData->rayTraceData.contact.normal = sphereTraceDirectionConstruct(sphereTraceVector3Add(intersectionMinusStart, sphereTraceVector3Scale(negativeDir, sphereRadiusPlusSphereCastRadius * sinf(theta))), 0);
			pData->sphereCenter = sphereTraceVector3Add(pSphere->rigidBody.position, pData->rayTraceData.contact.normal.v);
			sphereTraceDirectionNormalizeIfNotNormalizedByRef(&pData->rayTraceData.contact.normal);
			pData->rayTraceData.contact.point = sphereTraceVector3AddAndScale(pSphere->rigidBody.position, pData->rayTraceData.contact.normal.v, pSphere->radius);
			pData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->rayTraceData.contact.point, start));
			pData->traceDistance = sphereTraceVector3Distance(pData->rayTraceData.startPoint, pData->sphereCenter);
		}
	}

	return hits;
}

b32 sphereTraceColliderImposedSphereSphereTrace(ST_Vector3 start, ST_Direction dir, float radius, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereTraceData* const pData)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	ST_Vector3 dp = sphereTraceVector3Subtract(start, imposedPosition);
	float sphereRadiusPlusSphereCastRadius = imposedRadius + radius;
	float dpDist = sphereTraceVector3Length(dp);
	//ST_Vector3 p0 = sphere->pRigidBody->position;
	if (dpDist <= sphereRadiusPlusSphereCastRadius)
	{
		pData->rayTraceData.startPoint = start;
		pData->radius = radius;
		pData->rayTraceData.contact.normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(dp, 1.0f / dpDist), 1);
		pData->sphereCenter = start;
		pData->rayTraceData.contact.point = sphereTraceVector3AddAndScale(imposedPosition, pData->rayTraceData.contact.normal.v, imposedRadius);
		pData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->rayTraceData.contact.point, start));
		pData->traceDistance = 0.0f;
		return 1;
	}
	b32 hits = 0;
	ST_Vector3 negativeDir = sphereTraceVector3Negative(dir.v);
	float dist = sphereTraceVector3Dot(sphereTraceVector3Subtract(imposedPosition, start), negativeDir) / sphereTraceVector3Dot(dir.v, negativeDir);

	if (dist >= 0.0f)
	{
		ST_Vector3 intersection = sphereTraceVector3Add(start, sphereTraceVector3Scale(dir.v, dist));
		ST_Vector3 intersectionMinusStart = sphereTraceVector3Subtract(intersection, imposedPosition);
		float length = sphereTraceVector3Length(intersectionMinusStart);
		if (length <= sphereRadiusPlusSphereCastRadius)
		{
			hits = 1;
			pData->rayTraceData.startPoint = start;
			pData->radius = radius;
			float theta = acosf(length / sphereRadiusPlusSphereCastRadius);
			//ST_Vector3 contactPointDir =;
			pData->rayTraceData.contact.normal = sphereTraceDirectionConstruct(sphereTraceVector3Add(intersectionMinusStart, sphereTraceVector3Scale(negativeDir, sphereRadiusPlusSphereCastRadius * sinf(theta))), 0);
			pData->sphereCenter = sphereTraceVector3Add(imposedPosition, pData->rayTraceData.contact.normal.v);
			sphereTraceDirectionNormalizeIfNotNormalizedByRef(&pData->rayTraceData.contact.normal);
			pData->rayTraceData.contact.point = sphereTraceVector3AddAndScale(imposedPosition, pData->rayTraceData.contact.normal.v, imposedRadius);
			pData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->rayTraceData.contact.point, start));
			pData->traceDistance = sphereTraceVector3Distance(pData->rayTraceData.startPoint, pData->sphereCenter);
		}
	}

	return hits;
}

b32 sphereTraceColliderPointImposedSphereCollisionTest(ST_Vector3 point, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereContact* const pContact)
{
	float dist = sphereTraceVector3Distance(imposedPosition, point);
	if (dist <= imposedRadius)
	{
		pContact->penetrationDistance = imposedRadius - dist;
		pContact->point = point;
		pContact->normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(imposedPosition, point));
		pContact->collisionType = ST_COLLISION_POINT;
		return 1;
	}

	return 0;
}

b32 sphereTraceColliderEdgeImposedSphereCollisionTest(const ST_Edge* const pEdge, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereContact* const pContact)
{
	//ST_Vector3 edgeDir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pEdge->point2, pEdge->point1));
	//float edgeMaxDist = sphereTraceVector3Distance(pEdge->point2, pEdge->point1);
	ST_Vector3 positionMinusFirstPoint = sphereTraceVector3Subtract(imposedPosition, pEdge->point1);
	ST_Vector3 up = sphereTraceVector3Normalize(sphereTraceVector3Cross(pEdge->dir.v, positionMinusFirstPoint));
	if (sphereTraceVector3Nan(up))
	{
		pContact->normal = sphereTraceDirectionConstruct(gVector3Zero, 1);
		pContact->penetrationDistance = 0.0f;
		pContact->point = imposedPosition;
		pContact->collisionType = ST_COLLISION_EDGE;
		float edgeDist = sphereTraceVector3Dot(pEdge->dir.v, positionMinusFirstPoint);
		if (edgeDist < 0)
		{
			return sphereTraceColliderPointImposedSphereCollisionTest(pEdge->point1, imposedPosition, imposedRadius, pContact);
		}
		else if (edgeDist > pEdge->dist)
		{
			return sphereTraceColliderPointImposedSphereCollisionTest(pEdge->point2, imposedPosition, imposedRadius, pContact);
		}
		else
			return 1;
	}
	ST_Vector3 fwd = sphereTraceVector3Cross(pEdge->dir.v, up);
	ST_Vector3 cp = sphereTraceClosestPointOnLineBetweenTwoLines(imposedPosition, fwd, pEdge->point1, pEdge->dir.v);
	float dist = sphereTraceVector3Distance(cp, imposedPosition);
	if (dist <= imposedRadius)
	{
		pContact->normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(imposedPosition, cp));
		pContact->penetrationDistance = imposedRadius - dist;
		pContact->point = cp;
		pContact->collisionType = ST_COLLISION_EDGE;
		float edgeDist = sphereTraceVector3Dot(pEdge->dir.v, positionMinusFirstPoint);
		if (edgeDist < 0)
		{
			return sphereTraceColliderPointImposedSphereCollisionTest(pEdge->point1, imposedPosition, imposedRadius, pContact);
		}
		else if (edgeDist > pEdge->dist)
		{
			return sphereTraceColliderPointImposedSphereCollisionTest(pEdge->point2, imposedPosition, imposedRadius, pContact);
		}
		else
			return 1;
	}
	return 0;
}

b32 sphereTraceColliderRingImposedSphereCollisionTest(const ST_Ring* const pRing, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereContact* const pContact)
{
	ST_Vector3 dp = sphereTraceVector3Subtract(imposedPosition, pRing->centroid);
	if (sphereTraceVector3Equal(dp, gVector3Zero))
	{
		if (imposedRadius >= pRing->radius)
		{
			pContact->point = imposedPosition;
			pContact->collisionType = ST_COLLISION_EDGE;
			pContact->normal = pRing->normal;
			pContact->penetrationDistance = imposedRadius - pRing->radius;
			return 1;
		}
		else
			return 0;
	}
	float rightDist = sphereTraceDirectionGetMagnitudeInDirection(pRing->right, dp);
	float fwdDist = sphereTraceDirectionGetMagnitudeInDirection(pRing->forward, dp);

	ST_Vector3 planarDirection = sphereTraceVector3Normalize(sphereTraceVector3AddAndScale(sphereTraceVector3Scale(pRing->right.v, rightDist), pRing->forward.v, fwdDist));
	ST_Vector3 closestPointOnRing = sphereTraceVector3AddAndScale(pRing->centroid, planarDirection, pRing->radius);
	dp = sphereTraceVector3Subtract(imposedPosition, closestPointOnRing);
	float dist = sphereTraceVector3Length(dp);
	if (dist < imposedRadius)
	{
		pContact->point = closestPointOnRing;
		pContact->collisionType = ST_COLLISION_EDGE;
		pContact->normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(dp, 1.0f / dist), 1);
		pContact->penetrationDistance = imposedRadius -dist;
		return 1;
	}
	return 0;
}


b32 sphereTraceColliderPlaneSphereCollisionTest(ST_PlaneCollider* const pPlaneCollider, ST_SphereCollider* const pSphereCollider, ST_SphereContact* const contactInfo)
{
	if (sphereTraceColliderPlaneImposedSphereCollisionTest(pPlaneCollider, pSphereCollider->rigidBody.position, pSphereCollider->radius, contactInfo))
	{
		contactInfo->pSphereCollider = pSphereCollider;
		return 1;
	}
	return 0;
}

b32 sphereTraceColliderTriangleSphereCollisionTest(ST_TriangleCollider* const pTriangleCollider, ST_SphereCollider* const pSphereCollider, ST_SphereContact* const contactInfo)
{
	if (sphereTraceColliderTriangleImposedSphereCollisionTest(pTriangleCollider, pSphereCollider->rigidBody.position, pSphereCollider->radius, contactInfo))
	{
		contactInfo->pSphereCollider = pSphereCollider;
		return 1;
	}
	return 0;
}

b32 sphereTraceColliderPlaneImposedSphereCollisionTest(ST_PlaneCollider* const pPlaneCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereContact* const contactInfo)
{
	ST_RayTraceData rcd;
	ST_SphereContact contact;
	if (sphereTraceColliderInfinitePlaneImposedSphereCollisionTest(imposedPosition, imposedRadius, pPlaneCollider->normal, pPlaneCollider->position, &contact))
	{
		if (sphereTraceColliderPlaneIsProjectedPointContained(contact.point, pPlaneCollider))
		{
			contactInfo->collisionType = ST_COLLISION_FACE;
			contactInfo->normal = contact.normal;
			contactInfo->point = contact.point;
			contactInfo->penetrationDistance = contact.penetrationDistance;
			contactInfo->otherColliderType = COLLIDER_PLANE;
			contactInfo->pOtherCollider = pPlaneCollider;
			return 1;
		}
		ST_PlaneEdgeDirection closestEdge = sphereTraceColliderPlaneGetClosestTransformedEdgeToPoint(pPlaneCollider, imposedPosition);
		if (sphereTraceColliderEdgeImposedSphereCollisionTest(&pPlaneCollider->transformedEdges[closestEdge], imposedPosition, imposedRadius, &contact))
		{
			contactInfo->collisionType = contact.collisionType;
			contactInfo->normal = contact.normal;
			contactInfo->point = contact.point;
			contactInfo->penetrationDistance = contact.penetrationDistance;
			contactInfo->otherColliderType = COLLIDER_PLANE;
			contactInfo->pOtherCollider = pPlaneCollider;
			return 1;
		}
	}
	return 0;
}

//b32 sphereTraceColliderPlaneImposedSphereCollisionTest(ST_PlaneCollider* const pPlaneCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SpherePlaneContactInfo* const contactInfo)
//{
//	ST_RayTraceData rcd;
//	b32 didHit;
//	float dpDotNormal = sphereTraceVector3Dot(sphereTraceVector3Subtract(imposedPosition, pPlaneCollider->position), pPlaneCollider->normal.v);
//	if (dpDotNormal >= 0.0f)
//		didHit = sphereTraceColliderPlaneRayTrace(imposedPosition, sphereTraceDirectionNegative(pPlaneCollider->normal), pPlaneCollider, &rcd);
//	else
//		didHit = sphereTraceColliderPlaneRayTrace(imposedPosition, pPlaneCollider->normal, pPlaneCollider, &rcd);
//	if (rcd.distance <= imposedRadius)
//	{
//		//if we hit, we know its a face face collision
//		if (didHit)
//		{
//			//printf("face collision\n");
//			contactInfo->collisionType = COLLISION_FACE_FACE;
//			contactInfo->normal = rcd.normal;
//			contactInfo->penetrationDistance = imposedRadius - rcd.distance;
//			contactInfo->pSphereCollider = NULL;
//			contactInfo->pPlaneCollider = pPlaneCollider;
//			contactInfo->point = rcd.hitPoint;
//			return 1;
//		}
//
//		ST_RayTraceData rcd2;
//		ST_Vector3 contactPos;
//		//otherwise we need to check the edges first
//		ST_PlaneEdgeDirection closestEdge = sphereTraceColliderPlaneGetClosestTransformedEdgeToPoint(pPlaneCollider, imposedPosition);
//		switch (closestEdge)
//		{
//		case PLANE_EDGE_RIGHT:
//		{
//			didHit = sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, pPlaneCollider->forward, imposedPosition, imposedRadius, &rcd)
//				&& sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, sphereTraceDirectionNegative(pPlaneCollider->forward), imposedPosition, imposedRadius, &rcd2);
//			if (didHit)
//				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
//			break;
//		}
//		case PLANE_EDGE_FORWARD:
//		{
//			//printf("plane edge forwards\n");
//			didHit = sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, sphereTraceDirectionNegative(pPlaneCollider->right), imposedPosition, imposedRadius, &rcd)
//				&& sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, pPlaneCollider->right, imposedPosition, imposedRadius, &rcd2);
//			if (didHit)
//				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
//			break;
//		}
//		case PLANE_EDGE_LEFT:
//		{
//			didHit = sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, sphereTraceDirectionNegative(pPlaneCollider->forward), imposedPosition, imposedRadius, &rcd)
//				&& sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, pPlaneCollider->forward, imposedPosition, imposedRadius, &rcd2);
//			if (didHit)
//				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
//			break;
//		}
//		case PLANE_EDGE_BACK:
//		{
//			didHit = sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, pPlaneCollider->right, imposedPosition, imposedRadius, &rcd)
//				&& sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, sphereTraceDirectionNegative(pPlaneCollider->right), imposedPosition, imposedRadius, &rcd2);
//			if (didHit)
//				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
//			break;
//		}
//		}
//
//		if (didHit)
//		{
//			contactInfo->collisionType = COLLISION_FACE_EDGE;
//			contactInfo->normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(imposedPosition, contactPos));
//			//contactInfo->normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(contactPos, pSphereCollider->pRigidBody->position));
//			contactInfo->penetrationDistance = imposedRadius - sphereTraceVector3Length(sphereTraceVector3Subtract(imposedPosition, contactPos));
//			contactInfo->pSphereCollider = NULL;
//			contactInfo->pPlaneCollider = pPlaneCollider;
//			contactInfo->point = contactPos;
//			return 1;
//		}
//
//		//now we check the closest point
//		ST_PlaneVertexDirection closestVertexDirection = sphereTraceColliderPlaneGetClosestTransformedVertexToPoint(pPlaneCollider, imposedPosition);
//		contactPos = pPlaneCollider->transformedVertices[closestVertexDirection];
//		float dist = sphereTraceVector3Length(sphereTraceVector3Subtract(contactPos, imposedPosition));
//		if (dist <= imposedRadius)
//		{
//			//printf("point collision\n");
//			contactInfo->collisionType = COLLISION_FACE_POINT;
//			contactInfo->normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(sphereTraceVector3Subtract(imposedPosition, contactPos), 1.0f / dist), 1);
//			contactInfo->penetrationDistance = imposedRadius - dist;
//			contactInfo->pSphereCollider = NULL;
//			contactInfo->pPlaneCollider = pPlaneCollider;
//			contactInfo->point = contactPos;
//			return 1;
//		}
//
//		return 0;
//	}
//	else
//		return 0;
//}

b32 sphereTraceColliderTriangleImposedSphereCollisionTest(ST_TriangleCollider* const pTriangleCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereContact* const contactInfo)
{
	ST_RayTraceData rcd;
	ST_SphereContact contact;
	if (sphereTraceColliderInfinitePlaneImposedSphereCollisionTest(imposedPosition, imposedRadius, pTriangleCollider->normal, pTriangleCollider->centroid, &contact))
	{
		if (sphereTraceColliderTriangleIsProjectedPointContained(contact.point, pTriangleCollider))
		{
			contactInfo->collisionType = ST_COLLISION_FACE;
			contactInfo->normal = contact.normal;
			contactInfo->point = contact.point;
			contactInfo->penetrationDistance = contact.penetrationDistance;
			contactInfo->pOtherCollider = pTriangleCollider;
			contactInfo->otherColliderType = COLLIDER_TRIANGLE;
			return 1;
		}
		int closestEdgeIndex = sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToPoint(pTriangleCollider, imposedPosition);
		if (sphereTraceColliderEdgeImposedSphereCollisionTest(&pTriangleCollider->transformedEdges[closestEdgeIndex], imposedPosition, imposedRadius, &contact))
		{
			contactInfo->collisionType = contact.collisionType;
			contactInfo->normal = contact.normal;
			contactInfo->point = contact.point;
			contactInfo->penetrationDistance = contact.penetrationDistance;
			contactInfo->pOtherCollider = pTriangleCollider;
			contactInfo->otherColliderType = COLLIDER_TRIANGLE;
			return 1;
		}
	}
	return 0;
}

//b32 sphereTraceColliderTriangleImposedSphereCollisionTest(ST_TriangleCollider* const pTriangleCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereTriangleContactInfo* const contactInfo)
//{
//	ST_RayTraceData rcd;
//	b32 didHit;
//	float dpDotNormal = sphereTraceVector3Dot(sphereTraceVector3Subtract(imposedPosition, pTriangleCollider->centroid), pTriangleCollider->normal.v);
//	if (dpDotNormal >= 0.0f)
//		didHit = sphereTraceColliderTriangleRayTrace(imposedPosition, sphereTraceDirectionNegative(pTriangleCollider->normal), pTriangleCollider, &rcd);
//	else
//		didHit = sphereTraceColliderTriangleRayTrace(imposedPosition, pTriangleCollider->normal, pTriangleCollider, &rcd);
//	if (rcd.distance <= imposedRadius)
//	{
//		//if we hit, we know its a face face collision
//		if (didHit)
//		{
//			//printf("face collision\n");
//			contactInfo->collisionType = COLLISION_FACE_FACE;
//			contactInfo->normal = rcd.normal;
//			contactInfo->penetrationDistance = imposedRadius - rcd.distance;
//			contactInfo->pSphereCollider = NULL;
//			contactInfo->pTriangleCollider = pTriangleCollider;
//			contactInfo->point = rcd.hitPoint;
//			return 1;
//		}
//
//		ST_RayTraceData rcd2;
//		ST_Vector3 contactPos;
//		//otherwise we need to check the edges first
//		int closestEdge = sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToPoint(pTriangleCollider, imposedPosition);
//		didHit = sphereTraceColliderImposedSphereRayTrace(pTriangleCollider->transformedEdges[closestEdge].point1, pTriangleCollider->transformedEdges[closestEdge].dir, imposedPosition, imposedRadius, &rcd)
//			&& sphereTraceColliderImposedSphereRayTrace(pTriangleCollider->transformedEdges[closestEdge].point2, sphereTraceDirectionNegative(pTriangleCollider->transformedEdges[closestEdge].dir), imposedPosition, imposedRadius, &rcd2);
//		if (didHit)
//			contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
//		if (didHit)
//		{
//			contactInfo->collisionType = COLLISION_FACE_EDGE;
//			contactInfo->normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(imposedPosition, contactPos));
//			//contactInfo->normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(contactPos, pSphereCollider->pRigidBody->position));
//			contactInfo->penetrationDistance = imposedRadius - sphereTraceVector3Length(sphereTraceVector3Subtract(imposedPosition, contactPos));
//			contactInfo->pSphereCollider = NULL;
//			contactInfo->pTriangleCollider = pTriangleCollider;
//			contactInfo->point = contactPos;
//			return 1;
//		}
//
//		//now we check the closest point
//		int closestVertexDirection = sphereTraceColliderTriangleGetClosestTransformedVertexIndexToPoint(pTriangleCollider, imposedPosition);
//		contactPos = pTriangleCollider->transformedVertices[closestVertexDirection];
//		float dist = sphereTraceVector3Length(sphereTraceVector3Subtract(contactPos, imposedPosition));
//		if (dist <= imposedRadius)
//		{
//			//printf("point collision\n");
//			contactInfo->collisionType = COLLISION_FACE_POINT;
//			contactInfo->normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(sphereTraceVector3Subtract(imposedPosition, contactPos), 1.0f / dist), 1);
//			contactInfo->penetrationDistance = imposedRadius - dist;
//			contactInfo->pSphereCollider = NULL;
//			contactInfo->pTriangleCollider = pTriangleCollider;
//			contactInfo->point = contactPos;
//			return 1;
//		}
//
//		return 0;
//	}
//	else
//		return 0;
//}

b32 sphereTraceColliderSphereSphereCollisionTest(ST_SphereCollider* const pSphereColliderA, ST_SphereCollider* const pSphereColliderB, ST_SphereContact* const pContactInfo)
{
	float rab = pSphereColliderA->radius + pSphereColliderB->radius;
	if (sphereTraceVector3Equal(pSphereColliderA->rigidBody.position, pSphereColliderB->rigidBody.position))
	{
		pContactInfo->normal = sphereTraceDirectionConstruct(gVector3Right, 1);
		pContactInfo->pSphereCollider = pSphereColliderA;
		pContactInfo->pOtherCollider = pSphereColliderB;
		pContactInfo->otherColliderType = COLLIDER_SPHERE;
		pContactInfo->penetrationDistance = rab;
		pContactInfo->point = pSphereColliderA->rigidBody.position;
		return 1;
	}
	ST_Vector3 dp = sphereTraceVector3Subtract(pSphereColliderB->rigidBody.position, pSphereColliderA->rigidBody.position);
	float dist = sphereTraceVector3Length(dp);
	if (dist <= rab)
	{
		pContactInfo->normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(dp, 1.0f / dist), 1);
		pContactInfo->pSphereCollider = pSphereColliderA;
		pContactInfo->pOtherCollider = pSphereColliderB;
		pContactInfo->otherColliderType = COLLIDER_SPHERE;
		pContactInfo->penetrationDistance = rab - dist;
		ST_Vector3 pIntA = sphereTraceVector3AddAndScale(pSphereColliderA->rigidBody.position, pContactInfo->normal.v, pSphereColliderA->radius);
		ST_Vector3 pIntB = sphereTraceVector3AddAndScale(pSphereColliderB->rigidBody.position, sphereTraceVector3Negative(pContactInfo->normal.v), pSphereColliderB->radius);
		pContactInfo->point = sphereTraceVector3Average(pIntA, pIntB);
		return 1;
	}
	else
		return 0;
}

b32 sphereTraceColliderSphereSphereTraceOut(ST_Vector3 spherePos, float sphereRadius, ST_Direction clipoutDir, ST_Vector3 encompassingPos, float encompassingRadius, ST_SphereContact* const pContactInfo)
{
	ST_Vector3 dp = sphereTraceVector3Subtract(spherePos, encompassingPos);
	float dist = sphereTraceVector3Length(dp);
	if (dist > sphereRadius + encompassingRadius)
	{
		//the sphere has already clipped outward
		return 0;
	}
	ST_Vector3 castPoint = sphereTraceVector3AddAndScale(spherePos, clipoutDir.v, 2 * encompassingRadius + 2 * sphereRadius);
	return sphereTraceColliderImposedSphereSphereTrace(castPoint, sphereTraceDirectionNegative(clipoutDir), sphereRadius, encompassingPos, encompassingRadius, pContactInfo);

}