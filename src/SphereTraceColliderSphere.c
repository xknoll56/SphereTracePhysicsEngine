#include "SphereTraceColliderSphere.h"
#include "SphereTraceGlobals.h"

void sphereTraceColliderSphereAABBSetTransformedVertices(ST_SphereCollider* const pSphereCollider)
{
	pSphereCollider->aabb.rightTopForwardsTransformedVertex = sphereTraceVector3Add(pSphereCollider->pRigidBody->position, pSphereCollider->aabb.halfExtents);
	pSphereCollider->aabb.leftDownBackTransformedVertex = sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, pSphereCollider->aabb.halfExtents);
}

//todo, check the closest edge instead
b32 sphereTraceColliderAABBIntersectImposedSphere(const ST_AABB* const aabb, ST_Vector3 imposedPosition, float imposedRadius)
{
	ST_Vector3 dp = sphereTraceVector3Subtract(imposedPosition, sphereTraceColliderAABBMidPoint(aabb));
	if (fabsf(sphereTraceVector3Dot(dp, gVector3Right)) > aabb->halfExtents.x + imposedRadius)
		return 0;
	if (fabsf(sphereTraceVector3Dot(dp, gVector3Up)) > aabb->halfExtents.y + imposedRadius)
		return 0;
	if (fabsf(sphereTraceVector3Dot(dp, gVector3Forward)) > aabb->halfExtents.z + imposedRadius)
		return 0;
	return 1;
}

b32 sphereTraceColliderAABBIntersectSphere(const ST_AABB* const aabb, const ST_SphereCollider* const pSphereCollider)
{
	ST_Vector3 dp = sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, sphereTraceColliderAABBMidPoint(aabb));
	if (fabsf(sphereTraceVector3Dot(dp, gVector3Right)) > aabb->halfExtents.x + pSphereCollider->radius)
		return 0;
	if (fabsf(sphereTraceVector3Dot(dp, gVector3Up)) > aabb->halfExtents.y + pSphereCollider->radius)
		return 0;
	if (fabsf(sphereTraceVector3Dot(dp, gVector3Forward)) > aabb->halfExtents.z + pSphereCollider->radius)
		return 0;
	return 1;
}

b32 sphereTraceColliderInfinitePlaneImposedSphereCollisionTest(ST_Vector3 imposedPosition, float imposedRadius, ST_Direction planeNormal, ST_Vector3 pointOnPlane, ST_Contact* const pContact)
{
	ST_RayTraceData rtd;
	sphereTraceColliderInfinitePlaneRayTrace(imposedPosition, planeNormal, planeNormal, pointOnPlane, &rtd);
	if (fabsf(rtd.distance) <= imposedRadius)
	{
		pContact->penetrationDistance = imposedRadius - rtd.distance;
		pContact->point = rtd.hitPoint;
		pContact->normal = rtd.normal;
		//if (sphereTraceVector3Dot(sphereTraceVector3Subtract(rtd.hitPoint, imposedPosition), pContact->normal) > 0.0f)
		//{
		//	sphereTraceVector3NegativeByRef(&pContact->normal);
		//}
		return 1;
	}
	return 0;
}

ST_SphereCollider sphereTraceColliderSphereConstruct(float radius, ST_RigidBody* const pRigidBody)
{
	ST_SphereCollider sphereCollider;
	sphereCollider.pRigidBody = pRigidBody;
	sphereCollider.radius = radius;
	sphereCollider.aabb.halfExtents = sphereTraceVector3Construct(radius, radius, radius);
	sphereCollider.ignoreCollisions = 0;
	sphereTraceColliderSphereAABBSetTransformedVertices(&sphereCollider);

	return sphereCollider;
}

ST_SphereCollider sphereTraceColliderSphereConstructWithPosition(float radius, ST_RigidBody* const pRigidBody, ST_Vector3 position)
{
	ST_SphereCollider sphereCollider;
	sphereCollider.pRigidBody = pRigidBody;
	sphereCollider.pRigidBody->position = position;
	sphereCollider.radius = radius;
	sphereCollider.aabb.halfExtents = sphereTraceVector3Construct(radius, radius, radius);
	sphereCollider.ignoreCollisions = 0;
	sphereTraceColliderSphereAABBSetTransformedVertices(&sphereCollider);

	return sphereCollider;
}

b32 sphereTraceColliderSphereRayTrace(ST_Vector3 start, ST_Direction dir, const ST_SphereCollider* const pSphere, ST_RayTraceData* const pData)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	b32 hits = 0;
	ST_Vector3 p0 = pSphere->pRigidBody->position;
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
			float theta = fminf(1.0f, asinf(length / pSphere->radius));
			ST_Vector3 dirComp = sphereTraceVector3Scale(negativeDir, length / tanf(theta));
			if (length == 0.0f)
				dirComp = sphereTraceVector3Scale(negativeDir, pSphere->radius);
			pData->normal = sphereTraceDirectionConstruct(sphereTraceVector3Add(intersectionMinusStart, dirComp), 0);
			pData->hitPoint = sphereTraceVector3Add(pData->normal.v, p0);
			//stupid hack
			if (sphereTraceVector3Nan(pData->hitPoint))
				return 0;
			pData->startPoint = start;
			pData->distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->hitPoint, start));
			sphereTraceDirectionNormalizeIfNotNormalizedByRef(&pData->normal);
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
		float l2 = sphereTraceVector3Length2(intersectionMinusStart);
		if (l2 <= imposedRadius * imposedRadius)
		{
			hits = 1;
			float length = sphereTraceVector3Length(intersectionMinusStart);
			float theta = fminf(1.0f, asinf(length / imposedRadius));
			ST_Vector3 dirComp = sphereTraceVector3Scale(negativeDir, length / tanf(theta));
			if (length == 0.0f)
				dirComp = sphereTraceVector3Scale(negativeDir, imposedRadius);
			pData->normal = sphereTraceDirectionConstruct(sphereTraceVector3Add(intersectionMinusStart, dirComp), 0);
			pData->hitPoint = sphereTraceVector3Add(pData->normal.v, p0);
			//stupid hack
			if (sphereTraceVector3Nan(pData->hitPoint))
				return 0;
			pData->startPoint = start;
			pData->distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->hitPoint, start));
			sphereTraceDirectionNormalizeIfNotNormalizedByRef(&pData->normal);
		}
	}

	return hits;
}

b32 sphereTraceColliderSphereSphereTrace(ST_Vector3 start, ST_Direction dir, float radius, const ST_SphereCollider* const pSphere, ST_SphereTraceData* const pData)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	ST_Vector3 dp = sphereTraceVector3Subtract(start, pSphere->pRigidBody->position);
	float sphereRadiusPlusSphereCastRadius = pSphere->radius + radius;
	float dpDist = sphereTraceVector3Length(dp);
	//ST_Vector3 p0 = sphere->pRigidBody->position;
	if (dpDist <= sphereRadiusPlusSphereCastRadius)
	{
		pData->rayTraceData.startPoint = start;
		pData->radius = radius;
		pData->rayTraceData.normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(dp, 1.0f / dpDist), 1);
		pData->sphereCenter = start;
		pData->rayTraceData.hitPoint = sphereTraceVector3AddAndScale(pSphere->pRigidBody->position, pData->rayTraceData.normal.v, pSphere->radius);
		pData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->rayTraceData.hitPoint, start));
		pData->traceDistance = 0.0f;
		return 1;
	}
	b32 hits = 0;
	ST_Vector3 negativeDir = sphereTraceVector3Negative(dir.v);
	float dist = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphere->pRigidBody->position, start), negativeDir) / sphereTraceVector3Dot(dir.v, negativeDir);

	if (dist >= 0.0f)
	{
		ST_Vector3 intersection = sphereTraceVector3Add(start, sphereTraceVector3Scale(dir.v, dist));
		ST_Vector3 intersectionMinusStart = sphereTraceVector3Subtract(intersection, pSphere->pRigidBody->position);
		float l2 = sphereTraceVector3Length2(intersectionMinusStart);
		if (l2 <= sphereRadiusPlusSphereCastRadius * sphereRadiusPlusSphereCastRadius)
		{
			hits = 1;
			float length = sphereTraceVector3Length(intersectionMinusStart);
			float theta = asinf(length / sphereRadiusPlusSphereCastRadius);
			ST_Vector3 dirComp = sphereTraceVector3Scale(negativeDir, length / tanf(theta));
			pData->rayTraceData.startPoint = start;
			pData->radius = radius;
			pData->rayTraceData.normal = sphereTraceDirectionConstruct(sphereTraceVector3Add(intersectionMinusStart, dirComp), 1);
			pData->sphereCenter = sphereTraceVector3Add(pSphere->pRigidBody->position, pData->rayTraceData.normal.v);
			sphereTraceDirectionNormalizeIfNotNormalizedByRef(&pData->rayTraceData.normal);
			pData->rayTraceData.hitPoint = sphereTraceVector3AddAndScale(pSphere->pRigidBody->position, pData->rayTraceData.normal.v, pSphere->radius);
			pData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->rayTraceData.hitPoint, start));
			pData->traceDistance = sphereTraceVector3Distance(pData->rayTraceData.startPoint, pData->sphereCenter);
		}
	}

	return hits;
}

b32 sphereTraceColliderPointImposedSphereCollisionTest(ST_Vector3 point, ST_Vector3 imposedPosition, float imposedRadius, ST_Contact* const pContact)
{
	float dist = sphereTraceVector3Distance(imposedPosition, point);
	if (dist <= imposedRadius)
	{
		pContact->penetrationDistance = imposedRadius - dist;
		pContact->point = point;
		pContact->normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(imposedPosition, point));
		return 1;
	}

	return 0;
}

b32 sphereTraceColliderEdgeImposedSphereCollisionTest(const ST_Edge* const pEdge, ST_Vector3 imposedPosition, float imposedRadius, ST_Contact* const pContact)
{
	ST_Vector3 edgeDir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pEdge->point2, pEdge->point1));
	float edgeMaxDist = sphereTraceVector3Distance(pEdge->point2, pEdge->point1);
	ST_Vector3 positionMinusFirstPoint = sphereTraceVector3Subtract(imposedPosition, pEdge->point1);
	ST_Vector3 up = sphereTraceVector3Normalize(sphereTraceVector3Cross(edgeDir, positionMinusFirstPoint));
	if (sphereTraceVector3Nan(up))
	{
		pContact->normal = sphereTraceDirectionConstruct(gVector3Zero, 1);
		pContact->penetrationDistance = 0.0f;
		pContact->point = imposedPosition;
		float edgeDist = sphereTraceVector3Dot(edgeDir, positionMinusFirstPoint);
		if (edgeDist < 0)
		{
			return sphereTraceColliderPointImposedSphereCollisionTest(pEdge->point1, imposedPosition, imposedRadius, pContact);
		}
		else if (edgeDist > edgeMaxDist)
		{
			return sphereTraceColliderPointImposedSphereCollisionTest(pEdge->point2, imposedPosition, imposedRadius, pContact);
		}
		else
			return 1;
	}
	ST_Vector3 fwd = sphereTraceVector3Cross(edgeDir, up);
	ST_Vector3 cp = sphereTraceClosestPointOnLineBetweenTwoLines(imposedPosition, fwd, pEdge->point1, edgeDir);
	float dist = sphereTraceVector3Distance(cp, imposedPosition);
	if (dist <= imposedRadius)
	{
		pContact->normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(imposedPosition, cp));
		pContact->penetrationDistance = imposedRadius - dist;
		pContact->point = cp;
		float edgeDist = sphereTraceVector3Dot(edgeDir, positionMinusFirstPoint);
		if (edgeDist < 0)
		{
			return sphereTraceColliderPointImposedSphereCollisionTest(pEdge->point1, imposedPosition, imposedRadius, pContact);
		}
		else if (edgeDist > edgeMaxDist)
		{
			return sphereTraceColliderPointImposedSphereCollisionTest(pEdge->point2, imposedPosition, imposedRadius, pContact);
		}
		else
			return 1;
	}
	return 0;
}

b32 sphereTraceColliderPlaneSphereCollisionTest(ST_PlaneCollider* const pPlaneCollider, ST_SphereCollider* const pSphereCollider, ST_SpherePlaneContactInfo* const contactInfo)
{
	ST_RayTraceData rcd;
	b32 didHit;
	float dpDotNormal = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, pPlaneCollider->position), pPlaneCollider->normal.v);
	if (dpDotNormal >= 0.0f)
		didHit = sphereTraceColliderPlaneRayTrace(pSphereCollider->pRigidBody->position, sphereTraceDirectionNegative(pPlaneCollider->normal), pPlaneCollider, &rcd);
	else
		didHit = sphereTraceColliderPlaneRayTrace(pSphereCollider->pRigidBody->position, pPlaneCollider->normal, pPlaneCollider, &rcd);
	if (rcd.distance <= pSphereCollider->radius)
	{
		//if we hit, we know its a face face collision
		if (didHit)
		{
			//printf("face collision\n");
			contactInfo->collisionType = COLLISION_FACE_FACE;
			contactInfo->normal = rcd.normal;
			contactInfo->penetrationDistance = pSphereCollider->radius - rcd.distance;
			contactInfo->pSphereCollider = pSphereCollider;
			contactInfo->pPlaneCollider = pPlaneCollider;
			contactInfo->point = rcd.hitPoint;
			return 1;
		}

		ST_RayTraceData rcd2;
		ST_Vector3 contactPos;
		//otherwise we need to check the edges first
		ST_PlaneEdgeDirection closestEdge = sphereTraceColliderPlaneGetClosestTransformedEdgeToPoint(pPlaneCollider, pSphereCollider->pRigidBody->position);
		switch (closestEdge)
		{
		case PLANE_EDGE_RIGHT:
		{
			didHit = sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, pPlaneCollider->forward, pSphereCollider, &rcd)
				&& sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, sphereTraceDirectionNegative(pPlaneCollider->forward), pSphereCollider, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		case PLANE_EDGE_FORWARD:
		{
			//printf("plane edge forwards\n");
			didHit = sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, sphereTraceDirectionNegative(pPlaneCollider->right), pSphereCollider, &rcd)
				&& sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, pPlaneCollider->right, pSphereCollider, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		case PLANE_EDGE_LEFT:
		{
			didHit = sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, sphereTraceDirectionNegative(pPlaneCollider->forward), pSphereCollider, &rcd)
				&& sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, pPlaneCollider->forward, pSphereCollider, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		case PLANE_EDGE_BACK:
		{
			didHit = sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, pPlaneCollider->right, pSphereCollider, &rcd)
				&& sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, sphereTraceDirectionNegative(pPlaneCollider->right), pSphereCollider, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		}

		if (didHit)
		{
			contactInfo->collisionType = COLLISION_FACE_EDGE;
			contactInfo->normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, contactPos));
			//contactInfo->normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(contactPos, pSphereCollider->pRigidBody->position));
			contactInfo->penetrationDistance = pSphereCollider->radius - sphereTraceVector3Length(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, contactPos));
			contactInfo->pSphereCollider = pSphereCollider;
			contactInfo->pPlaneCollider = pPlaneCollider;
			contactInfo->point = contactPos;
			return 1;
		}

		//now we check the closest point
		ST_PlaneVertexDirection closestVertexDirection = sphereTraceColliderPlaneGetClosestTransformedVertexToPoint(pPlaneCollider, pSphereCollider->pRigidBody->position);
		contactPos = pPlaneCollider->transformedVertices[closestVertexDirection];
		float dist = sphereTraceVector3Length(sphereTraceVector3Subtract(contactPos, pSphereCollider->pRigidBody->position));
		if (dist <= pSphereCollider->radius)
		{
			//printf("point collision\n");
			contactInfo->collisionType = COLLISION_FACE_POINT;
			contactInfo->normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, contactPos), 1.0f / dist), 1);
			contactInfo->penetrationDistance = pSphereCollider->radius - dist;
			contactInfo->pSphereCollider = pSphereCollider;
			contactInfo->pPlaneCollider = pPlaneCollider;
			contactInfo->point = contactPos;
			return 1;
		}

		return 0;
	}
	else
		return 0;
}

b32 sphereTraceColliderTriangleSphereCollisionTest(ST_TriangleCollider* const pTriangleCollider, ST_SphereCollider* const pSphereCollider, ST_SphereTriangleContactInfo* const contactInfo)
{
	ST_RayTraceData rcd;
	b32 didHit;
	float dpDotNormal = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, pTriangleCollider->centroid), pTriangleCollider->normal.v);
	if (dpDotNormal >= 0.0f)
		didHit = sphereTraceColliderTriangleRayTrace(pSphereCollider->pRigidBody->position, sphereTraceDirectionNegative(pTriangleCollider->normal), pTriangleCollider, &rcd);
	else
		didHit = sphereTraceColliderTriangleRayTrace(pSphereCollider->pRigidBody->position, pTriangleCollider->normal, pTriangleCollider, &rcd);
	if (rcd.distance <= pSphereCollider->radius)
	{
		//if we hit, we know its a face face collision
		if (didHit)
		{
			//printf("face collision\n");
			contactInfo->collisionType = COLLISION_FACE_FACE;
			contactInfo->normal = rcd.normal;
			contactInfo->penetrationDistance = pSphereCollider->radius - rcd.distance;
			contactInfo->pSphereCollider = pSphereCollider;
			contactInfo->pTriangleCollider = pTriangleCollider;
			contactInfo->point = rcd.hitPoint;
			return 1;
		}

		ST_RayTraceData rcd2;
		ST_Vector3 contactPos;
		//otherwise we need to check the edges first
		int closestEdge = sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToPoint(pTriangleCollider, pSphereCollider->pRigidBody->position);
		didHit = sphereTraceColliderSphereRayTrace(pTriangleCollider->transformedEdges[closestEdge].point1, pTriangleCollider->transformedEdges[closestEdge].dir, pSphereCollider, &rcd)
			&& sphereTraceColliderSphereRayTrace(pTriangleCollider->transformedEdges[closestEdge].point2, sphereTraceDirectionNegative(pTriangleCollider->transformedEdges[closestEdge].dir), pSphereCollider, &rcd2);
		if (didHit)
			contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
		if (didHit)
		{
			contactInfo->collisionType = COLLISION_FACE_EDGE;
			contactInfo->normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, contactPos));
			//contactInfo->normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(contactPos, pSphereCollider->pRigidBody->position));
			contactInfo->penetrationDistance = pSphereCollider->radius - sphereTraceVector3Length(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, contactPos));
			contactInfo->pSphereCollider = pSphereCollider;
			contactInfo->pTriangleCollider = pTriangleCollider;
			contactInfo->point = contactPos;
			return 1;
		}

		//now we check the closest point
		int closestVertexDirection = sphereTraceColliderTriangleGetClosestTransformedVertexIndexToPoint(pTriangleCollider, pSphereCollider->pRigidBody->position);
		contactPos = pTriangleCollider->transformedVertices[closestVertexDirection];
		float dist = sphereTraceVector3Length(sphereTraceVector3Subtract(contactPos, pSphereCollider->pRigidBody->position));
		if (dist <= pSphereCollider->radius)
		{
			//printf("point collision\n");
			contactInfo->collisionType = COLLISION_FACE_POINT;
			contactInfo->normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, contactPos), 1.0f / dist), 1);
			contactInfo->penetrationDistance = pSphereCollider->radius - dist;
			contactInfo->pSphereCollider = pSphereCollider;
			contactInfo->pTriangleCollider = pTriangleCollider;
			contactInfo->point = contactPos;
			return 1;
		}

		return 0;
	}
	else
		return 0;
}

b32 sphereTraceColliderPlaneImposedSphereCollisionTest(ST_PlaneCollider* const pPlaneCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SpherePlaneContactInfo* const contactInfo)
{
	ST_RayTraceData rcd;
	b32 didHit;
	float dpDotNormal = sphereTraceVector3Dot(sphereTraceVector3Subtract(imposedPosition, pPlaneCollider->position), pPlaneCollider->normal.v);
	if (dpDotNormal >= 0.0f)
		didHit = sphereTraceColliderPlaneRayTrace(imposedPosition, sphereTraceDirectionNegative(pPlaneCollider->normal), pPlaneCollider, &rcd);
	else
		didHit = sphereTraceColliderPlaneRayTrace(imposedPosition, pPlaneCollider->normal, pPlaneCollider, &rcd);
	if (rcd.distance <= imposedRadius)
	{
		//if we hit, we know its a face face collision
		if (didHit)
		{
			//printf("face collision\n");
			contactInfo->collisionType = COLLISION_FACE_FACE;
			contactInfo->normal = rcd.normal;
			contactInfo->penetrationDistance = imposedRadius - rcd.distance;
			contactInfo->pSphereCollider = NULL;
			contactInfo->pPlaneCollider = pPlaneCollider;
			contactInfo->point = rcd.hitPoint;
			return 1;
		}

		ST_RayTraceData rcd2;
		ST_Vector3 contactPos;
		//otherwise we need to check the edges first
		ST_PlaneEdgeDirection closestEdge = sphereTraceColliderPlaneGetClosestTransformedEdgeToPoint(pPlaneCollider, imposedPosition);
		switch (closestEdge)
		{
		case PLANE_EDGE_RIGHT:
		{
			didHit = sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, pPlaneCollider->forward, imposedPosition, imposedRadius, &rcd)
				&& sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, sphereTraceDirectionNegative(pPlaneCollider->forward), imposedPosition, imposedRadius, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		case PLANE_EDGE_FORWARD:
		{
			//printf("plane edge forwards\n");
			didHit = sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, sphereTraceDirectionNegative(pPlaneCollider->right), imposedPosition, imposedRadius, &rcd)
				&& sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, pPlaneCollider->right, imposedPosition, imposedRadius, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		case PLANE_EDGE_LEFT:
		{
			didHit = sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, sphereTraceDirectionNegative(pPlaneCollider->forward), imposedPosition, imposedRadius, &rcd)
				&& sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, pPlaneCollider->forward, imposedPosition, imposedRadius, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		case PLANE_EDGE_BACK:
		{
			didHit = sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, pPlaneCollider->right, imposedPosition, imposedRadius, &rcd)
				&& sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, sphereTraceDirectionNegative(pPlaneCollider->right), imposedPosition, imposedRadius, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		}

		if (didHit)
		{
			contactInfo->collisionType = COLLISION_FACE_EDGE;
			contactInfo->normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(imposedPosition, contactPos));
			//contactInfo->normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(contactPos, pSphereCollider->pRigidBody->position));
			contactInfo->penetrationDistance = imposedRadius - sphereTraceVector3Length(sphereTraceVector3Subtract(imposedPosition, contactPos));
			contactInfo->pSphereCollider = NULL;
			contactInfo->pPlaneCollider = pPlaneCollider;
			contactInfo->point = contactPos;
			return 1;
		}

		//now we check the closest point
		ST_PlaneVertexDirection closestVertexDirection = sphereTraceColliderPlaneGetClosestTransformedVertexToPoint(pPlaneCollider, imposedPosition);
		contactPos = pPlaneCollider->transformedVertices[closestVertexDirection];
		float dist = sphereTraceVector3Length(sphereTraceVector3Subtract(contactPos, imposedPosition));
		if (dist <= imposedRadius)
		{
			//printf("point collision\n");
			contactInfo->collisionType = COLLISION_FACE_POINT;
			contactInfo->normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(sphereTraceVector3Subtract(imposedPosition, contactPos), 1.0f / dist), 1);
			contactInfo->penetrationDistance = imposedRadius - dist;
			contactInfo->pSphereCollider = NULL;
			contactInfo->pPlaneCollider = pPlaneCollider;
			contactInfo->point = contactPos;
			return 1;
		}

		return 0;
	}
	else
		return 0;
}

b32 sphereTraceColliderTriangleImposedSphereCollisionTest(ST_TriangleCollider* const pTriangleCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereTriangleContactInfo* const contactInfo)
{
	ST_RayTraceData rcd;
	b32 didHit;
	float dpDotNormal = sphereTraceVector3Dot(sphereTraceVector3Subtract(imposedPosition, pTriangleCollider->centroid), pTriangleCollider->normal.v);
	if (dpDotNormal >= 0.0f)
		didHit = sphereTraceColliderTriangleRayTrace(imposedPosition, sphereTraceDirectionNegative(pTriangleCollider->normal), pTriangleCollider, &rcd);
	else
		didHit = sphereTraceColliderTriangleRayTrace(imposedPosition, pTriangleCollider->normal, pTriangleCollider, &rcd);
	if (rcd.distance <= imposedRadius)
	{
		//if we hit, we know its a face face collision
		if (didHit)
		{
			//printf("face collision\n");
			contactInfo->collisionType = COLLISION_FACE_FACE;
			contactInfo->normal = rcd.normal;
			contactInfo->penetrationDistance = imposedRadius - rcd.distance;
			contactInfo->pSphereCollider = NULL;
			contactInfo->pTriangleCollider = pTriangleCollider;
			contactInfo->point = rcd.hitPoint;
			return 1;
		}

		ST_RayTraceData rcd2;
		ST_Vector3 contactPos;
		//otherwise we need to check the edges first
		int closestEdge = sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToPoint(pTriangleCollider, imposedPosition);
		didHit = sphereTraceColliderImposedSphereRayTrace(pTriangleCollider->transformedEdges[closestEdge].point1, pTriangleCollider->transformedEdges[closestEdge].dir, imposedPosition, imposedRadius, &rcd)
			&& sphereTraceColliderImposedSphereRayTrace(pTriangleCollider->transformedEdges[closestEdge].point2, sphereTraceDirectionNegative(pTriangleCollider->transformedEdges[closestEdge].dir), imposedPosition, imposedRadius, &rcd2);
		if (didHit)
			contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
		if (didHit)
		{
			contactInfo->collisionType = COLLISION_FACE_EDGE;
			contactInfo->normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(imposedPosition, contactPos));
			//contactInfo->normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(contactPos, pSphereCollider->pRigidBody->position));
			contactInfo->penetrationDistance = imposedRadius - sphereTraceVector3Length(sphereTraceVector3Subtract(imposedPosition, contactPos));
			contactInfo->pSphereCollider = NULL;
			contactInfo->pTriangleCollider = pTriangleCollider;
			contactInfo->point = contactPos;
			return 1;
		}

		//now we check the closest point
		int closestVertexDirection = sphereTraceColliderTriangleGetClosestTransformedVertexIndexToPoint(pTriangleCollider, imposedPosition);
		contactPos = pTriangleCollider->transformedVertices[closestVertexDirection];
		float dist = sphereTraceVector3Length(sphereTraceVector3Subtract(contactPos, imposedPosition));
		if (dist <= imposedRadius)
		{
			//printf("point collision\n");
			contactInfo->collisionType = COLLISION_FACE_POINT;
			contactInfo->normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(sphereTraceVector3Subtract(imposedPosition, contactPos), 1.0f / dist), 1);
			contactInfo->penetrationDistance = imposedRadius - dist;
			contactInfo->pSphereCollider = NULL;
			contactInfo->pTriangleCollider = pTriangleCollider;
			contactInfo->point = contactPos;
			return 1;
		}

		return 0;
	}
	else
		return 0;
}

b32 sphereTraceColliderSphereSphereCollisionTest(ST_SphereCollider* const pSphereColliderA, ST_SphereCollider* const pSphereColliderB, ST_SphereSphereContactInfo* const pContactInfo)
{
	ST_Vector3 dp = sphereTraceVector3Subtract(pSphereColliderB->pRigidBody->position, pSphereColliderA->pRigidBody->position);
	float dist = sphereTraceVector3Length(dp);
	float rab = pSphereColliderA->radius + pSphereColliderB->radius;
	if (dist <= rab)
	{
		pContactInfo->normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(dp, 1.0f / dist), 1);
		pContactInfo->pA = pSphereColliderA;
		pContactInfo->pB = pSphereColliderB;
		pContactInfo->penetrationDistance = rab - dist;
		ST_Vector3 pIntA = sphereTraceVector3AddAndScale(pSphereColliderA->pRigidBody->position, pContactInfo->normal.v, pSphereColliderA->radius);
		ST_Vector3 pIntB = sphereTraceVector3AddAndScale(pSphereColliderB->pRigidBody->position, sphereTraceVector3Negative(pContactInfo->normal.v), pSphereColliderB->radius);
		pContactInfo->point = sphereTraceVector3Average(pIntA, pIntB);
		return 1;
	}
	else
		return 0;
}