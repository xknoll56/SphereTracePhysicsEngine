#include "SphereTraceColliderPlane.h"
#include "SphereTraceGlobals.h"

void sphereTraceColliderPlaneSetTransformedVerticesAndEdges(ST_PlaneCollider* const pPlaneCollider)
{
	//forward right
	pPlaneCollider->transformedVertices[PLANE_VEREX_FORWARD_RIGHT] = sphereTraceVector3Add(sphereTraceVector3Add(pPlaneCollider->position, sphereTraceVector3Scale(pPlaneCollider->right.v, pPlaneCollider->xHalfExtent)), sphereTraceVector3Scale(pPlaneCollider->forward.v, pPlaneCollider->zHalfExtent));
	//forward left
	pPlaneCollider->transformedVertices[PLANE_VEREX_FORWARD_LEFT] = sphereTraceVector3Add(sphereTraceVector3Add(pPlaneCollider->position, sphereTraceVector3Scale(pPlaneCollider->right.v, -pPlaneCollider->xHalfExtent)), sphereTraceVector3Scale(pPlaneCollider->forward.v, pPlaneCollider->zHalfExtent));
	//back left
	pPlaneCollider->transformedVertices[PLANE_VEREX_BACK_LEFT] = sphereTraceVector3Add(sphereTraceVector3Add(pPlaneCollider->position, sphereTraceVector3Scale(pPlaneCollider->right.v, -pPlaneCollider->xHalfExtent)), sphereTraceVector3Scale(pPlaneCollider->forward.v, -pPlaneCollider->zHalfExtent));
	//back right
	pPlaneCollider->transformedVertices[PLANE_VEREX_BACK_RIGHT] = sphereTraceVector3Add(sphereTraceVector3Add(pPlaneCollider->position, sphereTraceVector3Scale(pPlaneCollider->right.v, pPlaneCollider->xHalfExtent)), sphereTraceVector3Scale(pPlaneCollider->forward.v, -pPlaneCollider->zHalfExtent));

	//right edge
	pPlaneCollider->transformedEdges[PLANE_EDGE_RIGHT] = sphereTraceEdgeConstruct(pPlaneCollider->transformedVertices[3], pPlaneCollider->transformedVertices[0]);
	//forward edge
	pPlaneCollider->transformedEdges[PLANE_EDGE_FORWARD] = sphereTraceEdgeConstruct(pPlaneCollider->transformedVertices[0], pPlaneCollider->transformedVertices[1]);
	//left edge
	pPlaneCollider->transformedEdges[PLANE_EDGE_LEFT] = sphereTraceEdgeConstruct(pPlaneCollider->transformedVertices[1], pPlaneCollider->transformedVertices[2]);
	//back edge
	pPlaneCollider->transformedEdges[PLANE_EDGE_BACK] = sphereTraceEdgeConstruct(pPlaneCollider->transformedVertices[2], pPlaneCollider->transformedVertices[3]);

}

ST_PlaneEdgeDirection sphereTraceColliderPlaneGetClosestTransformedEdgeToPoint(const ST_PlaneCollider* const pPlaneCollider, ST_Vector3 point)
{
	ST_Vector3 distVec = sphereTraceVector3Subtract(point, pPlaneCollider->position);
	float xDist = sphereTraceVector3Dot(distVec, pPlaneCollider->right.v) / pPlaneCollider->xHalfExtent;
	float zDist = sphereTraceVector3Dot(distVec, pPlaneCollider->forward.v) / pPlaneCollider->zHalfExtent;

	if (xDist > 0.0f)
	{
		if (fabsf(zDist) > xDist)
		{
			if (zDist > 0.0f)
			{
				//return forward edge
				return PLANE_EDGE_FORWARD;
			}
			else
			{
				//return back edge
				return PLANE_EDGE_BACK;
			}
		}
		else
		{
			//return right edge
			return PLANE_EDGE_RIGHT;
		}
	}
	else
	{
		if (fabsf(zDist) > -xDist)
		{
			if (zDist > 0.0f)
			{
				//return forward edge
				return PLANE_EDGE_FORWARD;
			}
			else
			{
				//return back edge
				return PLANE_EDGE_BACK;
			}
		}
		else
		{
			//return left edge
			return PLANE_EDGE_LEFT;
		}
	}
}

ST_PlaneEdgeDirection sphereTraceColliderPlaneGetClosestTransformedEdgeToSphereTrace(const ST_PlaneCollider* const pPlaneCollider, ST_Vector3 point, ST_Vector3 dir, float radius, ST_Vector3* closestPoint)
{
	float sForward = sphereTraceVector3Dot(dir, pPlaneCollider->forward.v);
	float sRight = sphereTraceVector3Dot(dir, pPlaneCollider->right.v);
	float s1, t1, s2, t2;
	float u1, v1, u2, v2;
	float dt1, dt2;
	ST_Vector3 right, fwd;
	ST_Vector3 start;
	ST_Vector3 closestPoint1, closestPoint2;
	ST_PlaneEdgeDirection e1, e2;
	if (sForward < 0.0f)
	{
		right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, pPlaneCollider->forward.v));
		fwd = sphereTraceVector3Cross(dir, right);
		float rightDist = sphereTraceVector3Dot(right, pPlaneCollider->transformedEdges[PLANE_EDGE_FORWARD].point1);
		float theta = acosf(fabsf(rightDist) / radius);
		start = sphereTraceVector3AddAndScale(point, fwd, radius * sinf(theta));
		ST_Vector3 edgeDir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pPlaneCollider->transformedEdges[PLANE_EDGE_FORWARD].point2, pPlaneCollider->transformedEdges[PLANE_EDGE_FORWARD].point1));
		sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(start, dir, pPlaneCollider->transformedEdges[PLANE_EDGE_FORWARD].point1, edgeDir, &s1, &t1);
		sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(point, dir, pPlaneCollider->transformedEdges[PLANE_EDGE_FORWARD].point1, edgeDir, &u1, &v1);
		float eMin, eMax;
		if (t1 > v1)
		{
			eMin = v1;
			eMax = t1;
		}
		else
		{
			eMin = t1;
			eMax = v1;
		}
		if (t1 >= pPlaneCollider->zHalfExtent)
		{
			if (eMin > pPlaneCollider->zHalfExtent * 2.0f)
				dt1 = FLT_MAX;
			else
				dt1 = pPlaneCollider->zHalfExtent * 2.0f - t1;
			closestPoint1 = pPlaneCollider->transformedEdges[PLANE_EDGE_FORWARD].point2;
		}
		else
		{
			if (eMax < 0.0f)
				dt1 = FLT_MAX;
			else
				dt1 = t1;
			closestPoint1 = pPlaneCollider->transformedEdges[PLANE_EDGE_FORWARD].point1;
		}
		e1 = PLANE_EDGE_FORWARD;
	}
	else
	{
		right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, sphereTraceVector3Negative(pPlaneCollider->forward.v)));
		fwd = sphereTraceVector3Cross(dir, right);
		float rightDist = sphereTraceVector3Dot(right, pPlaneCollider->transformedEdges[PLANE_EDGE_BACK].point1);
		float theta = acosf(fabsf(rightDist) / radius);
		start = sphereTraceVector3AddAndScale(point, fwd, radius * sinf(theta));
		ST_Vector3 edgeDir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pPlaneCollider->transformedEdges[PLANE_EDGE_BACK].point2, pPlaneCollider->transformedEdges[PLANE_EDGE_BACK].point1));
		sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(start, dir, pPlaneCollider->transformedEdges[PLANE_EDGE_BACK].point1, edgeDir, &s1, &t1);
		sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(point, dir, pPlaneCollider->transformedEdges[PLANE_EDGE_BACK].point1, edgeDir, &u1, &v1);
		float eMin, eMax;
		if (t1 > v1)
		{
			eMin = v1;
			eMax = t1;
		}
		else
		{
			eMin = t1;
			eMax = v1;
		}
		if (t1 >= pPlaneCollider->zHalfExtent)
		{
			if (eMin > pPlaneCollider->zHalfExtent * 2.0f)
				dt1 = FLT_MAX;
			else
				dt1 = pPlaneCollider->zHalfExtent * 2.0f - t1;
			closestPoint1 = pPlaneCollider->transformedEdges[PLANE_EDGE_BACK].point2;
		}
		else
		{
			if (eMax < 0.0f)
				dt1 = FLT_MAX;
			else
				dt1 = t1;
			closestPoint1 = pPlaneCollider->transformedEdges[PLANE_EDGE_BACK].point1;
		}
		e1 = PLANE_EDGE_BACK;
	}
	if (sRight < 0.0f)
	{
		right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, pPlaneCollider->right.v));
		fwd = sphereTraceVector3Cross(dir, right);
		float rightDist = sphereTraceVector3Dot(right, pPlaneCollider->transformedEdges[PLANE_EDGE_RIGHT].point1);
		float theta = acosf(fabsf(rightDist) / radius);
		start = sphereTraceVector3AddAndScale(point, fwd, radius * sinf(theta));
		ST_Vector3 edgeDir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pPlaneCollider->transformedEdges[PLANE_EDGE_RIGHT].point2, pPlaneCollider->transformedEdges[PLANE_EDGE_RIGHT].point1));
		sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(start, dir, pPlaneCollider->transformedEdges[PLANE_EDGE_RIGHT].point1, edgeDir, &s2, &t2);
		sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(point, dir, pPlaneCollider->transformedEdges[PLANE_EDGE_RIGHT].point1, edgeDir, &u2, &v2);
		float eMin, eMax;
		if (t2 > v2)
		{
			eMin = v2;
			eMax = t2;
		}
		else
		{
			eMin = t2;
			eMax = v2;
		}
		if (t2 >= pPlaneCollider->xHalfExtent)
		{
			if (eMin > pPlaneCollider->xHalfExtent * 2.0f)
				dt2 = FLT_MAX;
			else
				dt2 = pPlaneCollider->xHalfExtent * 2.0f - t2;
			closestPoint2 = pPlaneCollider->transformedEdges[PLANE_EDGE_RIGHT].point2;
		}
		else
		{
			if (eMax < 0.0f)
				dt2 = FLT_MAX;
			else
				dt2 = t2;
			closestPoint2 = pPlaneCollider->transformedEdges[PLANE_EDGE_RIGHT].point1;
		}
		e2 = PLANE_EDGE_RIGHT;
	}
	else
	{
		right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, sphereTraceVector3Negative(pPlaneCollider->right.v)));
		fwd = sphereTraceVector3Cross(dir, right);
		float rightDist = sphereTraceVector3Dot(right, pPlaneCollider->transformedEdges[PLANE_EDGE_LEFT].point1);
		float theta = acosf(fabsf(rightDist) / radius);
		start = sphereTraceVector3AddAndScale(point, fwd, radius * sinf(theta));
		ST_Vector3 edgeDir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pPlaneCollider->transformedEdges[PLANE_EDGE_LEFT].point2, pPlaneCollider->transformedEdges[PLANE_EDGE_LEFT].point1));
		sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(start, dir, pPlaneCollider->transformedEdges[PLANE_EDGE_LEFT].point1, edgeDir, &s2, &t2);
		sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(point, dir, pPlaneCollider->transformedEdges[PLANE_EDGE_LEFT].point1, edgeDir, &u2, &v2);
		float eMin, eMax;
		if (t2 > v2)
		{
			eMin = v2;
			eMax = t2;
		}
		else
		{
			eMin = t2;
			eMax = v2;
		}
		if (t2 >= pPlaneCollider->xHalfExtent)
		{
			if (eMin > pPlaneCollider->xHalfExtent * 2.0f)
				dt2 = FLT_MAX;
			else
				dt2 = pPlaneCollider->xHalfExtent * 2.0f - t2;
			closestPoint2 = pPlaneCollider->transformedEdges[PLANE_EDGE_LEFT].point2;
		}
		else
		{
			if (eMax < 0.0f)
				dt2 = FLT_MAX;
			else
				dt2 = t2;
			closestPoint2 = pPlaneCollider->transformedEdges[PLANE_EDGE_LEFT].point1;
		}
		e2 = PLANE_EDGE_LEFT;
	}

	if (dt1 <= dt2)
	{
		*closestPoint = closestPoint1;
		return e1;
	}
	else
	{
		*closestPoint = closestPoint2;
		return e2;
	}
}

ST_PlaneVertexDirection sphereTraceColliderPlaneGetClosestTransformedVertexToPoint(const ST_PlaneCollider* const pPlaneCollider, ST_Vector3 point)
{
	ST_Vector3 distVec = sphereTraceVector3Subtract(point, pPlaneCollider->position);
	float xDist = sphereTraceVector3Dot(distVec, pPlaneCollider->right.v);
	float zDist = sphereTraceVector3Dot(distVec, pPlaneCollider->forward.v);
	if (xDist > 0.0f)
	{
		if (zDist > 0.0f)
		{
			//return forward right
			return PLANE_VEREX_FORWARD_RIGHT;
		}
		else
		{
			//return back right
			return  PLANE_VEREX_BACK_RIGHT;
		}
	}
	else
	{
		if (zDist > 0.0f)
		{
			//return forward left
			return  PLANE_VEREX_FORWARD_LEFT;
		}
		else
		{
			//return back left
			return  PLANE_VEREX_BACK_LEFT;
		}
	}
}

void sphereTraceColliderPlaneSetAABBExtents(ST_PlaneCollider* const pPlaneCollider)
{
	//set aabb extents
	pPlaneCollider->aabb.halfExtents.x = sphereTraceVector3Subtract(pPlaneCollider->transformedVertices[sphereTraceColliderPlaneGetClosestTransformedVertexToPoint(pPlaneCollider, sphereTraceVector3Add(pPlaneCollider->position, gVector3Right))], pPlaneCollider->position).x;
	pPlaneCollider->aabb.halfExtents.y = sphereTraceVector3Subtract(pPlaneCollider->transformedVertices[sphereTraceColliderPlaneGetClosestTransformedVertexToPoint(pPlaneCollider, sphereTraceVector3Add(pPlaneCollider->position, gVector3Up))], pPlaneCollider->position).y;
	pPlaneCollider->aabb.halfExtents.z = sphereTraceVector3Subtract(pPlaneCollider->transformedVertices[sphereTraceColliderPlaneGetClosestTransformedVertexToPoint(pPlaneCollider, sphereTraceVector3Add(pPlaneCollider->position, gVector3Forward))], pPlaneCollider->position).z;
}

void sphereTraceColliderPlaneAABBSetTransformedVertices(ST_PlaneCollider* const pPlaneCollider)
{
	pPlaneCollider->aabb.rightTopForwardsTransformedVertex = sphereTraceVector3Add(pPlaneCollider->position, pPlaneCollider->aabb.halfExtents);
	pPlaneCollider->aabb.leftDownBackTransformedVertex = sphereTraceVector3Subtract(pPlaneCollider->position, pPlaneCollider->aabb.halfExtents);
}

ST_PlaneCollider sphereTraceColliderPlaneConstruct(ST_Vector3 normalDir, float angle, float xHalfExtent, float zHalfExtent, ST_Vector3 position)
{
	ST_PlaneCollider planeCollider;
	planeCollider.position = position;
	planeCollider.normal = sphereTraceDirectionConstructNormalized(normalDir);
	float dot = sphereTraceVector3Dot(planeCollider.normal.v, gVector3Forward);
	if (dot == 1.0f)
	{
		planeCollider.right = sphereTraceDirectionConstructNormalized(gVector3Right);
	}
	else if (dot == -1.0f)
	{
		planeCollider.right = sphereTraceDirectionConstructNormalized(sphereTraceVector3Negative(gVector3Right));
	}
	else
		planeCollider.right = sphereTraceDirectionConstructNormalized(sphereTraceVector3Cross(planeCollider.normal.v, gVector3Forward));
	ST_Quaternion rotation = sphereTraceQuaternionFromAngleAxis(planeCollider.normal.v, angle);
	planeCollider.right = sphereTraceDirectionConstruct(sphereTraceVector3RotatePoint(planeCollider.right.v, rotation), 1);
	planeCollider.forward = sphereTraceDirectionConstruct(sphereTraceVector3Cross(planeCollider.right.v, planeCollider.normal.v), 1);
	planeCollider.xHalfExtent = xHalfExtent;
	planeCollider.zHalfExtent = zHalfExtent;
	ST_Matrix4 rotMat = sphereTraceMatrixConstructFromRightForwardUp(planeCollider.right.v, planeCollider.normal.v, planeCollider.forward.v);
	planeCollider.rotation = sphereTraceQuaternionNormalize(sphereTraceMatrixQuaternionFromRotationMatrix(rotMat));

	//retreive the adjusted rotation matrix, matrix from quaternion is not perfect so the directional vectors change slightly
	rotMat = sphereTraceMatrixFromQuaternion(planeCollider.rotation);
	planeCollider.right = sphereTraceDirectionConstruct(sphereTraceVector3GetLocalXAxisFromRotationMatrix(rotMat), 1);
	planeCollider.normal = sphereTraceDirectionConstruct(sphereTraceVector3GetLocalYAxisFromRotationMatrix(rotMat), 1);
	planeCollider.forward = sphereTraceDirectionConstruct(sphereTraceVector3GetLocalZAxisFromRotationMatrix(rotMat), 1);
	sphereTraceColliderPlaneSetTransformedVerticesAndEdges(&planeCollider);
	sphereTraceColliderPlaneSetAABBExtents(&planeCollider);
	sphereTraceColliderPlaneAABBSetTransformedVertices(&planeCollider);

	return planeCollider;
}

ST_PlaneCollider sphereTraceColliderPlaneConstructWithRotationMatrix(ST_Matrix4 rotMat, float xHalfExtent, float zHalfExtent, ST_Vector3 position)
{
	ST_PlaneCollider planeCollider;
	planeCollider.position = position;
	planeCollider.xHalfExtent = xHalfExtent;
	planeCollider.zHalfExtent = zHalfExtent;
	planeCollider.right = sphereTraceDirectionConstruct(sphereTraceVector3GetLocalXAxisFromRotationMatrix(rotMat), 0);
	planeCollider.normal = sphereTraceDirectionConstruct(sphereTraceVector3GetLocalYAxisFromRotationMatrix(rotMat), 0);
	planeCollider.forward = sphereTraceDirectionConstruct(sphereTraceVector3GetLocalZAxisFromRotationMatrix(rotMat), 0);
	planeCollider.rotation = sphereTraceMatrixQuaternionFromRotationMatrix(rotMat);
	sphereTraceColliderPlaneSetTransformedVerticesAndEdges(&planeCollider);
	sphereTraceColliderPlaneSetAABBExtents(&planeCollider);
	sphereTraceColliderPlaneAABBSetTransformedVertices(&planeCollider);

	return planeCollider;
}


void sphereTraceColliderInfinitePlaneRayTrace(ST_Vector3 from, ST_Direction dir, ST_Direction planeNormal, ST_Vector3 pointOnPlane, ST_RayTraceData* const pRaycastData)
{
	pRaycastData->startPoint = from;
	if (sphereTraceVector3Equal(from, pointOnPlane))
	{
		pRaycastData->normal = planeNormal;
		pRaycastData->distance = 0.0f;
		pRaycastData->hitPoint = pointOnPlane;
		return;
	}
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	ST_Vector3 dp = sphereTraceVector3Subtract(pointOnPlane, from);
	pRaycastData->normal = planeNormal;
	float dirDotNormal = sphereTraceVector3Dot(dir.v, planeNormal.v);
	pRaycastData->distance = sphereTraceVector3Dot(dp, pRaycastData->normal.v) / dirDotNormal;
	pRaycastData->hitPoint = sphereTraceVector3Add(from, sphereTraceVector3Scale(dir.v, pRaycastData->distance));
	//sphereTraceVector3NormalizeByRef(&dp);
	if (sphereTraceVector3Dot(sphereTraceVector3Subtract(pRaycastData->hitPoint, from), pRaycastData->normal.v) > 0.0f)
	{
		pRaycastData->normal = sphereTraceDirectionNegative(planeNormal);
	}
}

b32 sphereTraceColliderPlaneRayTrace(ST_Vector3 from, ST_Direction dir, const ST_PlaneCollider* const pPlaneCollider, ST_RayTraceData* const pRaycastData)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	sphereTraceColliderInfinitePlaneRayTrace(from, dir, pPlaneCollider->normal, pPlaneCollider->position, pRaycastData);
	if (pRaycastData->distance >= 0.0f)
	{
		if (fpclassify(pRaycastData->distance) == FP_INFINITE)
			return 0;
		pRaycastData->startPoint = from;
		ST_Vector3 vectorFromCenter = sphereTraceVector3Subtract(pRaycastData->hitPoint, pPlaneCollider->position);
		float xDist = sphereTraceVector3Dot(vectorFromCenter, pPlaneCollider->right.v);
		if (fabsf(xDist) > pPlaneCollider->xHalfExtent)
			return 0;
		float zDist = sphereTraceVector3Dot(vectorFromCenter, pPlaneCollider->forward.v);
		if (fabsf(zDist) > pPlaneCollider->zHalfExtent)
			return 0;
		return 1;
	}
	return 0;
}

b32 sphereTraceColliderPlaneIsProjectedPointContained(ST_Vector3 projectedPoint, const ST_PlaneCollider* const pPlaneCollider)
{
	sphereTraceVector3SubtractByRef(&projectedPoint, pPlaneCollider->position);
	float dist = sphereTraceVector3Dot(projectedPoint, pPlaneCollider->right.v);
	if (fabsf(dist) > pPlaneCollider->xHalfExtent)
	{
		return 0;
	}
	dist = sphereTraceVector3Dot(projectedPoint, pPlaneCollider->forward.v);
	if (fabsf(dist) > pPlaneCollider->zHalfExtent)
	{
		return 0;
	}
	return 1;
}

b32 sphereTraceColliderInfinitePlaneSphereTrace(ST_Vector3 from, ST_Direction dir, float radius, ST_Vector3 pointOnPlane, ST_Direction planeNormal, ST_SphereTraceData* const pSphereTraceData)
{
	ST_Contact contact;
	if (sphereTraceColliderInfinitePlaneImposedSphereCollisionTest(from, radius, planeNormal, pointOnPlane, &contact))
	{
		pSphereTraceData->radius = radius;
		pSphereTraceData->sphereCenter = from;
		pSphereTraceData->traceDistance = 0.0f;
		pSphereTraceData->rayTraceData.distance = sphereTraceVector3Distance(from, contact.point);
		pSphereTraceData->rayTraceData.hitPoint = contact.point;
		pSphereTraceData->rayTraceData.normal = contact.normal;
		pSphereTraceData->rayTraceData.startPoint = from;
		return 1;
	}
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	sphereTraceColliderInfinitePlaneRayTrace(from, dir, planeNormal, pointOnPlane, &pSphereTraceData->rayTraceData);
	if (pSphereTraceData->rayTraceData.distance < 0.0f || isinf(pSphereTraceData->rayTraceData.distance))
		return 0;
	float alpha;
	float dot = sphereTraceVector3Dot(dir.v, pSphereTraceData->rayTraceData.normal.v);
	float hypotinus;
	ST_Vector3 orthogonal = pSphereTraceData->rayTraceData.normal.v;
	if (fabsf(fabsf(dot) - 1) < COLLIDER_TOLERANCE)
	{
		alpha = 0.0f;
		hypotinus = radius;
	}
	else
	{
		orthogonal = sphereTraceVector3Cross(pSphereTraceData->rayTraceData.normal.v, dir.v);
		orthogonal = sphereTraceVector3Cross(pSphereTraceData->rayTraceData.normal.v, orthogonal);
		sphereTraceVector3NormalizeByRef(&orthogonal);
		alpha = acosf(sphereTraceVector3Dot(sphereTraceVector3Negative(dir.v), orthogonal));
		hypotinus = radius / sinf(alpha);
	}

	//if (sphereTraceVector3EpsilonEquals(dir, sphereTraceVector3Negative(orthogonal), COLLIDER_TOLERANCE))
	//{

	//}
	//

	pSphereTraceData->sphereCenter = sphereTraceVector3AddAndScale(from, dir.v, pSphereTraceData->rayTraceData.distance - hypotinus);

	pSphereTraceData->rayTraceData.hitPoint = sphereTraceVector3AddAndScale(pSphereTraceData->sphereCenter, sphereTraceVector3Negative(pSphereTraceData->rayTraceData.normal.v), radius);
	pSphereTraceData->rayTraceData.startPoint = from;
	pSphereTraceData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pSphereTraceData->rayTraceData.hitPoint, from));
	pSphereTraceData->radius = radius;
	pSphereTraceData->traceDistance = sphereTraceVector3Distance(pSphereTraceData->sphereCenter, pSphereTraceData->rayTraceData.startPoint);
	//pSphereTraceData->rayTraceData.normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(pSphereTraceData->sphereCenter, pSphereTraceData->rayTraceData.hitPoint));
	return 1;


}

b32 sphereTraceColliderPlaneSphereTrace(ST_Vector3 from, ST_Direction dir, float radius, ST_PlaneCollider* const pPlaneCollider, ST_SphereTraceData* const pSphereTraceData)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	if (sphereTraceColliderInfinitePlaneSphereTrace(from, dir, radius, pPlaneCollider->position, pPlaneCollider->normal, pSphereTraceData))
	{
		if (sphereTraceColliderPlaneIsProjectedPointContained(pSphereTraceData->rayTraceData.hitPoint, pPlaneCollider))
		{
			return 1;
		}
	}
	float closestEdgePointDist = FLT_MAX;
	ST_SphereTraceData datTest;
	if (sphereTraceColliderEdgeSphereTrace(from, dir, radius, &pPlaneCollider->transformedEdges[0], &datTest))
	{
		closestEdgePointDist = datTest.traceDistance;
		*pSphereTraceData = datTest;
	}
	if (sphereTraceColliderEdgeSphereTrace(from, dir, radius, &pPlaneCollider->transformedEdges[1], &datTest))
	{
		if (datTest.traceDistance < closestEdgePointDist)
		{
			closestEdgePointDist = datTest.traceDistance;
			*pSphereTraceData = datTest;
		}
	}
	if (sphereTraceColliderEdgeSphereTrace(from, dir, radius, &pPlaneCollider->transformedEdges[2], &datTest))
	{
		if (datTest.traceDistance < closestEdgePointDist)
		{
			closestEdgePointDist = datTest.traceDistance;
			*pSphereTraceData = datTest;
		}
	}
	if (sphereTraceColliderEdgeSphereTrace(from, dir, radius, &pPlaneCollider->transformedEdges[3], &datTest))
	{
		if (datTest.traceDistance < closestEdgePointDist)
		{
			closestEdgePointDist = datTest.traceDistance;
			*pSphereTraceData = datTest;
		}
	}
	if (closestEdgePointDist < FLT_MAX)
		return 1;
	return 0;
}