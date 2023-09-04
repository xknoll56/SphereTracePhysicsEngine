#include "SphereTraceColliderTriangle.h"

int sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToPoint(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point)
{
	ST_Vector3 dp1 = sphereTraceVector3Subtract(point, pTriangleCollider->transformedEdges[0].point1);
	ST_Vector3 dp2 = sphereTraceVector3Subtract(point, pTriangleCollider->transformedEdges[1].point1);
	ST_Vector3 dp3 = sphereTraceVector3Subtract(point, pTriangleCollider->transformedEdges[2].point1);
	//float normalizedDir0 = sphereTraceVector3Dot(pTriangleCollider->edgeCenterDirs[0], dp) / pTriangleCollider->edgeCenterDists[0];
	//float normalizedDir1 = sphereTraceVector3Dot(pTriangleCollider->edgeCenterDirs[1], dp) / pTriangleCollider->edgeCenterDists[1];
	//float normalizedDir2 = sphereTraceVector3Dot(pTriangleCollider->edgeCenterDirs[2], dp) / pTriangleCollider->edgeCenterDists[2];
	float normalizedDir0 = sphereTraceVector3Dot(pTriangleCollider->edgeOrthogonalDirs[0], dp1);
	float normalizedDir1 = sphereTraceVector3Dot(pTriangleCollider->edgeOrthogonalDirs[1], dp2);
	float normalizedDir2 = sphereTraceVector3Dot(pTriangleCollider->edgeOrthogonalDirs[2], dp3);
	if ((normalizedDir0 >= normalizedDir1) && (normalizedDir0 >= normalizedDir2))
	{
		return 0;
	}
	else if (normalizedDir1 >= normalizedDir2)
	{
		return 1;
	}
	else
	{
		return 2;
	}

}

int sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToLine(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point, ST_Vector3 dir)
{
	float s1, t1, s2, t2, s3, t3;
	sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(point, dir, pTriangleCollider->transformedEdges[0].point1, pTriangleCollider->edgeDirs[0], &s1, &t1);
	float s = sphereTraceVector3Dot(dir, pTriangleCollider->edgeOrthogonalDirs[0]);
	if (t1 <0.0f || t1 > pTriangleCollider->edgeDists[0] || s > 0.0f || fpclassify(s1) == FP_NAN)
		s1 = FLT_MAX;
	sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(point, dir, pTriangleCollider->transformedEdges[1].point1, pTriangleCollider->edgeDirs[1], &s2, &t2);
	s = sphereTraceVector3Dot(dir, pTriangleCollider->edgeOrthogonalDirs[1]);
	if (t2 < 0.0f || t2 > pTriangleCollider->edgeDists[1] || s > 0.0f || fpclassify(s2) == FP_NAN)
		s2 = FLT_MAX;
	sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(point, dir, pTriangleCollider->transformedEdges[2].point1, pTriangleCollider->edgeDirs[2], &s3, &t3);
	s = sphereTraceVector3Dot(dir, pTriangleCollider->edgeOrthogonalDirs[2]);
	if (t3 < 0.0f || t3 > pTriangleCollider->edgeDists[2] || s > 0.0f || fpclassify(s3) == FP_NAN)
		s3 = FLT_MAX;
	if ((s1 <= s2) && (s1 <= s3))
	{
		return 0;
	}
	else if (s2 <= s3)
	{
		return 1;
	}
	else
	{
		return 2;
	}

}

//TODO FIX THIS AWFUL CODE!
int sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToSphereTrace(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point, float radius, ST_Vector3 dir, ST_Vector3* const closestPoint)
{
	float s1, t1, s2, t2, s3, t3, dt1, dt2, dt3;
	float u1, v1, u2, v2, u3, v3;
	ST_Vector3 closestPoint1, closestPoint2, closestPoint3;
	ST_Vector3 right, fwd;
	right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, pTriangleCollider->edgeDirs[0]));
	fwd = sphereTraceVector3Cross(dir, right);
	float rightDist = sphereTraceVector3Dot(right, pTriangleCollider->transformedEdges[0].point1);
	float theta = acosf(fabsf(rightDist) / radius);
	ST_Vector3 start = sphereTraceVector3AddAndScale(point, fwd, radius * sinf(theta));
	sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(start, dir, pTriangleCollider->transformedEdges[0].point1, pTriangleCollider->edgeDirs[0], &s1, &t1);
	sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(point, dir, pTriangleCollider->transformedEdges[0].point1, pTriangleCollider->edgeDirs[0], &u1, &v1);
	b32 increasing = 0;
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
	float s = sphereTraceVector3Dot(dir, pTriangleCollider->edgeOrthogonalDirs[0]);
	if (isnan(t1))
		t1 = v1;

	{
		if (t1 >= pTriangleCollider->edgeDists[0] * 0.5f)
		{
			if (eMin > pTriangleCollider->edgeDists[0])
				u1 = FLT_MAX;
			else
				dt1 = pTriangleCollider->edgeDists[0] - t1;
			closestPoint1 = pTriangleCollider->transformedEdges[0].point2;
		}
		else
		{
			if (eMax < 0.0f)
				u1 = FLT_MAX;
			else
				dt1 = t1;
			closestPoint1 = pTriangleCollider->transformedEdges[0].point1;
		}
	}
	if (s > 0.0f)
	{
		dt1 = FLT_MAX;
		u1 = FLT_MAX;
	}
	right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, pTriangleCollider->edgeDirs[1]));
	fwd = sphereTraceVector3Cross(dir, right);
	rightDist = sphereTraceVector3Dot(right, pTriangleCollider->transformedEdges[1].point1);
	theta = acosf(fabsf(rightDist) / radius);
	start = sphereTraceVector3AddAndScale(point, fwd, radius * sinf(theta));
	sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(start, dir, pTriangleCollider->transformedEdges[1].point1, pTriangleCollider->edgeDirs[1], &s2, &t2);
	sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(point, dir, pTriangleCollider->transformedEdges[1].point1, pTriangleCollider->edgeDirs[1], &u2, &v2);
	//d2 = s2;
	s = sphereTraceVector3Dot(dir, pTriangleCollider->edgeOrthogonalDirs[1]);
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
	if (isnan(t2))
		t2 = v2;

	{
		if (t2 >= pTriangleCollider->edgeDists[1] * 0.5f)
		{
			if (eMin > pTriangleCollider->edgeDists[1])
				u2 = FLT_MAX;
			else
				dt2 = pTriangleCollider->edgeDists[1] - t2;
			closestPoint2 = pTriangleCollider->transformedEdges[1].point2;
		}
		else
		{
			if (eMax < 0.0f)
				u2 = FLT_MAX;
			else
				dt2 = t2;
			closestPoint2 = pTriangleCollider->transformedEdges[1].point1;
		}
	}
	if (s > 0.0f)
	{
		dt2 = FLT_MAX;
		u2 = FLT_MAX;
	}
	right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, pTriangleCollider->edgeDirs[2]));
	fwd = sphereTraceVector3Cross(dir, right);
	rightDist = sphereTraceVector3Dot(right, pTriangleCollider->transformedEdges[2].point1);
	theta = acosf(fabsf(rightDist) / radius);
	start = sphereTraceVector3AddAndScale(point, fwd, radius * sinf(theta));
	sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(start, dir, pTriangleCollider->transformedEdges[2].point1, pTriangleCollider->edgeDirs[2], &s3, &t3);
	sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(point, dir, pTriangleCollider->transformedEdges[2].point1, pTriangleCollider->edgeDirs[2], &u3, &v3);
	//d3 = s3;
	s = sphereTraceVector3Dot(dir, pTriangleCollider->edgeOrthogonalDirs[2]);
	if (t3 > v3)
	{
		eMin = v3;
		eMax = t3;
	}
	else
	{
		eMin = t3;
		eMax = v3;
	}
	if (isnan(t3))
		t3 = v3;

	{
		if (t3 >= pTriangleCollider->edgeDists[2] * 0.5f)
		{
			if (eMin > pTriangleCollider->edgeDists[2])
				u3 = FLT_MAX;
			else
				dt3 = pTriangleCollider->edgeDists[2] - t3;
			closestPoint3 = pTriangleCollider->transformedEdges[2].point2;
		}
		else
		{
			if (eMax < 0.0f)
				u3 = FLT_MAX;
			else
				dt3 = t3;
			closestPoint3 = pTriangleCollider->transformedEdges[2].point1;
		}
	}
	if (s > 0.0f)
	{
		dt3 = FLT_MAX;
		u3 = FLT_MAX;
	}
	//if (isnan(dt1))
	//	dt1 = 0.0f;
	//if (isnan(dt2))
	//	dt2 = 0.0f;
	//if (isnan(dt3))
	//	dt3 = 0.0f;
	if (u1 < 0)
		u1 = FLT_MAX;
	if (u2 < 0)
		u2 = FLT_MAX;
	if (u3 < 0)
		u3 = FLT_MAX;
	if ((u1 <= u2) && (u1 <= u3))
	{
		*closestPoint = closestPoint1;
		return 0;
	}
	else if (u2 <= u3)
	{
		*closestPoint = closestPoint2;
		return 1;
	}
	else
	{
		*closestPoint = closestPoint3;
		return 2;
	}

}

int sphereTraceColliderTriangleGetClosestTransformedVertexIndexToPoint(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point)
{
	ST_Vector3 dp = sphereTraceVector3Subtract(point, pTriangleCollider->centroid);
	float normalizedDir0 = sphereTraceVector3Dot(pTriangleCollider->vertexDirs[0], dp) / pTriangleCollider->vertexDists[0];
	float normalizedDir1 = sphereTraceVector3Dot(pTriangleCollider->vertexDirs[1], dp) / pTriangleCollider->vertexDists[1];
	float normalizedDir2 = sphereTraceVector3Dot(pTriangleCollider->vertexDirs[2], dp) / pTriangleCollider->vertexDists[2];
	if ((normalizedDir0 >= normalizedDir1) && (normalizedDir0 >= normalizedDir2))
	{
		return 0;
	}
	else if (normalizedDir1 >= normalizedDir2)
	{
		return 1;
	}
	else
	{
		return 2;
	}

}

void sphereTraceColliderTriangleSetAABB(ST_TriangleCollider* const pTriangleCollider)
{
	pTriangleCollider->aabb.rightTopForwardsTransformedVertex = sphereTraceVector3Construct(fmaxf(pTriangleCollider->transformedVertices[0].x, fmaxf(pTriangleCollider->transformedVertices[1].x, pTriangleCollider->transformedVertices[2].x)),
		fmaxf(pTriangleCollider->transformedVertices[0].y, fmaxf(pTriangleCollider->transformedVertices[1].y, pTriangleCollider->transformedVertices[2].y)),
		fmaxf(pTriangleCollider->transformedVertices[0].z, fmaxf(pTriangleCollider->transformedVertices[1].z, pTriangleCollider->transformedVertices[2].z)));

	pTriangleCollider->aabb.leftDownBackTransformedVertex = sphereTraceVector3Construct(fminf(pTriangleCollider->transformedVertices[0].x, fminf(pTriangleCollider->transformedVertices[1].x, pTriangleCollider->transformedVertices[2].x)),
		fminf(pTriangleCollider->transformedVertices[0].y, fminf(pTriangleCollider->transformedVertices[1].y, pTriangleCollider->transformedVertices[2].y)),
		fminf(pTriangleCollider->transformedVertices[0].z, fminf(pTriangleCollider->transformedVertices[1].z, pTriangleCollider->transformedVertices[2].z)));

	pTriangleCollider->aabb.halfExtents = sphereTraceVector3Subtract(pTriangleCollider->aabb.rightTopForwardsTransformedVertex,
		sphereTraceVector3Average(pTriangleCollider->aabb.rightTopForwardsTransformedVertex, pTriangleCollider->aabb.leftDownBackTransformedVertex));
}

void sphereTraceColliderTriangleSetVertexAndEdgeData(ST_TriangleCollider* const pTriangleCollider, ST_Vector3 v1, ST_Vector3 v2, ST_Vector3 v3)
{
	pTriangleCollider->transformedVertices[0] = v1;
	pTriangleCollider->transformedVertices[1] = v2;
	pTriangleCollider->transformedVertices[2] = v3;
	pTriangleCollider->centroid = sphereTraceVector3Construct((v1.x + v2.x + v3.x) / 3.0f, (v1.y + v2.y + v3.y) / 3.0f, (v1.z + v2.z + v3.z) / 3.0f);
	pTriangleCollider->normal = sphereTraceVector3Normalize(sphereTraceVector3Cross(sphereTraceVector3Subtract(v3, v2), sphereTraceVector3Subtract(v1, v2)));
	pTriangleCollider->transformedEdges[0] = sphereTraceEdgeConstruct(v3, v1);
	pTriangleCollider->transformedEdges[1] = sphereTraceEdgeConstruct(v1, v2);
	pTriangleCollider->transformedEdges[2] = sphereTraceEdgeConstruct(v2, v3);
	pTriangleCollider->vertexDirs[0] = sphereTraceVector3Normalize(sphereTraceVector3Subtract(v1, pTriangleCollider->centroid));
	pTriangleCollider->vertexDirs[1] = sphereTraceVector3Normalize(sphereTraceVector3Subtract(v2, pTriangleCollider->centroid));
	pTriangleCollider->vertexDirs[2] = sphereTraceVector3Normalize(sphereTraceVector3Subtract(v3, pTriangleCollider->centroid));
	pTriangleCollider->vertexDists[0] = sphereTraceVector3Length(sphereTraceVector3Subtract(v1, pTriangleCollider->centroid));
	pTriangleCollider->vertexDists[1] = sphereTraceVector3Length(sphereTraceVector3Subtract(v2, pTriangleCollider->centroid));
	pTriangleCollider->vertexDists[2] = sphereTraceVector3Length(sphereTraceVector3Subtract(v3, pTriangleCollider->centroid));
	pTriangleCollider->edgeCenterDists[0] = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereTraceVector3Average(pTriangleCollider->transformedEdges[0].point1, pTriangleCollider->transformedEdges[0].point2), pTriangleCollider->centroid));
	pTriangleCollider->edgeCenterDists[1] = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereTraceVector3Average(pTriangleCollider->transformedEdges[1].point1, pTriangleCollider->transformedEdges[1].point2), pTriangleCollider->centroid));
	pTriangleCollider->edgeCenterDists[2] = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereTraceVector3Average(pTriangleCollider->transformedEdges[2].point1, pTriangleCollider->transformedEdges[2].point2), pTriangleCollider->centroid));
	pTriangleCollider->edgeDirs[0] = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pTriangleCollider->transformedEdges[0].point2, pTriangleCollider->transformedEdges[0].point1));
	pTriangleCollider->edgeDirs[1] = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pTriangleCollider->transformedEdges[1].point2, pTriangleCollider->transformedEdges[1].point1));
	pTriangleCollider->edgeDirs[2] = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pTriangleCollider->transformedEdges[2].point2, pTriangleCollider->transformedEdges[2].point1));
	pTriangleCollider->edgeDists[0] = sphereTraceVector3Length(sphereTraceVector3Subtract(pTriangleCollider->transformedEdges[0].point2, pTriangleCollider->transformedEdges[0].point1));
	pTriangleCollider->edgeDists[1] = sphereTraceVector3Length(sphereTraceVector3Subtract(pTriangleCollider->transformedEdges[1].point2, pTriangleCollider->transformedEdges[1].point1));
	pTriangleCollider->edgeDists[2] = sphereTraceVector3Length(sphereTraceVector3Subtract(pTriangleCollider->transformedEdges[2].point2, pTriangleCollider->transformedEdges[2].point1));
	pTriangleCollider->edgeOrthogonalDirs[0] = sphereTraceVector3Normalize(sphereTraceVector3Cross(pTriangleCollider->edgeDirs[0], pTriangleCollider->normal));
	pTriangleCollider->edgeOrthogonalDirs[1] = sphereTraceVector3Normalize(sphereTraceVector3Cross(pTriangleCollider->edgeDirs[1], pTriangleCollider->normal));
	pTriangleCollider->edgeOrthogonalDirs[2] = sphereTraceVector3Normalize(sphereTraceVector3Cross(pTriangleCollider->edgeDirs[2], pTriangleCollider->normal));
	sphereTraceColliderTriangleSetAABB(pTriangleCollider);
}

ST_TriangleCollider sphereTraceColliderTriangleConstruct(ST_Vector3 v1, ST_Vector3 v2, ST_Vector3 v3)
{
	ST_TriangleCollider triangleCollider;
	sphereTraceColliderTriangleSetVertexAndEdgeData(&triangleCollider, v1, v2, v3);
	triangleCollider.ignoreCollisions = 0;
	return triangleCollider;
}

b32 sphereTraceColliderTriangleIsProjectedPointContained(ST_Vector3 projectedPoint, const ST_TriangleCollider* const pTriangleCollider)
{
	float dot1 = sphereTraceVector3Dot(pTriangleCollider->normal, sphereTraceVector3Cross(sphereTraceVector3Subtract(pTriangleCollider->transformedVertices[0], pTriangleCollider->transformedVertices[2]),
		sphereTraceVector3Subtract(projectedPoint, pTriangleCollider->transformedVertices[2])));
	float dot2 = sphereTraceVector3Dot(pTriangleCollider->normal, sphereTraceVector3Cross(sphereTraceVector3Subtract(pTriangleCollider->transformedVertices[1], pTriangleCollider->transformedVertices[0]),
		sphereTraceVector3Subtract(projectedPoint, pTriangleCollider->transformedVertices[0])));
	float dot3 = sphereTraceVector3Dot(pTriangleCollider->normal, sphereTraceVector3Cross(sphereTraceVector3Subtract(pTriangleCollider->transformedVertices[2], pTriangleCollider->transformedVertices[1]),
		sphereTraceVector3Subtract(projectedPoint, pTriangleCollider->transformedVertices[1])));
	dot1 = copysignf(1.0f, dot1);
	dot2 = copysignf(1.0f, dot2);
	dot3 = copysignf(1.0f, dot3);
	if ((dot1 == dot2) && (dot1 == dot3))
	{
		return 1;
	}
	return 0;
}

b32 sphereTraceColliderTriangleRayTrace(ST_Vector3 from, ST_Vector3 dir, const ST_TriangleCollider* const pTriangleCollider, ST_RayTraceData* const pRaycastData)
{
	sphereTraceColliderInfinitePlaneRayTrace(from, dir, pTriangleCollider->normal, pTriangleCollider->centroid, pRaycastData);
	//dir = sphereTraceVector3Normalize(dir);
	//float dirDotNormal = sphereTraceVector3Dot(dir, pTriangleCollider->normal);
	//pRaycastData->normal = pTriangleCollider->normal;
	//if (dirDotNormal > 0.0f)
	//{
	//	pRaycastData->normal = sphereTraceVector3Negative(pRaycastData->normal);
	//	dirDotNormal = -dirDotNormal;
	//}
	//pRaycastData->distance = sphereTraceVector3Dot(sphereTraceVector3Subtract(pTriangleCollider->centroid, from), pRaycastData->normal) / dirDotNormal;
	//pRaycastData->hitPoint = sphereTraceVector3Add(from, sphereTraceVector3Scale(dir, pRaycastData->distance));
	if (pRaycastData->distance >= 0.0f)
	{
		if (fpclassify(pRaycastData->distance) == FP_INFINITE)
			return 0;
		pRaycastData->startPoint = from;
		pRaycastData->hitPoint = sphereTraceVector3Add(from, sphereTraceVector3Scale(dir, pRaycastData->distance));

		return sphereTraceColliderTriangleIsProjectedPointContained(pRaycastData->hitPoint, pTriangleCollider);

	}
	return 0;
}

b32 sphereTraceColliderTriangleSphereTrace(ST_Vector3 from, ST_Vector3 dir, float radius, ST_TriangleCollider* const pTriangleCollider, ST_SphereTraceData* const pSphereCastData)
{
	ST_SphereTriangleContactInfo contactInfo;
	if (sphereTraceColliderTriangleImposedSphereCollisionTest(pTriangleCollider, from, radius, &contactInfo))
	{
		pSphereCastData->radius = radius;
		pSphereCastData->rayTraceData.startPoint = from;
		pSphereCastData->sphereCenter = from;
		pSphereCastData->rayTraceData.hitPoint = contactInfo.point;
		pSphereCastData->rayTraceData.normal = contactInfo.normal;
		pSphereCastData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(from, contactInfo.point));
		pSphereCastData->traceDistance = 0.0f;
		return 1;
	}
	else
	{

		dir = sphereTraceVector3Normalize(dir);
		if (sphereTraceVector3Nan(dir))
			return 0;
		sphereTraceColliderTriangleRayTrace(from, dir, pTriangleCollider, &pSphereCastData->rayTraceData);
		//if (fpclassify(pSphereCastData->rayTraceData.distance) == -FP_INFINITE)
		//	return 0;
		//ST_Vector3 fromToPlane = sphereTraceVector3Subtract(pTriangleCollider->centroid, from);
		//if (sphereTraceVector3Dot(dir, fromToPlane) < 0.0f)
		//	return 0;
		float alpha;
		b32 alphaInvalid = 0;
		float dot = sphereTraceVector3Dot(dir, pSphereCastData->rayTraceData.normal);
		float hypotinus;
		ST_Vector3 orthogonal = pSphereCastData->rayTraceData.normal;
		if (dot == -1.0f)
		{
			alpha = M_PI;
			hypotinus = radius;
		}
		else
		{
			orthogonal = sphereTraceVector3Cross(pSphereCastData->rayTraceData.normal, dir);
			orthogonal = sphereTraceVector3Cross(pSphereCastData->rayTraceData.normal, orthogonal);
			orthogonal = sphereTraceVector3Normalize(orthogonal);
			if (sphereTraceVector3EpsilonEquals(dir, sphereTraceVector3Negative(orthogonal), COLLIDER_TOLERANCE))
			{
				alphaInvalid = 1;
			}
			alpha = acosf(sphereTraceVector3Dot(sphereTraceVector3Negative(dir), orthogonal));
			hypotinus = radius / sinf(alpha);
			if (alpha == 0.0f || fpclassify(alpha) == FP_INFINITE || fpclassify(alpha) == FP_NAN)
			{
				alphaInvalid = 1;
			}
		}

		pSphereCastData->sphereCenter = sphereTraceVector3AddAndScale(from, dir, pSphereCastData->rayTraceData.distance - hypotinus);
		ST_Vector3 testPoint = sphereTraceVector3AddAndScale(pSphereCastData->sphereCenter, sphereTraceVector3Negative(pSphereCastData->rayTraceData.normal), radius);

		if (!alphaInvalid && sphereTraceColliderTriangleIsProjectedPointContained(testPoint, pTriangleCollider))
		{
			pSphereCastData->rayTraceData.hitPoint = testPoint;
			pSphereCastData->rayTraceData.startPoint = from;
			pSphereCastData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pSphereCastData->rayTraceData.hitPoint, from));
			pSphereCastData->radius = radius;
			pSphereCastData->traceDistance = sphereTraceVector3Distance(pSphereCastData->sphereCenter, pSphereCastData->rayTraceData.startPoint);
			//pSphereCastData->rayTraceData.startPoint = from;
			return 1;
		}
		else
		{
			int closestEdgeIndex;
			ST_Vector3 closestPoint;
			if (!alphaInvalid)
				closestEdgeIndex = sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToPoint(pTriangleCollider, testPoint);
			else
			{
				closestEdgeIndex = sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToSphereTrace(pTriangleCollider, from, radius, dir, &closestPoint);

			}
			ST_Vector3 edgeDir = pTriangleCollider->edgeDirs[closestEdgeIndex];
			ST_Vector3 right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, edgeDir));
			ST_Vector3 sphereDir = sphereTraceVector3Negative(dir);
			if (!alphaInvalid)
				sphereDir = sphereTraceVector3Normalize(sphereTraceVector3Cross(right, edgeDir));
			else
			{
				float d = fabsf(sphereTraceVector3Dot(edgeDir, dir));
				if (d > COLLIDER_TOLERANCE)
					sphereDir = sphereTraceVector3Normalize(sphereTraceVector3Cross(right, edgeDir));
			}
			ST_Vector3 fwd = sphereTraceVector3Normalize(sphereTraceVector3Cross(right, dir));
			float theta = acosf(sphereTraceVector3Dot(edgeDir, dir));
			float rightDist;
			if (!alphaInvalid)
				rightDist = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereCastData->rayTraceData.hitPoint, pTriangleCollider->transformedEdges[closestEdgeIndex].point1), right);
			else
				rightDist = sphereTraceVector3Dot(sphereTraceVector3Subtract(from, pTriangleCollider->transformedEdges[closestEdgeIndex].point1), right);
			if (fabsf(rightDist) < radius)
			{
				float beta = acosf(rightDist / radius);
				float fwdDist = sinf(beta) * radius;
				ST_Vector3 pointOnEdgeClosestToCenterRaycast = sphereTraceClosestPointOnLineBetweenTwoLines(pTriangleCollider->transformedEdges[closestEdgeIndex].point1, edgeDir, from, dir);
				if (sphereTraceVector3Dot(dir, sphereTraceVector3Subtract(pointOnEdgeClosestToCenterRaycast, from)) < 0.0f)
					return 0;
				ST_Vector3 pointOnPlaneNearOriginClosestToEdge = sphereTraceClosestPointOnLineBetweenTwoLines(from, right, pTriangleCollider->transformedEdges[closestEdgeIndex].point1, edgeDir);
				pointOnPlaneNearOriginClosestToEdge = sphereTraceVector3AddAndScale(pointOnPlaneNearOriginClosestToEdge, fwd, -fwdDist);
				ST_Vector3 pointOnEdgeTracedFrompointOnPlaneNearOriginClosestToEdge = sphereTraceClosestPointOnLineBetweenTwoLines(pTriangleCollider->transformedEdges[closestEdgeIndex].point1, edgeDir, pointOnPlaneNearOriginClosestToEdge, dir);
				float distToPointOnEdgeOrthogonalWithSphereDirectionAndEdge = cosf(theta) * sphereTraceVector3Length(sphereTraceVector3Subtract(pointOnEdgeTracedFrompointOnPlaneNearOriginClosestToEdge, pointOnEdgeClosestToCenterRaycast));
				ST_Vector3 intersection = sphereTraceVector3AddAndScale(pointOnEdgeClosestToCenterRaycast, edgeDir, -distToPointOnEdgeOrthogonalWithSphereDirectionAndEdge);

				fwdDist = fabsf(tanf(theta) * sphereTraceVector3Length(sphereTraceVector3Subtract(pointOnEdgeClosestToCenterRaycast, intersection)));
				ST_Vector3 testPoint2 = sphereTraceClosestPointOnLineBetweenTwoLines(intersection, sphereDir, pointOnEdgeClosestToCenterRaycast, dir);
				if (sphereTraceVector3Nan(testPoint2))
				{
					fwdDist = sinf(beta) * radius;
					testPoint2 = sphereTraceVector3AddAndScale(intersection, sphereDir, fwdDist);
				}
				float edgeDist = sphereTraceVector3Dot(sphereTraceVector3Subtract(intersection, pTriangleCollider->transformedEdges[closestEdgeIndex].point1), edgeDir);
				if (edgeDist >= 0.0f && edgeDist <= pTriangleCollider->edgeDists[closestEdgeIndex])
				{

					pSphereCastData->rayTraceData.startPoint = from;
					pSphereCastData->rayTraceData.hitPoint = intersection;
					pSphereCastData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pSphereCastData->rayTraceData.startPoint, pSphereCastData->rayTraceData.hitPoint));
					pSphereCastData->radius = radius;
					pSphereCastData->sphereCenter = sphereTraceVector3Add(sphereTraceVector3AddAndScale(pointOnPlaneNearOriginClosestToEdge, right, rightDist), sphereTraceVector3Subtract(testPoint2, pointOnPlaneNearOriginClosestToEdge));
					pSphereCastData->rayTraceData.normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pSphereCastData->sphereCenter, intersection));
					pSphereCastData->traceDistance = sphereTraceVector3Distance(pSphereCastData->sphereCenter, pSphereCastData->rayTraceData.startPoint);
					return 1;
				}
			}

			{
				//check point

				ST_Vector3 dp;
				if (!alphaInvalid)
				{
					closestPoint = pTriangleCollider->transformedVertices[sphereTraceColliderTriangleGetClosestTransformedVertexIndexToPoint(pTriangleCollider, pSphereCastData->rayTraceData.hitPoint)];
					dp = sphereTraceVector3Subtract(pSphereCastData->rayTraceData.hitPoint, closestPoint);
				}
				else
				{
					//float p1Dist = sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistanceOnLine(pTriangleCollider->transformedEdges[closestEdgeIndex].point1, edgeDir, from, dir);
					//float p2Dist = sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistanceOnLine(pTriangleCollider->transformedEdges[closestEdgeIndex].point2, edgeDir, from, dir);
					//if (fabsf(p1Dist) <= fabsf(p2Dist))
					//	closestPoint = pTriangleCollider->transformedEdges[closestEdgeIndex].point1;
					//else
					//	closestPoint = pTriangleCollider->transformedEdges[closestEdgeIndex].point2;
					ST_Vector3 hitPoint = sphereTraceClosestPointOnLineBetweenTwoLines(from, dir, closestPoint, pTriangleCollider->normal);
					dp = sphereTraceVector3Subtract(hitPoint, closestPoint);
				}
				ST_Vector3 cross = sphereTraceVector3Normalize(sphereTraceVector3Cross(sphereTraceVector3Normalize(dp), dir));
				cross = sphereTraceVector3Cross(dir, cross);
				float distToCenterNormal = sphereTraceVector3Dot(dp, cross);
				if (fpclassify(distToCenterNormal) == FP_NAN)
				{
					pSphereCastData->rayTraceData.startPoint = from;
					pSphereCastData->rayTraceData.hitPoint = closestPoint;
					pSphereCastData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pSphereCastData->rayTraceData.startPoint, pSphereCastData->rayTraceData.hitPoint));
					pSphereCastData->radius = radius;
					pSphereCastData->sphereCenter = sphereTraceVector3AddAndScale(closestPoint, dir, -radius);
					pSphereCastData->rayTraceData.normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pSphereCastData->sphereCenter, pSphereCastData->rayTraceData.hitPoint));
					pSphereCastData->traceDistance = sphereTraceVector3Distance(pSphereCastData->sphereCenter, pSphereCastData->rayTraceData.startPoint);
					return 1;
				}
				else if (fabsf(distToCenterNormal) <= radius)
				{
					pSphereCastData->rayTraceData.startPoint = from;
					pSphereCastData->rayTraceData.hitPoint = closestPoint;
					pSphereCastData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pSphereCastData->rayTraceData.startPoint, pSphereCastData->rayTraceData.hitPoint));
					pSphereCastData->radius = radius;
					float beta = acosf(distToCenterNormal / radius);
					float opposite = sinf(beta) * radius;
					pSphereCastData->sphereCenter = sphereTraceVector3AddAndScale(sphereTraceVector3AddAndScale(closestPoint, cross, distToCenterNormal), dir, -opposite);
					pSphereCastData->rayTraceData.normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pSphereCastData->sphereCenter, pSphereCastData->rayTraceData.hitPoint));
					pSphereCastData->traceDistance = sphereTraceVector3Distance(pSphereCastData->sphereCenter, pSphereCastData->rayTraceData.startPoint);
					return 1;
				}

			}
		}
	}
	return 0;
}
