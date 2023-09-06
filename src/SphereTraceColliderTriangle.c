#include "SphereTraceColliderTriangle.h"

int sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToPoint(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point)
{
	ST_Vector3 dp1 = sphereTraceVector3Subtract(point, pTriangleCollider->transformedEdges[0].point1);
	ST_Vector3 dp2 = sphereTraceVector3Subtract(point, pTriangleCollider->transformedEdges[1].point1);
	ST_Vector3 dp3 = sphereTraceVector3Subtract(point, pTriangleCollider->transformedEdges[2].point1);
	float normalizedDir0 = sphereTraceVector3Dot(pTriangleCollider->edgeOrthogonalDirs[0].v, dp1);
	float normalizedDir1 = sphereTraceVector3Dot(pTriangleCollider->edgeOrthogonalDirs[1].v, dp2);
	float normalizedDir2 = sphereTraceVector3Dot(pTriangleCollider->edgeOrthogonalDirs[2].v, dp3);
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

int sphereTraceColliderTriangleGetClosestTransformedVertexIndexToPoint(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point)
{
	ST_Vector3 dp = sphereTraceVector3Subtract(point, pTriangleCollider->centroid);
	float normalizedDir0 = sphereTraceVector3Dot(pTriangleCollider->vertexDirs[0].v, dp) / pTriangleCollider->vertexDists[0];
	float normalizedDir1 = sphereTraceVector3Dot(pTriangleCollider->vertexDirs[1].v, dp) / pTriangleCollider->vertexDists[1];
	float normalizedDir2 = sphereTraceVector3Dot(pTriangleCollider->vertexDirs[2].v, dp) / pTriangleCollider->vertexDists[2];
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
	pTriangleCollider->normal = sphereTraceDirectionConstructNormalized(sphereTraceVector3Cross(sphereTraceVector3Subtract(v3, v2), sphereTraceVector3Subtract(v1, v2)));
	pTriangleCollider->transformedEdges[0] = sphereTraceEdgeConstruct(v3, v1);
	pTriangleCollider->transformedEdges[1] = sphereTraceEdgeConstruct(v1, v2);
	pTriangleCollider->transformedEdges[2] = sphereTraceEdgeConstruct(v2, v3);
	pTriangleCollider->vertexDirs[0] = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(v1, pTriangleCollider->centroid));
	pTriangleCollider->vertexDirs[1] = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(v2, pTriangleCollider->centroid));
	pTriangleCollider->vertexDirs[2] = sphereTraceDirectionConstructNormalized(sphereTraceVector3Subtract(v3, pTriangleCollider->centroid));
	pTriangleCollider->vertexDists[0] = sphereTraceVector3Length(sphereTraceVector3Subtract(v1, pTriangleCollider->centroid));
	pTriangleCollider->vertexDists[1] = sphereTraceVector3Length(sphereTraceVector3Subtract(v2, pTriangleCollider->centroid));
	pTriangleCollider->vertexDists[2] = sphereTraceVector3Length(sphereTraceVector3Subtract(v3, pTriangleCollider->centroid));
	//pTriangleCollider->edgeCenterDists[0] = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereTraceVector3Average(pTriangleCollider->transformedEdges[0].point1, pTriangleCollider->transformedEdges[0].point2), pTriangleCollider->centroid));
	//pTriangleCollider->edgeCenterDists[1] = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereTraceVector3Average(pTriangleCollider->transformedEdges[1].point1, pTriangleCollider->transformedEdges[1].point2), pTriangleCollider->centroid));
	//pTriangleCollider->edgeCenterDists[2] = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereTraceVector3Average(pTriangleCollider->transformedEdges[2].point1, pTriangleCollider->transformedEdges[2].point2), pTriangleCollider->centroid));
	//pTriangleCollider->edgeDirs[0] = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pTriangleCollider->transformedEdges[0].point2, pTriangleCollider->transformedEdges[0].point1));
	//pTriangleCollider->edgeDirs[1] = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pTriangleCollider->transformedEdges[1].point2, pTriangleCollider->transformedEdges[1].point1));
	//pTriangleCollider->edgeDirs[2] = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pTriangleCollider->transformedEdges[2].point2, pTriangleCollider->transformedEdges[2].point1));
	//pTriangleCollider->edgeDists[0] = sphereTraceVector3Length(sphereTraceVector3Subtract(pTriangleCollider->transformedEdges[0].point2, pTriangleCollider->transformedEdges[0].point1));
	//pTriangleCollider->edgeDists[1] = sphereTraceVector3Length(sphereTraceVector3Subtract(pTriangleCollider->transformedEdges[1].point2, pTriangleCollider->transformedEdges[1].point1));
	//pTriangleCollider->edgeDists[2] = sphereTraceVector3Length(sphereTraceVector3Subtract(pTriangleCollider->transformedEdges[2].point2, pTriangleCollider->transformedEdges[2].point1));
	pTriangleCollider->edgeOrthogonalDirs[0] = sphereTraceDirectionConstructNormalized(sphereTraceVector3Cross(pTriangleCollider->transformedEdges[0].dir.v, pTriangleCollider->normal.v));
	pTriangleCollider->edgeOrthogonalDirs[1] = sphereTraceDirectionConstructNormalized(sphereTraceVector3Cross(pTriangleCollider->transformedEdges[1].dir.v, pTriangleCollider->normal.v));
	pTriangleCollider->edgeOrthogonalDirs[2] = sphereTraceDirectionConstructNormalized(sphereTraceVector3Cross(pTriangleCollider->transformedEdges[2].dir.v, pTriangleCollider->normal.v));
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
	float dot1 = sphereTraceVector3Dot(pTriangleCollider->normal.v, sphereTraceVector3Cross(sphereTraceVector3Subtract(pTriangleCollider->transformedVertices[0], pTriangleCollider->transformedVertices[2]),
		sphereTraceVector3Subtract(projectedPoint, pTriangleCollider->transformedVertices[2])));
	float dot2 = sphereTraceVector3Dot(pTriangleCollider->normal.v, sphereTraceVector3Cross(sphereTraceVector3Subtract(pTriangleCollider->transformedVertices[1], pTriangleCollider->transformedVertices[0]),
		sphereTraceVector3Subtract(projectedPoint, pTriangleCollider->transformedVertices[0])));
	float dot3 = sphereTraceVector3Dot(pTriangleCollider->normal.v, sphereTraceVector3Cross(sphereTraceVector3Subtract(pTriangleCollider->transformedVertices[2], pTriangleCollider->transformedVertices[1]),
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

b32 sphereTraceColliderTriangleRayTrace(ST_Vector3 from, ST_Direction dir, const ST_TriangleCollider* const pTriangleCollider, ST_RayTraceData* const pRaycastData)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	sphereTraceColliderInfinitePlaneRayTrace(from, dir, pTriangleCollider->normal, pTriangleCollider->centroid, pRaycastData);
	if (pRaycastData->distance >= 0.0f)
	{
		if (fpclassify(pRaycastData->distance) == FP_INFINITE)
			return 0;
		pRaycastData->startPoint = from;
		pRaycastData->hitPoint = sphereTraceVector3Add(from, sphereTraceVector3Scale(dir.v, pRaycastData->distance));

		return sphereTraceColliderTriangleIsProjectedPointContained(pRaycastData->hitPoint, pTriangleCollider);

	}
	return 0;
}

b32 sphereTraceColliderTriangleSphereTrace(ST_Vector3 from, ST_Direction dir, float radius, ST_TriangleCollider* const pTriangleCollider, ST_SphereTraceData* const pSphereTraceData)
{
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
	if (sphereTraceColliderInfinitePlaneSphereTrace(from, dir, radius, pTriangleCollider->centroid, pTriangleCollider->normal, pSphereTraceData))
	{
		if (sphereTraceColliderTriangleIsProjectedPointContained(pSphereTraceData->rayTraceData.hitPoint, pTriangleCollider))
		{
			return 1;
		}
	}
	float closestEdgePointDist = FLT_MAX;
	ST_SphereTraceData datTest;
	if (sphereTraceColliderEdgeSphereTrace(from, dir, radius, &pTriangleCollider->transformedEdges[0], &datTest))
	{
		closestEdgePointDist = datTest.traceDistance;
		*pSphereTraceData = datTest;
	}
	if (sphereTraceColliderEdgeSphereTrace(from, dir, radius, &pTriangleCollider->transformedEdges[1], &datTest))
	{
		if (datTest.traceDistance < closestEdgePointDist)
		{
			closestEdgePointDist = datTest.traceDistance;
			*pSphereTraceData = datTest;
		}
	}
	if (sphereTraceColliderEdgeSphereTrace(from, dir, radius, &pTriangleCollider->transformedEdges[2], &datTest))
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