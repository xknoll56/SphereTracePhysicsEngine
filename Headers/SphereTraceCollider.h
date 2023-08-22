#pragma once

#define COLLIDER_TOLERANCE 1e-6

typedef enum ST_ColliderType
{
	COLLIDER_SPHERE = 0,
	COLLIDER_PLANE = 1
} ST_ColliderType;

typedef int ST_ColliderIndex;

//typedef struct ST_SpacialPartitionBucket
//{
//	ST_Vector3 position;
//	int index;
//};

typedef struct ST_AABB
{
	//The size of the aabb
	ST_Vector3 halfExtents;

	//Transformed vertices
	ST_Vector3 rightTopForwardsTransformedVertex;
	ST_Vector3 leftDownBackTransformedVertex;
} ST_AABB;

typedef struct ST_Edge
{
	ST_Vector3 point1;
	ST_Vector3 point2;
} ST_Edge;

typedef struct ST_PlaneCollider
{
	ST_Vector3 normal;
	ST_Vector3 right;
	ST_Vector3 forward;
	float xHalfExtent;
	float zHalfExtent;
	ST_Vector3 position;
	ST_Quaternion rotation;
	ST_Vector3 transformedVertices[4];
	ST_Edge transformedEdges[4];
	ST_AABB aabb;
	ST_IntList bucketIndices;
	ST_ColliderIndex planeColliderIndex;
} ST_PlaneCollider;

typedef struct ST_TriangleCollider
{
	ST_Vector3 transformedVertices[3];
	ST_Edge transformedEdges[3];
	ST_Vector3 vertexDirs[3];
	ST_Vector3 edgeOrthogonalDirs[3];
	float vertexDists[3];
	float edgeCenterDists[3];
	ST_Vector3 edgeDirs[3];
	float edgeDists[3];
	ST_Vector3 normal;
	ST_Vector3 centroid;
	ST_AABB aabb;
	ST_IntList bucketIndices;
	b32 ignoreCollisions;
} ST_TriangleCollider;

typedef struct ST_SphereCollider
{
	float radius;
	ST_RigidBody* pRigidBody;
	ST_AABB aabb;
	ST_IntList bucketIndices;
	ST_ColliderIndex sphereColliderIndex;
	b32 ignoreCollisions;
} ST_SphereCollider;

typedef enum ST_PlaneEdgeDirection
{
	PLANE_EDGE_RIGHT = 0,
	PLANE_EDGE_FORWARD = 1,
	PLANE_EDGE_LEFT = 2,
	PLANE_EDGE_BACK = 3
} ST_PlaneEdgeDirection;

typedef enum ST_PlaneVertexDirection
{
	PLANE_VEREX_FORWARD_RIGHT = 0,
	PLANE_VEREX_FORWARD_LEFT = 1,
	PLANE_VEREX_BACK_LEFT = 2,
	PLANE_VEREX_BACK_RIGHT = 3
} ST_PlaneVertexDirection;

typedef struct ST_RayTraceData
{
	ST_Vector3 startPoint;
	ST_Vector3 hitPoint;
	ST_Vector3 normal;
	float distance;
} ST_RayTraceData;

typedef struct ST_SphereTraceData
{
	ST_RayTraceData rayTraceData;
	ST_Vector3 sphereCenter;
	float radius;
	//the trace distance is the distance from the start sphere to the end sphere
	float traceDistance;
} ST_SphereTraceData;

typedef enum ST_SpherePlaneCollisionType
{
	COLLISION_FACE_FACE = 0,
	COLLISION_FACE_EDGE = 1,
	COLLISION_FACE_POINT = 2
} ST_SpherePlaneCollisionType;

typedef struct ST_SpherePlaneContactInfo
{
	float penetrationDistance;
	ST_Vector3 normal;
	ST_Vector3 point;
	ST_SphereCollider* pSphereCollider;
	ST_PlaneCollider* pPlaneCollider;
	ST_SpherePlaneCollisionType collisionType;
} ST_SpherePlaneContactInfo;

typedef struct ST_SphereTriangleContactInfo
{
	float penetrationDistance;
	ST_Vector3 normal;
	ST_Vector3 point;
	ST_SphereCollider* pSphereCollider;
	ST_TriangleCollider* pTriangleCollider;
	ST_SpherePlaneCollisionType collisionType;
} ST_SphereTriangleContactInfo;

typedef struct ST_SphereTerrainTriangleContactInfo
{
	ST_SphereTriangleContactInfo sphereTriangleContactInfo;
	ST_SphereTraceData downSphereTraceData;
} ST_SphereTerrainTriangleContactInfo;

typedef struct ST_SphereSphereContactInfo
{
	float penetrationDistance;
	ST_Vector3 normal;
	ST_Vector3 point;
	ST_SphereCollider* pA;
	ST_SphereCollider* pB;
} ST_SphereSphereContactInfo;

void sphereTraceColliderPlaneSetTransformedVerticesAndEdges(ST_PlaneCollider* const pPlaneCollider)
{
	//forward right
	pPlaneCollider->transformedVertices[PLANE_VEREX_FORWARD_RIGHT] = sphereTraceVector3Add(sphereTraceVector3Add(pPlaneCollider->position, sphereTraceVector3Scale(pPlaneCollider->right, pPlaneCollider->xHalfExtent)), sphereTraceVector3Scale(pPlaneCollider->forward, pPlaneCollider->zHalfExtent));
	//forward left
	pPlaneCollider->transformedVertices[PLANE_VEREX_FORWARD_LEFT] = sphereTraceVector3Add(sphereTraceVector3Add(pPlaneCollider->position, sphereTraceVector3Scale(pPlaneCollider->right, -pPlaneCollider->xHalfExtent)), sphereTraceVector3Scale(pPlaneCollider->forward, pPlaneCollider->zHalfExtent));
	//back left
	pPlaneCollider->transformedVertices[PLANE_VEREX_BACK_LEFT] = sphereTraceVector3Add(sphereTraceVector3Add(pPlaneCollider->position, sphereTraceVector3Scale(pPlaneCollider->right, -pPlaneCollider->xHalfExtent)), sphereTraceVector3Scale(pPlaneCollider->forward, -pPlaneCollider->zHalfExtent));
	//back right
	pPlaneCollider->transformedVertices[PLANE_VEREX_BACK_RIGHT] = sphereTraceVector3Add(sphereTraceVector3Add(pPlaneCollider->position, sphereTraceVector3Scale(pPlaneCollider->right, pPlaneCollider->xHalfExtent)), sphereTraceVector3Scale(pPlaneCollider->forward, -pPlaneCollider->zHalfExtent));

	//right edge
	pPlaneCollider->transformedEdges[PLANE_EDGE_RIGHT] = (ST_Edge){ pPlaneCollider->transformedVertices[3], pPlaneCollider->transformedVertices[0] };
	//forward edge
	pPlaneCollider->transformedEdges[PLANE_EDGE_FORWARD] = (ST_Edge){ pPlaneCollider->transformedVertices[0], pPlaneCollider->transformedVertices[1] };
	//left edge
	pPlaneCollider->transformedEdges[PLANE_EDGE_LEFT] = (ST_Edge){ pPlaneCollider->transformedVertices[1], pPlaneCollider->transformedVertices[2] };
	//back edge
	pPlaneCollider->transformedEdges[PLANE_EDGE_BACK] = (ST_Edge){ pPlaneCollider->transformedVertices[2], pPlaneCollider->transformedVertices[3] };

}


ST_PlaneEdgeDirection sphereTraceColliderPlaneGetClosestTransformedEdgeToPoint(const ST_PlaneCollider* const pPlaneCollider, ST_Vector3 point)
{
	ST_Vector3 distVec = sphereTraceVector3Subtract(point, pPlaneCollider->position);
	float xDist = sphereTraceVector3Dot(distVec, pPlaneCollider->right)/pPlaneCollider->xHalfExtent;
	float zDist = sphereTraceVector3Dot(distVec, pPlaneCollider->forward)/pPlaneCollider->zHalfExtent;

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
	float sForward = sphereTraceVector3Dot(dir, pPlaneCollider->forward);
	float sRight = sphereTraceVector3Dot(dir, pPlaneCollider->right);
	float s1, t1, s2, t2;
	float u1, v1, u2, v2;
	float dt1, dt2;
	ST_Vector3 right, fwd;
	ST_Vector3 start;
	ST_Vector3 closestPoint1, closestPoint2;
	ST_PlaneEdgeDirection e1, e2;
	if (sForward < 0.0f)
	{
		right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, pPlaneCollider->forward));
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
			if (eMin > pPlaneCollider->zHalfExtent*2.0f)
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
		right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, sphereTraceVector3Negative(pPlaneCollider->forward)));
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
		right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, pPlaneCollider->right));
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
		right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, sphereTraceVector3Negative(pPlaneCollider->right)));
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
	float xDist = sphereTraceVector3Dot(distVec, pPlaneCollider->right);
	float zDist = sphereTraceVector3Dot(distVec, pPlaneCollider->forward);
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
	if (t1 <0.0f || t1 > pTriangleCollider->edgeDists[0] || s>0.0f || fpclassify(s1) == FP_NAN)
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
	right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, pTriangleCollider->edgeOrthogonalDirs[0]));
	fwd = sphereTraceVector3Cross(dir, right);
	float rightDist = sphereTraceVector3Dot(right, pTriangleCollider->transformedEdges[0].point1);
	float theta = acosf(fabsf(rightDist) / radius);
	ST_Vector3 start = sphereTraceVector3AddAndScale(point, fwd, radius*sinf(theta));
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


	{
		if (t1 >= pTriangleCollider->edgeDists[0] * 0.5f)
		{
			if(eMin > pTriangleCollider->edgeDists[0])
				dt1 = FLT_MAX;
			else
				dt1 = pTriangleCollider->edgeDists[0] - t1;
			closestPoint1 = pTriangleCollider->transformedEdges[0].point2;
		}
		else
		{
			if (eMax < 0.0f)
				dt1 = FLT_MAX;
			else
				dt1 = t1;
			closestPoint1 = pTriangleCollider->transformedEdges[0].point1;
		}
	}
	if (s > 0.0f || fpclassify(s1) == FP_NAN)
	{
		dt1 = FLT_MAX;
	}
	//if (t1 <0.0f )
	//{
	//	ds1 = -t1;
	//	d1 = FLT_MAX;
	//}
	//else if (t1 > pTriangleCollider->edgeDists[0])
	//{
	//	ds1 = t1- pTriangleCollider->edgeDists[0];
	//	d1 = FLT_MAX;
	//}
	//if (s > 0.0f || fpclassify(s1) == FP_NAN)
	//{
	//	d1 = FLT_MAX;
	//	ds1 = FLT_MAX;
	//}
	right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, pTriangleCollider->edgeOrthogonalDirs[1]));
	fwd = sphereTraceVector3Cross(dir, right);
	rightDist = sphereTraceVector3Dot(right, pTriangleCollider->transformedEdges[1].point1);
	theta = acosf(fabsf(rightDist) / radius);
	start = sphereTraceVector3AddAndScale(point, fwd, radius*sinf(theta));
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

	{
		if (t2 >= pTriangleCollider->edgeDists[1] * 0.5f)
		{
			if (eMin > pTriangleCollider->edgeDists[1])
				dt2 = FLT_MAX;
			else
				dt2 = pTriangleCollider->edgeDists[1] - t2;
			closestPoint2 = pTriangleCollider->transformedEdges[1].point2;
		}
		else
		{
			if (eMax < 0.0f)
				dt2 = FLT_MAX;
			else
				dt2 = t2;
			closestPoint2 = pTriangleCollider->transformedEdges[1].point1;
		}
	}
	if (s > 0.0f || fpclassify(s2) == FP_NAN)
	{
		dt2 = FLT_MAX;
	}
	//if (t2 < 0.0f)
	//{
	//	ds2 = -t2;
	//	d2 = FLT_MAX;
	//}
	//else if (t2 > pTriangleCollider->edgeDists[1])
	//{
	//	ds2 = t2- pTriangleCollider->edgeDists[1];
	//	d2 = FLT_MAX;
	//}
	//if (s > 0.0f || fpclassify(s2) == FP_NAN)
	//{
	//	d2 = FLT_MAX;
	//	ds2 = FLT_MAX;
	//}
	right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, pTriangleCollider->edgeOrthogonalDirs[2]));
	fwd = sphereTraceVector3Cross(dir, right);
	rightDist = sphereTraceVector3Dot(right, pTriangleCollider->transformedEdges[2].point1);
	theta = acosf(fabsf(rightDist) / radius);
	start = sphereTraceVector3AddAndScale(point, fwd, radius*sinf(theta));
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

	{
		if (t3 >= pTriangleCollider->edgeDists[2] * 0.5f)
		{
			if (eMin > pTriangleCollider->edgeDists[2])
				dt3 = FLT_MAX;
			else
				dt3 = pTriangleCollider->edgeDists[2]- t3;
			closestPoint3 = pTriangleCollider->transformedEdges[2].point2;
		}
		else
		{
			if (eMax < 0.0f)
				dt3 = FLT_MAX;
			else
				dt3 = t3;
			closestPoint3 = pTriangleCollider->transformedEdges[2].point1;
		}
	}
	if (s > 0.0f || fpclassify(s3) == FP_NAN)
	{
		dt3 = FLT_MAX;
	}
	//if (t3 < 0.0f)
	//{
	//	ds3 = -t3;
	//	d3 = FLT_MAX;
	//}
	//else if (t3 > pTriangleCollider->edgeDists[2])
	//{
	//	ds3 = t3- pTriangleCollider->edgeDists[2];
	//	d3 = FLT_MAX;
	//}
	//if (s > 0.0f || fpclassify(s3) == FP_NAN)
	//{
	//	d3 = FLT_MAX;
	//	ds3 = FLT_MAX;
	//}
	//if (d1 == d2 && d1 == d3)
	//{
	//	if ((ds1 <= ds2) && (ds1 <= ds3))
	//	{
	//		if (t1 >= pTriangleCollider->edgeDists[0] * 0.5f)
	//			*closestPoint = pTriangleCollider->transformedEdges[0].point2;
	//		else
	//			*closestPoint = pTriangleCollider->transformedEdges[0].point1;
	//		return 0;
	//	}
	//	else if (ds2 <= ds3)
	//	{
	//		if (t2 >= pTriangleCollider->edgeDists[1] * 0.5f)
	//			*closestPoint = pTriangleCollider->transformedEdges[1].point2;
	//		else
	//			*closestPoint = pTriangleCollider->transformedEdges[1].point1;
	//		return 1;
	//	}
	//	else
	//	{
	//		if (t3 >= pTriangleCollider->edgeDists[2] * 0.5f)
	//			*closestPoint = pTriangleCollider->transformedEdges[2].point2;
	//		else
	//			*closestPoint = pTriangleCollider->transformedEdges[2].point1;
	//		return 2;
	//	}
	//}
	if ((dt1 <= dt2) && (dt1 <= dt3))
	{
		*closestPoint = closestPoint1;
		return 0;
	}
	else if (dt2 <= dt3)
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

void sphereTraceColliderSphereAABBSetTransformedVertices(ST_SphereCollider* const pSphereCollider)
{
	pSphereCollider->aabb.rightTopForwardsTransformedVertex = sphereTraceVector3Add(pSphereCollider->pRigidBody->position, pSphereCollider->aabb.halfExtents);
	pSphereCollider->aabb.leftDownBackTransformedVertex = sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, pSphereCollider->aabb.halfExtents);
}

void sphereTraceColliderTriangleSetAABB(ST_TriangleCollider* const pTriangleCollider)
{
	pTriangleCollider->aabb.rightTopForwardsTransformedVertex = (ST_Vector3){ fmaxf(pTriangleCollider->transformedVertices[0].x, fmaxf(pTriangleCollider->transformedVertices[1].x, pTriangleCollider->transformedVertices[2].x)),
	fmaxf(pTriangleCollider->transformedVertices[0].y, fmaxf(pTriangleCollider->transformedVertices[1].y, pTriangleCollider->transformedVertices[2].y)),
	fmaxf(pTriangleCollider->transformedVertices[0].z, fmaxf(pTriangleCollider->transformedVertices[1].z, pTriangleCollider->transformedVertices[2].z)) };

	pTriangleCollider->aabb.leftDownBackTransformedVertex = (ST_Vector3){ fminf(pTriangleCollider->transformedVertices[0].x, fminf(pTriangleCollider->transformedVertices[1].x, pTriangleCollider->transformedVertices[2].x)),
fminf(pTriangleCollider->transformedVertices[0].y, fminf(pTriangleCollider->transformedVertices[1].y, pTriangleCollider->transformedVertices[2].y)),
fminf(pTriangleCollider->transformedVertices[0].z, fminf(pTriangleCollider->transformedVertices[1].z, pTriangleCollider->transformedVertices[2].z)) };

	pTriangleCollider->aabb.halfExtents = sphereTraceVector3Subtract(pTriangleCollider->aabb.rightTopForwardsTransformedVertex,
		sphereTraceVector3Average(pTriangleCollider->aabb.rightTopForwardsTransformedVertex, pTriangleCollider->aabb.leftDownBackTransformedVertex));
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
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale((ST_Vector3){pSphereCastData->rayTraceData.startPoint.x, 
					pSphereCastData->rayTraceData.startPoint.y, pSphereCastData->sphereCenter.z}, gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale((ST_Vector3) {pSphereCastData->sphereCenter.x,
						pSphereCastData->sphereCenter.y, pSphereCastData->rayTraceData.startPoint.z}, gVector3One, pSphereCastData->radius);
			}
		}
		else
		{
			if (dp.z >= 0.0f)
			{
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale((ST_Vector3) {pSphereCastData->rayTraceData.startPoint.x,
					pSphereCastData->sphereCenter.y, pSphereCastData->rayTraceData.startPoint.z}, gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale((ST_Vector3) {pSphereCastData->sphereCenter.x,
					pSphereCastData->rayTraceData.startPoint.y, pSphereCastData->sphereCenter.z}, gVector3One, pSphereCastData->radius);
			}
			else
			{
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale((ST_Vector3) {pSphereCastData->rayTraceData.startPoint.x,
						pSphereCastData->sphereCenter.y, pSphereCastData->sphereCenter.z}, gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale((ST_Vector3) {pSphereCastData->sphereCenter.x,
						pSphereCastData->rayTraceData.startPoint.y, pSphereCastData->rayTraceData.startPoint.z}, gVector3One, pSphereCastData->radius);

			}
		}
	}
	else
	{
		if (dp.y >= 0.0f)
		{
			if (dp.z >= 0.0f)
			{
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale((ST_Vector3) {pSphereCastData->sphereCenter.x,
						pSphereCastData->rayTraceData.startPoint.y, pSphereCastData->rayTraceData.startPoint.z}, gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale((ST_Vector3) {pSphereCastData->rayTraceData.startPoint.x,
						pSphereCastData->sphereCenter.y, pSphereCastData->sphereCenter.z}, gVector3One, pSphereCastData->radius);
			}
			else
			{
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale((ST_Vector3) {pSphereCastData->sphereCenter.x,
						pSphereCastData->rayTraceData.startPoint.y, pSphereCastData->sphereCenter.z}, gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale((ST_Vector3) {pSphereCastData->rayTraceData.startPoint.x,
						pSphereCastData->sphereCenter.y, pSphereCastData->rayTraceData.startPoint.z}, gVector3One, pSphereCastData->radius);
			}
		}
		else
		{
			if (dp.z >= 0.0f)
			{
				aabb->leftDownBackTransformedVertex = sphereTraceVector3AddAndScale((ST_Vector3) {pSphereCastData->sphereCenter.x,
						pSphereCastData->sphereCenter.y, pSphereCastData->rayTraceData.startPoint.z}, gVector3One, -pSphereCastData->radius);
				aabb->rightTopForwardsTransformedVertex = sphereTraceVector3AddAndScale((ST_Vector3) {pSphereCastData->rayTraceData.startPoint.x,
						pSphereCastData->rayTraceData.startPoint.y, pSphereCastData->sphereCenter.z}, gVector3One, pSphereCastData->radius);
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

ST_PlaneCollider sphereTraceColliderPlaneConstruct(ST_Vector3 normal, float angle, float xHalfExtent, float zHalfExtent, ST_Vector3 position)
{
	ST_PlaneCollider planeCollider;
	planeCollider.position = position;
	planeCollider.normal = sphereTraceVector3Normalize(normal);
	float dot = sphereTraceVector3Dot(planeCollider.normal, gVector3Forward);
	if (dot==1.0f)
	{
		planeCollider.right = gVector3Right;
	}
	else if (dot == -1.0f)
	{
		planeCollider.right = sphereTraceVector3Negative(gVector3Right);
	}
	else
		planeCollider.right = sphereTraceVector3Normalize(sphereTraceVector3Cross(planeCollider.normal, gVector3Forward));
	ST_Quaternion rotation = sphereTraceQuaternionFromAngleAxis(normal, angle);
	planeCollider.right = sphereTraceVector3RotatePoint(planeCollider.right, rotation);
	planeCollider.forward = sphereTraceVector3Cross(planeCollider.right, planeCollider.normal);
	planeCollider.xHalfExtent = xHalfExtent;
	planeCollider.zHalfExtent = zHalfExtent;
	ST_Matrix4 rotMat = sphereTraceMatrixConstructFromRightForwardUp(planeCollider.right, planeCollider.normal, planeCollider.forward);
	planeCollider.rotation = sphereTraceQuaternionNormalize(sphereTraceMatrixQuaternionFromRotationMatrix(rotMat));

	//retreive the adjusted rotation matrix, matrix from quaternion is not perfect so the directional vectors change slightly
	rotMat = sphereTraceMatrixFromQuaternion(planeCollider.rotation);
	planeCollider.right = sphereTraceVector3GetLocalXAxisFromRotationMatrix(rotMat);
	planeCollider.normal = sphereTraceVector3GetLocalYAxisFromRotationMatrix(rotMat);
	planeCollider.forward = sphereTraceVector3GetLocalZAxisFromRotationMatrix(rotMat);
	sphereTraceColliderPlaneSetTransformedVerticesAndEdges(&planeCollider);
	sphereTraceColliderPlaneSetAABBExtents(&planeCollider);
	sphereTraceColliderPlaneAABBSetTransformedVertices(&planeCollider);

	return planeCollider;
}

ST_PlaneCollider sphereTraceColliderPlaneConstructWithRotationMatrix(ST_Matrix4 rotMat,float xHalfExtent, float zHalfExtent, ST_Vector3 position)
{
	ST_PlaneCollider planeCollider;
	planeCollider.position = position;
	planeCollider.xHalfExtent = xHalfExtent;
	planeCollider.zHalfExtent = zHalfExtent;
	planeCollider.right = sphereTraceVector3GetLocalXAxisFromRotationMatrix(rotMat);
	planeCollider.normal = sphereTraceVector3GetLocalYAxisFromRotationMatrix(rotMat);
	planeCollider.forward = sphereTraceVector3GetLocalZAxisFromRotationMatrix(rotMat);
	planeCollider.rotation = sphereTraceMatrixQuaternionFromRotationMatrix(rotMat);
	sphereTraceColliderPlaneSetTransformedVerticesAndEdges(&planeCollider);
	sphereTraceColliderPlaneSetAABBExtents(&planeCollider);
	sphereTraceColliderPlaneAABBSetTransformedVertices(&planeCollider);

	return planeCollider;
}


ST_SphereCollider sphereTraceColliderSphereConstruct(float radius, const ST_RigidBody* const pRigidBody)
{
	ST_SphereCollider sphereCollider;
	sphereCollider.pRigidBody = pRigidBody;
	sphereCollider.radius = radius;
	sphereCollider.aabb.halfExtents = (ST_Vector3){ radius, radius, radius };
	sphereCollider.ignoreCollisions = 0;
	sphereTraceColliderSphereAABBSetTransformedVertices(&sphereCollider);

	return sphereCollider;
}

ST_SphereCollider sphereTraceColliderSphereConstructWithPosition(float radius, const ST_RigidBody* const pRigidBody, ST_Vector3 position)
{
	ST_SphereCollider sphereCollider;
	sphereCollider.pRigidBody = pRigidBody;
	sphereCollider.pRigidBody->position = position;
	sphereCollider.radius = radius;
	sphereCollider.aabb.halfExtents = (ST_Vector3){ radius, radius, radius };
	sphereCollider.ignoreCollisions = 0;
	sphereTraceColliderSphereAABBSetTransformedVertices(&sphereCollider);

	return sphereCollider;
}

void sphereTraceColliderTriangleSetVertexAndEdgeData(ST_TriangleCollider* const pTriangleCollider, ST_Vector3 v1, ST_Vector3 v2, ST_Vector3 v3)
{
	pTriangleCollider->transformedVertices[0] = v1;
	pTriangleCollider->transformedVertices[1] = v2;
	pTriangleCollider->transformedVertices[2] = v3;
	pTriangleCollider->centroid = (ST_Vector3){ (v1.x + v2.x + v3.x) / 3.0f, (v1.y + v2.y + v3.y) / 3.0f , (v1.z + v2.z + v3.z) / 3.0f };
	pTriangleCollider->normal = sphereTraceVector3Normalize(sphereTraceVector3Cross(sphereTraceVector3Subtract(v3, v2), sphereTraceVector3Subtract(v1, v2)));
	pTriangleCollider->transformedEdges[0] = (ST_Edge){ v3, v1 };
	pTriangleCollider->transformedEdges[1] = (ST_Edge){ v1, v2 };
	pTriangleCollider->transformedEdges[2] = (ST_Edge){ v2, v3 };
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

void sphereTraceColliderInfinitePlaneRayTrace(ST_Vector3 from, ST_Vector3 dir, ST_Vector3 planeNormal, ST_Vector3 pointOnPlane, ST_RayTraceData* const pRaycastData)
{
	dir = sphereTraceVector3Normalize(dir);
	float dirDotNormal = sphereTraceVector3Dot(dir, planeNormal);
	pRaycastData->normal = planeNormal;
	if (dirDotNormal > 0.0f)
	{
		pRaycastData->normal = sphereTraceVector3Negative(pRaycastData->normal);
		dirDotNormal = -dirDotNormal;
	}
	pRaycastData->distance = sphereTraceVector3Dot(sphereTraceVector3Subtract(pointOnPlane, from), pRaycastData->normal) / dirDotNormal;
	pRaycastData->hitPoint = sphereTraceVector3Add(from, sphereTraceVector3Scale(dir, pRaycastData->distance));
}

b32 sphereTraceColliderPlaneRayTrace(ST_Vector3 from, ST_Vector3 dir, const ST_PlaneCollider* const pPlaneCollider, ST_RayTraceData* const pRaycastData)
{
	sphereTraceColliderInfinitePlaneRayTrace(from, dir, pPlaneCollider->normal, pPlaneCollider->position, pRaycastData);
	//dir = sphereTraceVector3Normalize(dir);
	//float dirDotNormal = sphereTraceVector3Dot(dir, pPlaneCollider->normal);
	//pRaycastData->normal = pPlaneCollider->normal;
	//if (dirDotNormal > 0.0f)
	//{
	//	pRaycastData->normal = sphereTraceVector3Negative(pRaycastData->normal);
	//	dirDotNormal = -dirDotNormal;
	//}
	//pRaycastData->distance = sphereTraceVector3Dot(sphereTraceVector3Subtract(pPlaneCollider->position, from), pRaycastData->normal) / dirDotNormal;
	//pRaycastData->hitPoint = sphereTraceVector3Add(from, sphereTraceVector3Scale(dir, pRaycastData->distance));
	if (pRaycastData->distance >= 0.0f)
	{
		if (fpclassify(pRaycastData->distance) == FP_INFINITE)
			return 0;
		pRaycastData->startPoint = from;
		ST_Vector3 vectorFromCenter = sphereTraceVector3Subtract(pRaycastData->hitPoint, pPlaneCollider->position);
		float xDist = sphereTraceVector3Dot(vectorFromCenter, pPlaneCollider->right);
		if (fabsf(xDist) > pPlaneCollider->xHalfExtent)
			return 0;
		float zDist = sphereTraceVector3Dot(vectorFromCenter, pPlaneCollider->forward);
		if (fabsf(zDist) > pPlaneCollider->zHalfExtent)
			return 0;
		return 1;
	}
	return 0;
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
	if ((dot1 == dot2) &&  (dot1== dot3))
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


b32 sphereTraceColliderSphereRayTrace(ST_Vector3 start, ST_Vector3 dir, const ST_SphereCollider* const pSphere, ST_RayTraceData* const pData)
{
	dir = sphereTraceVector3Normalize(dir);
	b32 hits = 0;
	ST_Vector3 p0 = pSphere->pRigidBody->position;
	ST_Vector3 negativeDir = sphereTraceVector3Negative(dir);
	float dist = sphereTraceVector3Dot(sphereTraceVector3Subtract(p0, start), negativeDir) / sphereTraceVector3Dot(dir, negativeDir);

	if (dist >= 0.0f)
	{
		ST_Vector3 intersection = sphereTraceVector3Add(start, sphereTraceVector3Scale(dir, dist));
		ST_Vector3 intersectionMinusStart = sphereTraceVector3Subtract(intersection, p0);
		float l2 = sphereTraceVector3Length2(intersectionMinusStart);
		if (l2 <= pSphere->radius * pSphere->radius)
		{
			hits = 1;
			float length = sphereTraceVector3Length(intersectionMinusStart);
			float theta = min(1.0f, asinf(length / pSphere->radius));
			ST_Vector3 dirComp = sphereTraceVector3Scale(negativeDir, length / tanf(theta));
			if (length == 0.0f)
				dirComp = sphereTraceVector3Scale(negativeDir, pSphere->radius);
			pData->normal = sphereTraceVector3Add(intersectionMinusStart, dirComp);
			pData->hitPoint = sphereTraceVector3Add(pData->normal, p0);
			//stupid hack
			if (sphereTraceVector3Nan(pData->hitPoint))
				return 0;
			pData->startPoint = start;
			pData->distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->hitPoint, start));
			pData->normal = sphereTraceVector3Normalize(pData->normal);
		}
	}

	return hits;
}

b32 sphereTraceColliderImposedSphereRayTrace(ST_Vector3 start, ST_Vector3 dir, ST_Vector3 imposedPosition, float imposedRadius, ST_RayTraceData* const pData)
{
	dir = sphereTraceVector3Normalize(dir);
	b32 hits = 0;
	ST_Vector3 p0 = imposedPosition;
	ST_Vector3 negativeDir = sphereTraceVector3Negative(dir);
	float dist = sphereTraceVector3Dot(sphereTraceVector3Subtract(p0, start), negativeDir) / sphereTraceVector3Dot(dir, negativeDir);

	if (dist >= 0.0f)
	{
		ST_Vector3 intersection = sphereTraceVector3Add(start, sphereTraceVector3Scale(dir, dist));
		ST_Vector3 intersectionMinusStart = sphereTraceVector3Subtract(intersection, p0);
		float l2 = sphereTraceVector3Length2(intersectionMinusStart);
		if (l2 <= imposedRadius * imposedRadius)
		{
			hits = 1;
			float length = sphereTraceVector3Length(intersectionMinusStart);
			float theta = min(1.0f, asinf(length / imposedRadius));
			ST_Vector3 dirComp = sphereTraceVector3Scale(negativeDir, length / tanf(theta));
			if (length == 0.0f)
				dirComp = sphereTraceVector3Scale(negativeDir, imposedRadius);
			pData->normal = sphereTraceVector3Add(intersectionMinusStart, dirComp);
			pData->hitPoint = sphereTraceVector3Add(pData->normal, p0);
			//stupid hack
			if (sphereTraceVector3Nan(pData->hitPoint))
				return 0;
			pData->startPoint = start;
			pData->distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->hitPoint, start));
			pData->normal = sphereTraceVector3Normalize(pData->normal);
		}
	}

	return hits;
}

b32 sphereTraceColliderSphereSphereTrace(ST_Vector3 start, ST_Vector3 dir, float radius, const ST_SphereCollider* const pSphere, ST_SphereTraceData* const pData)
{
	dir = sphereTraceVector3Normalize(dir);
	ST_Vector3 dp = sphereTraceVector3Subtract(start, pSphere->pRigidBody->position);
	float sphereRadiusPlusSphereCastRadius = pSphere->radius + radius;
	float dpDist = sphereTraceVector3Length(dp);
	//ST_Vector3 p0 = sphere->pRigidBody->position;
	if (dpDist <= sphereRadiusPlusSphereCastRadius)
	{
		pData->rayTraceData.startPoint = start;
		pData->radius = radius;
		pData->rayTraceData.normal = sphereTraceVector3Scale(dp, 1.0f / dpDist);
		pData->sphereCenter = start;
		pData->rayTraceData.hitPoint = sphereTraceVector3AddAndScale(pSphere->pRigidBody->position, pData->rayTraceData.normal, pSphere->radius);
		pData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->rayTraceData.hitPoint, start));
		pData->traceDistance = 0.0f;
		return 1;
	}
	b32 hits = 0;
	ST_Vector3 negativeDir = sphereTraceVector3Negative(dir);
	float dist = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphere->pRigidBody->position, start), negativeDir) / sphereTraceVector3Dot(dir, negativeDir);

	if (dist >= 0.0f)
	{
		ST_Vector3 intersection = sphereTraceVector3Add(start, sphereTraceVector3Scale(dir, dist));
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
			pData->rayTraceData.normal = sphereTraceVector3Add(intersectionMinusStart, dirComp);
			pData->sphereCenter = sphereTraceVector3Add(pSphere->pRigidBody->position, pData->rayTraceData.normal);
			pData->rayTraceData.normal = sphereTraceVector3Normalize(pData->rayTraceData.normal);
			pData->rayTraceData.hitPoint = sphereTraceVector3AddAndScale(pSphere->pRigidBody->position, pData->rayTraceData.normal, pSphere->radius);
			pData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pData->rayTraceData.hitPoint, start));
			pData->traceDistance = sphereTraceVector3Distance(pData->rayTraceData.startPoint, pData->sphereCenter);
		}
	}

	return hits;
}



b32 sphereTraceColliderPlaneSphereCollisionTest(const ST_PlaneCollider* const pPlaneCollider, const ST_SphereCollider* const pSphereCollider, ST_SpherePlaneContactInfo* const contactInfo)
{
	ST_RayTraceData rcd;
	b32 didHit;
	float dpDotNormal = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, pPlaneCollider->position), pPlaneCollider->normal);
	if (dpDotNormal >= 0.0f)
		didHit = sphereTraceColliderPlaneRayTrace(pSphereCollider->pRigidBody->position, sphereTraceVector3Negative(pPlaneCollider->normal), pPlaneCollider, &rcd);
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
				&& sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, sphereTraceVector3Negative(pPlaneCollider->forward), pSphereCollider, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		case PLANE_EDGE_FORWARD:
		{
			//printf("plane edge forwards\n");
			didHit = sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, sphereTraceVector3Negative(pPlaneCollider->right), pSphereCollider, &rcd)
				&& sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, pPlaneCollider->right, pSphereCollider, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		case PLANE_EDGE_LEFT:
		{
			didHit = sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, sphereTraceVector3Negative(pPlaneCollider->forward), pSphereCollider, &rcd)
				&& sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, pPlaneCollider->forward, pSphereCollider, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		case PLANE_EDGE_BACK:
		{
			didHit = sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, pPlaneCollider->right, pSphereCollider, &rcd)
				&& sphereTraceColliderSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, sphereTraceVector3Negative(pPlaneCollider->right), pSphereCollider, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		}

		if (didHit)
		{
			contactInfo->collisionType = COLLISION_FACE_EDGE;
			contactInfo->normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, contactPos));
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
			contactInfo->normal = sphereTraceVector3Scale(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, contactPos), 1.0f / dist);
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


b32 sphereTraceColliderTriangleSphereCollisionTest(const ST_TriangleCollider* const pTriangleCollider, const ST_SphereCollider* const pSphereCollider, ST_SphereTriangleContactInfo* const contactInfo)
{
	ST_RayTraceData rcd;
	b32 didHit;
	float dpDotNormal = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, pTriangleCollider->centroid), pTriangleCollider->normal);
	if (dpDotNormal >= 0.0f)
		didHit = sphereTraceColliderTriangleRayTrace(pSphereCollider->pRigidBody->position, sphereTraceVector3Negative(pTriangleCollider->normal), pTriangleCollider, &rcd);
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
		didHit = sphereTraceColliderSphereRayTrace(pTriangleCollider->transformedEdges[closestEdge].point1, pTriangleCollider->edgeDirs[closestEdge], pSphereCollider, &rcd)
			&& sphereTraceColliderSphereRayTrace(pTriangleCollider->transformedEdges[closestEdge].point2, sphereTraceVector3Negative(pTriangleCollider->edgeDirs[closestEdge]), pSphereCollider, &rcd2);
		if (didHit)
			contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
		if (didHit)
		{
			contactInfo->collisionType = COLLISION_FACE_EDGE;
			contactInfo->normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, contactPos));
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
			contactInfo->normal = sphereTraceVector3Scale(sphereTraceVector3Subtract(pSphereCollider->pRigidBody->position, contactPos), 1.0f / dist);
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

b32 sphereTraceColliderPlaneImposedSphereCollisionTest(const ST_PlaneCollider* const pPlaneCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SpherePlaneContactInfo* const contactInfo)
{
	ST_RayTraceData rcd;
	b32 didHit;
	float dpDotNormal = sphereTraceVector3Dot(sphereTraceVector3Subtract(imposedPosition, pPlaneCollider->position), pPlaneCollider->normal);
	if (dpDotNormal >= 0.0f)
		didHit = sphereTraceColliderPlaneRayTrace(imposedPosition, sphereTraceVector3Negative(pPlaneCollider->normal), pPlaneCollider, &rcd);
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
				&& sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, sphereTraceVector3Negative(pPlaneCollider->forward), imposedPosition, imposedRadius, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		case PLANE_EDGE_FORWARD:
		{
			//printf("plane edge forwards\n");
			didHit = sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, sphereTraceVector3Negative(pPlaneCollider->right), imposedPosition, imposedRadius, &rcd)
				&& sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, pPlaneCollider->right, imposedPosition, imposedRadius, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		case PLANE_EDGE_LEFT:
		{
			didHit = sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, sphereTraceVector3Negative(pPlaneCollider->forward), imposedPosition, imposedRadius, &rcd)
				&& sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, pPlaneCollider->forward, imposedPosition, imposedRadius, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		case PLANE_EDGE_BACK:
		{
			didHit = sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point1, pPlaneCollider->right, imposedPosition, imposedRadius, &rcd)
				&& sphereTraceColliderImposedSphereRayTrace(pPlaneCollider->transformedEdges[closestEdge].point2, sphereTraceVector3Negative(pPlaneCollider->right), imposedPosition, imposedRadius, &rcd2);
			if (didHit)
				contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
			break;
		}
		}

		if (didHit)
		{
			contactInfo->collisionType = COLLISION_FACE_EDGE;
			contactInfo->normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(imposedPosition, contactPos));
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
			contactInfo->normal = sphereTraceVector3Scale(sphereTraceVector3Subtract(imposedPosition, contactPos), 1.0f / dist);
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

b32 sphereTraceColliderTriangleImposedSphereCollisionTest(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereTriangleContactInfo* const contactInfo)
{
	ST_RayTraceData rcd;
	b32 didHit;
	float dpDotNormal = sphereTraceVector3Dot(sphereTraceVector3Subtract(imposedPosition, pTriangleCollider->centroid), pTriangleCollider->normal);
	if (dpDotNormal >= 0.0f)
		didHit = sphereTraceColliderTriangleRayTrace(imposedPosition, sphereTraceVector3Negative(pTriangleCollider->normal), pTriangleCollider, &rcd);
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
		didHit = sphereTraceColliderImposedSphereRayTrace(pTriangleCollider->transformedEdges[closestEdge].point1, pTriangleCollider->edgeDirs[closestEdge], imposedPosition, imposedRadius, &rcd)
			&& sphereTraceColliderImposedSphereRayTrace(pTriangleCollider->transformedEdges[closestEdge].point2, sphereTraceVector3Negative(pTriangleCollider->edgeDirs[closestEdge]), imposedPosition, imposedRadius, &rcd2);
		if (didHit)
			contactPos = sphereTraceVector3Average(rcd.hitPoint, rcd2.hitPoint);
		if (didHit)
		{
			contactInfo->collisionType = COLLISION_FACE_EDGE;
			contactInfo->normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(imposedPosition, contactPos));
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
			contactInfo->normal = sphereTraceVector3Scale(sphereTraceVector3Subtract(imposedPosition, contactPos), 1.0f / dist);
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

b32 sphereTraceColliderPlaneSphereTrace(ST_Vector3 from, ST_Vector3 dir, float radius, const ST_PlaneCollider* const pPlaneCollider, ST_SphereTraceData* const pSphereCastData)
{
	ST_SpherePlaneContactInfo contactInfo;
	if (sphereTraceColliderPlaneImposedSphereCollisionTest(pPlaneCollider, from, radius, &contactInfo))
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
		sphereTraceColliderPlaneRayTrace(from, dir, pPlaneCollider, &pSphereCastData->rayTraceData);
		//if (pSphereCastData->rayTraceData.distance < 0.0f)
		//	return 0;
		//rendererDrawSphere(pSphereCastData->rayTraceData.hitPoint, sphereTraceVector3Scale(gVector3One, 0.1f), gQuaternionIdentity, gVector4ColorRed);
		if (fpclassify(pSphereCastData->rayTraceData.distance) == FP_INFINITE)
			return 0;
		ST_Vector3 fromToPlane = sphereTraceVector3Subtract(pPlaneCollider->position, from);
		if (sphereTraceVector3Dot(dir, fromToPlane) < 0.0f)
			return 0;
		float alpha;
		b32 alphaInvalid = 0;
		float dot = sphereTraceVector3Dot(dir, pSphereCastData->rayTraceData.normal);
		float hypotinus;
		ST_Vector3 orthogonal;
		if (dot == 1.0f)
		{
			alpha = 0.0f;
			hypotinus = radius;
		}
		else if (dot == -1.0f)
		{
			alpha = M_PI;
			hypotinus = radius;
			orthogonal = pPlaneCollider->right;
		}
		else
		{
			ST_Vector3 orthogonalRight = sphereTraceVector3Normalize(sphereTraceVector3Cross(pSphereCastData->rayTraceData.normal, dir));

			orthogonal = sphereTraceVector3Cross(pSphereCastData->rayTraceData.normal, orthogonalRight);
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
		ST_Vector3 testPoint;
		ST_Vector3 dp;
		if (alphaInvalid)
		{
			//orthoginal and dir are same
			testPoint = from;
		}
		else
		{
			pSphereCastData->sphereCenter = sphereTraceVector3AddAndScale(from, dir, pSphereCastData->rayTraceData.distance - hypotinus);
			testPoint = sphereTraceVector3AddAndScale(pSphereCastData->sphereCenter, sphereTraceVector3Negative(pSphereCastData->rayTraceData.normal), radius);
			dp = sphereTraceVector3Subtract(testPoint, pPlaneCollider->position);
			float rightDist = sphereTraceVector3Dot(dp, pPlaneCollider->right);
			float fwdDist = sphereTraceVector3Dot(dp, pPlaneCollider->forward);
			if (fabsf(rightDist) <= pPlaneCollider->xHalfExtent && fabsf(fwdDist) <= pPlaneCollider->zHalfExtent)
			{
				pSphereCastData->rayTraceData.hitPoint = testPoint;
				pSphereCastData->rayTraceData.startPoint = from;
				pSphereCastData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pSphereCastData->rayTraceData.hitPoint, from));
				pSphereCastData->radius = radius;
				//pSphereCastData->rayTraceData.startPoint = from;
				pSphereCastData->traceDistance = sphereTraceVector3Distance(pSphereCastData->sphereCenter, pSphereCastData->rayTraceData.startPoint);
				return 1;
			}
		}

		{
			ST_PlaneEdgeDirection closestEdge;
			ST_Vector3 closestPoint;
			if (!alphaInvalid)
				closestEdge = sphereTraceColliderPlaneGetClosestTransformedEdgeToPoint(pPlaneCollider, testPoint);
			else
				closestEdge = sphereTraceColliderPlaneGetClosestTransformedEdgeToSphereTrace(pPlaneCollider, from, dir, radius, &closestPoint);

			ST_Vector3 edgeDir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pPlaneCollider->transformedEdges[closestEdge].point2, pPlaneCollider->transformedEdges[closestEdge].point1));
			//sceneDrawSphereCast(pPlaneCollider->transformedEdges[closestEdge].point2, pPlaneCollider->transformedEdges[closestEdge].point1, 0.05f, gVector4ColorRed);
			//ST_Vector3 closestPointOnEdge = sphereTraceClosestPointOnLineBetweenTwoLines(pPlaneCollider->transformedEdges[closestEdge].point1,
			//	edgeDir,
			//	pSphereCastData->rayTraceData.hitPoint, dir);
			ST_Vector3 right = sphereTraceVector3Normalize(sphereTraceVector3Cross(dir, edgeDir));
			ST_Vector3 sphereDir = sphereTraceVector3Normalize(sphereTraceVector3Cross(right, edgeDir));
			ST_Vector3 fwd = sphereTraceVector3Normalize(sphereTraceVector3Cross(right, dir));
			float theta = acosf(sphereTraceVector3Dot(edgeDir, dir));
			float rightDist;
			if (alphaInvalid)
				rightDist = sphereTraceVector3Dot(sphereTraceVector3Subtract(from, pPlaneCollider->transformedEdges[closestEdge].point1), right);
			else
				rightDist = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereCastData->rayTraceData.hitPoint, pPlaneCollider->transformedEdges[closestEdge].point1), right);

			float maxDist;
			switch (closestEdge)
			{
			case PLANE_EDGE_RIGHT:
			{
				maxDist = pPlaneCollider->zHalfExtent * 2.0f;
				break;
			}
			case PLANE_EDGE_FORWARD:
			{
				maxDist = pPlaneCollider->xHalfExtent * 2.0f;
				break;
			}
			case PLANE_EDGE_LEFT:
			{
				maxDist = pPlaneCollider->zHalfExtent * 2.0f;
				break;
			}
			case PLANE_EDGE_BACK:
			{
				maxDist = pPlaneCollider->xHalfExtent * 2.0f;
				break;
			}
			}
			//ST_Vector3 dp = sphereTraceVector3Subtract(pSphereCastData->rayTraceData.hitPoint, closestPointOnEdge);
			//ST_Vector3 cross = sphereTraceVector3Normalize(sphereTraceVector3Cross(edgeDir, dir));
			//float distToCenterNormal = sphereTraceVector3Dot(dp, cross);
			if (fabsf(rightDist) < radius)
			{
				float beta = acosf(rightDist / radius);
				float fwdDist = sinf(beta) * radius;
				ST_Vector3 pointOnEdgeClosestToCenterRaycast = sphereTraceClosestPointOnLineBetweenTwoLines(pPlaneCollider->transformedEdges[closestEdge].point1, edgeDir, from, dir);
				ST_Vector3 pointOnPlaneNearOriginClosestToEdge = sphereTraceClosestPointOnLineBetweenTwoLines(from, right, pPlaneCollider->transformedEdges[closestEdge].point1, edgeDir);
				pointOnPlaneNearOriginClosestToEdge = sphereTraceVector3AddAndScale(pointOnPlaneNearOriginClosestToEdge, fwd, -fwdDist);
				ST_Vector3 pointOnEdgeTracedFrompointOnPlaneNearOriginClosestToEdge = sphereTraceClosestPointOnLineBetweenTwoLines(pPlaneCollider->transformedEdges[closestEdge].point1, edgeDir, pointOnPlaneNearOriginClosestToEdge, dir);
				float distToPointOnEdgeOrthogonalWithSphereDirectionAndEdge = cosf(theta) * sphereTraceVector3Length(sphereTraceVector3Subtract(pointOnEdgeTracedFrompointOnPlaneNearOriginClosestToEdge, pointOnEdgeClosestToCenterRaycast));
				ST_Vector3 intersection = sphereTraceVector3AddAndScale(pointOnEdgeClosestToCenterRaycast, edgeDir, -distToPointOnEdgeOrthogonalWithSphereDirectionAndEdge);

				fwdDist = fabsf(tanf(theta) * sphereTraceVector3Length(sphereTraceVector3Subtract(pointOnEdgeClosestToCenterRaycast, intersection)));
				ST_Vector3 testPoint2 = sphereTraceClosestPointOnLineBetweenTwoLines(intersection, sphereDir, pointOnEdgeClosestToCenterRaycast, dir);
				if (sphereTraceVector3Nan(testPoint2))
				{
					fwdDist = sinf(beta) * radius;
					testPoint2 = sphereTraceVector3AddAndScale(intersection, sphereDir, fwdDist);
				}
				float edgeDist = sphereTraceVector3Dot(sphereTraceVector3Subtract(intersection, pPlaneCollider->transformedEdges[closestEdge].point1), edgeDir);

				//float edgeDist = sphereTraceVector3Dot(edgeDir, sphereTraceVector3Subtract(closestPointOnEdge, pPlaneCollider->transformedEdges[closestEdge].point1));
				if (edgeDist >= 0.0f && edgeDist <= maxDist)
				{
					//pSphereCastData->rayTraceData.startPoint = from;
					//pSphereCastData->rayTraceData.hitPoint = closestPointOnEdge;
					//pSphereCastData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pSphereCastData->rayTraceData.startPoint, pSphereCastData->rayTraceData.hitPoint));
					//pSphereCastData->radius = radius;
					////float beta = acosf(distToCenterNormal / radius);
					////float opposite = sinf(beta) * radius;
					//pSphereCastData->sphereCenter = sphereTraceVector3AddAndScale(sphereTraceVector3AddAndScale(closestPointOnEdge, cross, distToCenterNormal), dir, -opposite);
					//pSphereCastData->rayTraceData.normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pSphereCastData->sphereCenter, pSphereCastData->rayTraceData.hitPoint));
					//sphereTraceVector3Print(sphereTraceVector3Normalize(sphereTraceVector3Subtract(pSphereCastData->sphereCenter, from)));
					//pSphereCastData->rayTraceData.normal = sphereTrace;
					pSphereCastData->rayTraceData.startPoint = from;
					pSphereCastData->rayTraceData.hitPoint = intersection;
					pSphereCastData->rayTraceData.distance = sphereTraceVector3Length(sphereTraceVector3Subtract(pSphereCastData->rayTraceData.startPoint, pSphereCastData->rayTraceData.hitPoint));
					pSphereCastData->radius = radius;
					pSphereCastData->sphereCenter = sphereTraceVector3Add(sphereTraceVector3AddAndScale(pointOnPlaneNearOriginClosestToEdge, right, rightDist), sphereTraceVector3Subtract(testPoint2, pointOnPlaneNearOriginClosestToEdge));
					pSphereCastData->rayTraceData.normal = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pSphereCastData->sphereCenter, intersection));
					pSphereCastData->traceDistance = sphereTraceVector3Distance(pSphereCastData->sphereCenter, pSphereCastData->rayTraceData.startPoint);
					return 1;
				}
				//else
				{
					//check point
					if (!alphaInvalid)
					{
						closestPoint = pPlaneCollider->transformedVertices[sphereTraceColliderPlaneGetClosestTransformedVertexToPoint(pPlaneCollider, pSphereCastData->rayTraceData.hitPoint)];
						dp = sphereTraceVector3Subtract(pSphereCastData->rayTraceData.hitPoint, closestPoint);
					}
					else
					{
						ST_Vector3 hitPoint = sphereTraceClosestPointOnLineBetweenTwoLines(from, dir, closestPoint, pPlaneCollider->normal);
						dp = sphereTraceVector3Subtract(hitPoint, closestPoint);
					}

					ST_Vector3 cross = sphereTraceVector3Normalize(sphereTraceVector3Cross(sphereTraceVector3Normalize(dp), dir));
					cross = sphereTraceVector3Cross(dir, cross);
					float distToCenterNormal = sphereTraceVector3Dot(dp, cross);
					if (fabsf(distToCenterNormal) < COLLIDER_TOLERANCE)
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
	}
	return 0;
}


b32 sphereTraceColliderTriangleSphereTrace(ST_Vector3 from, ST_Vector3 dir, float radius, const ST_TriangleCollider* const pTriangleCollider, ST_SphereTraceData* const pSphereCastData);

b32 sphereTraceColliderSphereSphereCollisionTest(const ST_SphereCollider* const pSphereColliderA, const ST_SphereCollider* const pSphereColliderB, ST_SphereSphereContactInfo* const pContactInfo)
{
	ST_Vector3 dp = sphereTraceVector3Subtract(pSphereColliderB->pRigidBody->position, pSphereColliderA->pRigidBody->position);
	float dist = sphereTraceVector3Length(dp);
	float rab = pSphereColliderA->radius + pSphereColliderB->radius;
	if (dist <= rab)
	{
		pContactInfo->normal = sphereTraceVector3Scale(dp, 1.0f / dist);
		pContactInfo->pA = pSphereColliderA;
		pContactInfo->pB = pSphereColliderB;
		pContactInfo->penetrationDistance = rab - dist;
		ST_Vector3 pIntA = sphereTraceVector3AddAndScale(pSphereColliderA->pRigidBody->position, pContactInfo->normal, pSphereColliderA->radius);
		ST_Vector3 pIntB = sphereTraceVector3AddAndScale(pSphereColliderB->pRigidBody->position, sphereTraceVector3Negative(pContactInfo->normal), pSphereColliderB->radius);
		pContactInfo->point = sphereTraceVector3Average(pIntA, pIntB);
		return 1;
	}
	else
		return 0;
}

typedef struct ST_UniformTerrainCollider
{
	ST_TriangleCollider* triangles;
	int xCells;
	int zCells;
	float xSize;
	float zSize;
	float cellSize;
	ST_AABB aabb;
	float angle;
	ST_Vector3 right;
	ST_Vector3 forward;
	ST_Vector3 position;
	ST_Quaternion rotation;
	ST_IntList bucketIndices;
	ST_Vector3 halfExtents;

	//needed for quick raytrace
	//replace with obb when implemented
	ST_PlaneCollider rightPlane;
	ST_PlaneCollider topPlane;
	ST_PlaneCollider forwardPlane;
	ST_PlaneCollider leftPlane;
	ST_PlaneCollider bottomPlane;
	ST_PlaneCollider backPlane;
} ST_UniformTerrainCollider;

ST_UniformTerrainCollider sphereTraceColliderUniformTerrainConstruct(int xCells, int zCells, float cellSize)
{
	ST_UniformTerrainCollider terrain;
	terrain.xCells = xCells;
	terrain.zCells = zCells;
	terrain.cellSize = cellSize;
	terrain.xSize = xCells * cellSize;
	terrain.zSize = zCells * cellSize;
	terrain.triangles = (ST_TriangleCollider*)malloc(2 * xCells * 2 * zCells * sizeof(ST_TriangleCollider));
	terrain.aabb.halfExtents = sphereTraceVector3Construct(xCells * cellSize * 0.5f, 0.0f, zCells * cellSize * 0.5f);
	terrain.aabb.leftDownBackTransformedVertex = sphereTraceVector3Construct(0.0f, 0.0f, 0.0f);
	terrain.aabb.rightTopForwardsTransformedVertex = sphereTraceVector3Construct(xCells * cellSize, 0.0f, zCells * cellSize);
	ST_Vector3 midpoint = sphereTraceVector3Average(terrain.aabb.leftDownBackTransformedVertex, terrain.aabb.rightTopForwardsTransformedVertex);
	terrain.rightPlane = sphereTraceColliderPlaneConstruct(gVector3Right, 0.0f, 0.0f, terrain.aabb.halfExtents.z, (ST_Vector3) { 0.0f, midpoint.y, midpoint.z });
	terrain.topPlane = sphereTraceColliderPlaneConstruct(gVector3Up, 0.0f, terrain.aabb.halfExtents.x, terrain.aabb.halfExtents.z, (ST_Vector3) { midpoint.x, midpoint.y, midpoint.z });
	terrain.forwardPlane = sphereTraceColliderPlaneConstruct(gVector3Forward, 0.0f, terrain.aabb.halfExtents.x, 0.0f, (ST_Vector3) { midpoint.x, midpoint.y, 0.0f });
	terrain.angle = 0.0f;
	terrain.right = gVector3Right;
	terrain.forward = gVector3Forward;
	terrain.position = gVector3Zero;
	terrain.rotation = gQuaternionIdentity;
	return terrain;
}

void sphereTraceColliderUniformTerrainSetTransform(ST_UniformTerrainCollider* const pTerrainCollider, float angle, ST_Vector3 position)
{

	pTerrainCollider->angle = angle;
	pTerrainCollider->right = (ST_Vector3){ cosf(angle), 0.0f, -sinf(angle) };
	pTerrainCollider->forward = (ST_Vector3){ sinf(angle), 0.0f, cosf(angle) };
	pTerrainCollider->rotation = sphereTraceQuaternionFromAngleAxis(gVector3Up, angle);
	pTerrainCollider->aabb.halfExtents = sphereTraceVector3Construct(0.0f, 0.0f, 0.0f);
	pTerrainCollider->aabb.leftDownBackTransformedVertex = position;
	pTerrainCollider->aabb.rightTopForwardsTransformedVertex = position;
	for (int z = 0; z < pTerrainCollider->zCells; z++)
	{
		for (int x = 0; x < pTerrainCollider->xCells; x++)
		{
			int triangleIndex = 2 * x + 2 * z * pTerrainCollider->xCells;
			ST_TriangleCollider* pTC = &pTerrainCollider->triangles[triangleIndex];
			ST_Vector3 v1 = sphereTraceVector3Subtract(pTC->transformedVertices[0], pTerrainCollider->position);
			ST_Vector3 v2 = sphereTraceVector3Subtract(pTC->transformedVertices[1], pTerrainCollider->position);
			ST_Vector3 v3 = sphereTraceVector3Subtract(pTC->transformedVertices[2], pTerrainCollider->position);
			v1 = (ST_Vector3){ cosf(angle) * v1.x + sinf(angle) * v1.z, v1.y,-sinf(angle) * v1.x + cosf(angle) * v1.z };
			v2 = (ST_Vector3){ cosf(angle) * v2.x + sinf(angle) * v2.z, v2.y, -sinf(angle) * v2.x + cosf(angle) * v2.z };
			v3 = (ST_Vector3){ cosf(angle) * v3.x + sinf(angle) * v3.z, v3.y, -sinf(angle) * v3.x + cosf(angle) * v3.z };
			sphereTraceColliderTriangleSetVertexAndEdgeData(pTC, sphereTraceVector3Add(v1, position), sphereTraceVector3Add(v2, position), sphereTraceVector3Add(v3, position));
			//sphereTraceColliderTriangleSetAABB(pTC);
			sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&pTerrainCollider->aabb, &pTerrainCollider->triangles[triangleIndex].aabb);

			triangleIndex++;
			pTC = &pTerrainCollider->triangles[triangleIndex];
			v1 = sphereTraceVector3Subtract(pTC->transformedVertices[0], pTerrainCollider->position);
			v2 = sphereTraceVector3Subtract(pTC->transformedVertices[1], pTerrainCollider->position);
			v3 = sphereTraceVector3Subtract(pTC->transformedVertices[2], pTerrainCollider->position);
			v1 = (ST_Vector3){ cosf(angle) * v1.x + sinf(angle) * v1.z, v1.y,-sinf(angle) * v1.x + cosf(angle) * v1.z };
			v2 = (ST_Vector3){ cosf(angle) * v2.x + sinf(angle) * v2.z, v2.y, -sinf(angle) * v2.x + cosf(angle) * v2.z };
			v3 = (ST_Vector3){ cosf(angle) * v3.x + sinf(angle) * v3.z, v3.y, -sinf(angle) * v3.x + cosf(angle) * v3.z };
			sphereTraceColliderTriangleSetVertexAndEdgeData(pTC, sphereTraceVector3Add(v1, position), sphereTraceVector3Add(v2, position), sphereTraceVector3Add(v3, position)); sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&pTerrainCollider->aabb, &pTerrainCollider->triangles[triangleIndex].aabb);
			//sphereTraceColliderTriangleSetAABB(pTC);
			sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&pTerrainCollider->aabb, &pTerrainCollider->triangles[triangleIndex].aabb);
		}
	}
	pTerrainCollider->position = position;
	ST_Vector3 midpoint = sphereTraceVector3Construct(pTerrainCollider->xCells * pTerrainCollider->cellSize * 0.5f, 0.5f*(pTerrainCollider->aabb.leftDownBackTransformedVertex.y+ pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y), pTerrainCollider->zCells * pTerrainCollider->cellSize * 0.5f);
	midpoint = sphereTraceVector3Add(position, (ST_Vector3) { cosf(angle)* midpoint.x + sinf(angle) * midpoint.z, midpoint.y, -sinf(angle) * midpoint.x + cosf(angle) * midpoint.z });
	ST_Vector3 endpoint = sphereTraceVector3Construct(pTerrainCollider->xCells * pTerrainCollider->cellSize , pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y, pTerrainCollider->zCells * pTerrainCollider->cellSize);
	endpoint = sphereTraceVector3Add(position, (ST_Vector3) { cosf(angle)* endpoint.x + sinf(angle) * endpoint.z, endpoint.y, -sinf(angle) * endpoint.x + cosf(angle) * endpoint.z });
	pTerrainCollider->leftPlane = sphereTraceColliderPlaneConstruct(sphereTraceVector3Negative(pTerrainCollider->right), 0.0f, pTerrainCollider->leftPlane.xHalfExtent, pTerrainCollider->leftPlane.zHalfExtent, sphereTraceVector3Add(position, (ST_Vector3) { cosf(angle)* pTerrainCollider->leftPlane.position.x + sinf(angle) * pTerrainCollider->leftPlane.position.z, pTerrainCollider->leftPlane.position.y, -sinf(angle) * pTerrainCollider->leftPlane.position.x + cosf(angle) * pTerrainCollider->leftPlane.position.z }));
	pTerrainCollider->topPlane = sphereTraceColliderPlaneConstruct(gVector3Up, angle, pTerrainCollider->topPlane.xHalfExtent, pTerrainCollider->topPlane.zHalfExtent, (ST_Vector3) { midpoint.x, pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y, midpoint.z });
	pTerrainCollider->backPlane = sphereTraceColliderPlaneConstruct(sphereTraceVector3Negative(pTerrainCollider->forward), 0.0f, pTerrainCollider->backPlane.zHalfExtent, pTerrainCollider->backPlane.xHalfExtent, sphereTraceVector3Add(position, (ST_Vector3) { cosf(angle)* pTerrainCollider->backPlane.position.x + sinf(angle) * pTerrainCollider->backPlane.position.z, pTerrainCollider->backPlane.position.y, -sinf(angle) * pTerrainCollider->backPlane.position.x + cosf(angle) * pTerrainCollider->backPlane.position.z }));
	pTerrainCollider->rightPlane = sphereTraceColliderPlaneConstruct(pTerrainCollider->right, 0.0f, pTerrainCollider->rightPlane.xHalfExtent, pTerrainCollider->rightPlane.zHalfExtent, sphereTraceVector3Add(position, (ST_Vector3) { cosf(angle)* pTerrainCollider->rightPlane.position.x + sinf(angle) * pTerrainCollider->rightPlane.position.z, pTerrainCollider->rightPlane.position.y, -sinf(angle) * pTerrainCollider->rightPlane.position.x + cosf(angle) * pTerrainCollider->rightPlane.position.z }));
	pTerrainCollider->bottomPlane = sphereTraceColliderPlaneConstruct(gVector3Down, angle, pTerrainCollider->bottomPlane.xHalfExtent, pTerrainCollider->bottomPlane.zHalfExtent, (ST_Vector3) { midpoint.x, pTerrainCollider->aabb.leftDownBackTransformedVertex.y, midpoint.z });
	pTerrainCollider->forwardPlane = sphereTraceColliderPlaneConstruct(pTerrainCollider->forward, 0.0f, pTerrainCollider->forwardPlane.zHalfExtent, pTerrainCollider->forwardPlane.xHalfExtent, sphereTraceVector3Add(position, (ST_Vector3) { cosf(angle)* pTerrainCollider->forwardPlane.position.x + sinf(angle) * pTerrainCollider->forwardPlane.position.z, pTerrainCollider->forwardPlane.position.y, -sinf(angle) * pTerrainCollider->forwardPlane.position.x + cosf(angle) * pTerrainCollider->forwardPlane.position.z }));


}

void sphereTraceColliderUniformTerrainFillTrianglesWithFunction(ST_UniformTerrainCollider* const terrainCollider, float (*fxz)(float, float))
{
	for (int z = 0; z < terrainCollider->zCells; z++)
	{
		for (int x = 0; x < terrainCollider->xCells; x++)
		{
			int triangleIndex = 2 * x + 2 * z * terrainCollider->xCells;
			//(0,0), (1,0), (0,1)
			float x1 = x * terrainCollider->cellSize;
			float x2 = (x + 1) * terrainCollider->cellSize;
			float z1 = z * terrainCollider->cellSize;
			float z2 = (z + 1) * terrainCollider->cellSize;
			terrainCollider->triangles[triangleIndex] = sphereTraceColliderTriangleConstruct((ST_Vector3) { x1, fxz(x1, z1), z1 },
				(ST_Vector3) {
				x1, fxz(x1, z2), z2
			},
				(ST_Vector3) {
				x2, fxz(x2, z1), z1
			});
			sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&terrainCollider->aabb, &terrainCollider->triangles[triangleIndex].aabb);
			triangleIndex++;
			//(1,0), (1,1), (0,1)
			terrainCollider->triangles[triangleIndex] = sphereTraceColliderTriangleConstruct((ST_Vector3) { x2, fxz(x2, z1), z1 },
				(ST_Vector3) {
				x1, fxz(x1, z2), z2
			},
				(ST_Vector3) {
				x2, fxz(x2, z2), z2
			});
			sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&terrainCollider->aabb, &terrainCollider->triangles[triangleIndex].aabb);
		}
	}

	ST_Vector3 midpoint = sphereTraceVector3Average(terrainCollider->aabb.leftDownBackTransformedVertex, terrainCollider->aabb.rightTopForwardsTransformedVertex);
	terrainCollider->leftPlane = sphereTraceColliderPlaneConstruct(sphereTraceVector3Negative(gVector3Right), 0.0f, terrainCollider->aabb.halfExtents.y, terrainCollider->aabb.halfExtents.z, (ST_Vector3) { 0.0f, midpoint.y, midpoint.z });
	terrainCollider->topPlane = sphereTraceColliderPlaneConstruct(gVector3Up, 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.z, (ST_Vector3) { midpoint.x, midpoint.y, midpoint.z });
	terrainCollider->backPlane = sphereTraceColliderPlaneConstruct(sphereTraceVector3Negative(gVector3Forward), 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.y, (ST_Vector3) { midpoint.x, midpoint.y, 0.0f });
	terrainCollider->rightPlane = sphereTraceColliderPlaneConstruct(gVector3Right, 0.0f, terrainCollider->aabb.halfExtents.y, terrainCollider->aabb.halfExtents.z, (ST_Vector3) { terrainCollider->aabb.rightTopForwardsTransformedVertex.x, midpoint.y, midpoint.z });
	terrainCollider->bottomPlane = sphereTraceColliderPlaneConstruct(gVector3Down, 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.z, (ST_Vector3) { midpoint.x, 0.0f, midpoint.z });
	terrainCollider->forwardPlane = sphereTraceColliderPlaneConstruct(gVector3Forward, 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.y, (ST_Vector3) { midpoint.x, midpoint.y, terrainCollider->aabb.rightTopForwardsTransformedVertex.z });

}

void sphereTraceColliderUniformTerrainFillTrianglesWithFunctionAndConditionalFunction(ST_UniformTerrainCollider* const terrainCollider, float (*fxz)(float, float), b32 (*conditionalFunc)(float (*fxz)(float, float), float, float))
{
	for (int z = 0; z < terrainCollider->zCells; z++)
	{
		for (int x = 0; x < terrainCollider->xCells; x++)
		{
			int triangleIndex = 2 * x + 2 * z * terrainCollider->xCells;
			//(0,0), (1,0), (0,1)
			float x1 = x * terrainCollider->cellSize;
			float x2 = (x + 1) * terrainCollider->cellSize;
			float z1 = z * terrainCollider->cellSize;
			float z2 = (z + 1) * terrainCollider->cellSize;
			if (conditionalFunc(fxz, x1, z1))
			{
				terrainCollider->triangles[triangleIndex] = sphereTraceColliderTriangleConstruct((ST_Vector3) { x1, fxz(x1, z1), z1 },
					(ST_Vector3) {
					x1, fxz(x1, z2), z2
				},
					(ST_Vector3) {
					x2, fxz(x2, z1), z1
				});
				sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&terrainCollider->aabb, &terrainCollider->triangles[triangleIndex].aabb);
				triangleIndex++;
				//(1,0), (1,1), (0,1)
				terrainCollider->triangles[triangleIndex] = sphereTraceColliderTriangleConstruct((ST_Vector3) { x2, fxz(x2, z1), z1 },
					(ST_Vector3) {
					x1, fxz(x1, z2), z2
				},
					(ST_Vector3) {
					x2, fxz(x2, z2), z2
				});
				sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&terrainCollider->aabb, &terrainCollider->triangles[triangleIndex].aabb);
			}
			else
			{
				terrainCollider->triangles[triangleIndex] = sphereTraceColliderTriangleConstruct(terrainCollider->position, terrainCollider->position, terrainCollider->position);
				terrainCollider->triangles[triangleIndex++].ignoreCollisions = 1;
				terrainCollider->triangles[triangleIndex] = sphereTraceColliderTriangleConstruct(terrainCollider->position, terrainCollider->position, terrainCollider->position);
				terrainCollider->triangles[triangleIndex].ignoreCollisions = 1;
			}
		}
	}

	ST_Vector3 midpoint = sphereTraceVector3Average(terrainCollider->aabb.leftDownBackTransformedVertex, terrainCollider->aabb.rightTopForwardsTransformedVertex);
	terrainCollider->leftPlane = sphereTraceColliderPlaneConstruct(sphereTraceVector3Negative(gVector3Right), 0.0f, terrainCollider->aabb.halfExtents.y, terrainCollider->aabb.halfExtents.z, (ST_Vector3) { 0.0f, midpoint.y, midpoint.z });
	terrainCollider->topPlane = sphereTraceColliderPlaneConstruct(gVector3Up, 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.z, (ST_Vector3) { midpoint.x, midpoint.y, midpoint.z });
	terrainCollider->backPlane = sphereTraceColliderPlaneConstruct(sphereTraceVector3Negative(gVector3Forward), 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.y, (ST_Vector3) { midpoint.x, midpoint.y, 0.0f });
	terrainCollider->rightPlane = sphereTraceColliderPlaneConstruct(gVector3Right, 0.0f, terrainCollider->aabb.halfExtents.y, terrainCollider->aabb.halfExtents.z, (ST_Vector3) { terrainCollider->aabb.rightTopForwardsTransformedVertex.x, midpoint.y, midpoint.z });
	terrainCollider->bottomPlane = sphereTraceColliderPlaneConstruct(gVector3Down, 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.z, (ST_Vector3) { midpoint.x, 0.0f, midpoint.z });
	terrainCollider->forwardPlane = sphereTraceColliderPlaneConstruct(gVector3Forward, 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.y, (ST_Vector3) { midpoint.x, midpoint.y, terrainCollider->aabb.rightTopForwardsTransformedVertex.z });
}

int sphereTraceColliderUniformTerrainSampleFirstTriangleIndex(const ST_UniformTerrainCollider* const terrainCollider, ST_Vector3 samplePosition)
{
	//if (sphereTraceColliderAABBIsPointInside(&terrainCollider->aabb, (ST_Vector3) { samplePosition.x, terrainCollider->aabb.leftDownBackTransformedVertex.y, samplePosition.z }))
	//{
		float dpx = samplePosition.x - terrainCollider->position.x;
		float dpz = samplePosition.z - terrainCollider->position.z;
		float xDist = (dpx * terrainCollider->right.x + dpz * terrainCollider->right.z);
		float zDist = (dpx * terrainCollider->forward.x + dpz * terrainCollider->forward.z);
		if (xDist <= terrainCollider->xSize && zDist <= terrainCollider->zSize && xDist >= 0.0f && zDist >= 0.0f)
		{
			int x = (int)(xDist / terrainCollider->cellSize);
			int z = (int)(zDist / terrainCollider->cellSize);
			return 2 * x + 2 * z * terrainCollider->xCells;
		}
		else
			return -1;
	//}
	//else
	//{
	//	return -1;
	//}
}

ST_IntList sphereTraceColliderUniformTerrainSampleTriangleIndicesForSphere(const ST_UniformTerrainCollider* const terrainCollider, ST_Vector3 spherePosition, float radius)
{
	float maxCheckDistRight = radius * 2.0f + terrainCollider->cellSize;
	float maxCheckDistFwd = radius * 2.0f + terrainCollider->cellSize;
	//ST_Vector2 maxCheckPos = { paabb->rightTopForwardsTransformedVertex.x + terrainCollider->cellSize, paabb->rightTopForwardsTransformedVertex.z + terrainCollider->cellSize };
	ST_IntList indices = sphereTraceIntListConstruct();
	float distRight = 0.0f;
	float distFwd = 0.0f;
	ST_Vector3 posStart = sphereTraceVector3AddAndScale(sphereTraceVector3AddAndScale(spherePosition, terrainCollider->leftPlane.normal, radius), terrainCollider->backPlane.normal, radius);
	for (float distFwd = 0.0f; distFwd < maxCheckDistFwd; distFwd += terrainCollider->cellSize)
	{
		for (float distRight = 0.0f; distRight < maxCheckDistRight; distRight += terrainCollider->cellSize)
		{
			ST_Vector3 pos = sphereTraceVector3AddAndScale(sphereTraceVector3AddAndScale(posStart, terrainCollider->rightPlane.normal, distRight), terrainCollider->forwardPlane.normal, distFwd);
			int index = sphereTraceColliderUniformTerrainSampleFirstTriangleIndex(terrainCollider, (ST_Vector3) { pos.x, terrainCollider->aabb.leftDownBackTransformedVertex.y, pos.z });
			if (index != -1)
			{
				sphereTraceIntListAddFirst(&indices, index);
				sphereTraceIntListAddFirst(&indices, index+1);
			}

		}
	}
	return indices;
}


b32 sphereTraceColliderUniformTerrainImposedSphereFindMaxPenetratingTriangle(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereTerrainTriangleContactInfo* const pContactInfo)
{
	ST_IntListData* pIl;
	ST_AABB imposedAABB;
	imposedAABB.halfExtents = (ST_Vector3){ imposedRadius, imposedRadius , imposedRadius };
	imposedAABB.rightTopForwardsTransformedVertex = sphereTraceVector3Add(imposedPosition, imposedAABB.halfExtents);
	imposedAABB.leftDownBackTransformedVertex = sphereTraceVector3Subtract(imposedPosition, imposedAABB.halfExtents);
	ST_IntList il = sphereTraceColliderUniformTerrainSampleTriangleIndicesForSphere(pTerrainCollider, imposedPosition, imposedRadius);
	pIl = il.pFirst;
	//float maxPen = -FLT_MAX;
	ST_IntList penetratingIndices = sphereTraceIntListConstruct();
	for (int i = 0; i < il.count; i++)
	{

		int index = pIl->value;
		if (!pTerrainCollider->triangles[index].ignoreCollisions)
		{
			if (sphereTraceColliderAABBIntersectAABBVertically(&imposedAABB, &pTerrainCollider->triangles[index].aabb))
			{
				if (sphereTraceColliderAABBIntersectImposedSphere(&pTerrainCollider->triangles[index].aabb, imposedPosition, imposedRadius))
				{
					ST_SphereTriangleContactInfo ciTest;
					if (sphereTraceColliderTriangleImposedSphereCollisionTest(&pTerrainCollider->triangles[index], imposedPosition, imposedRadius, &ciTest))
					{
						sphereTraceIntListAddFirst(&penetratingIndices, index);
						//if (ciTest.penetrationDistance > maxPen)
						//{
						//	maxPen = ciTest.penetrationDistance;
						//	*pContactInfo = ciTest;
						//}
					}
				}
			}
		}
		pIl = pIl->pNext;
	}
	sphereTraceIntListFree(&il);
	if (penetratingIndices.count > 0)
	{
		//now find the point at which the triangle is contacted with the sphere on the y axis
		pIl = penetratingIndices.pFirst;
		ST_Vector3 castPoint = sphereTraceVector3Construct(imposedPosition.x, pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y+imposedRadius, imposedPosition.z);
		float minDist = FLT_MAX;
		int minIndex;
		ST_SphereTraceData std;
		for (int i = 0; i < penetratingIndices.count; i++)
		{
			if (sphereTraceColliderTriangleSphereTrace(castPoint, gVector3Down, imposedRadius, &pTerrainCollider->triangles[pIl->value], &std))
			{
				float height = castPoint.y - std.sphereCenter.y;
				if (height < minDist)
				{
					minDist = height;
					minIndex = pIl->value;
					pContactInfo->downSphereTraceData = std;
					pContactInfo->downSphereTraceData.traceDistance = 0.0f;
				}
			}

			pIl = pIl->pNext;
		}
		//now recollect the contact info for the sphere-triangle
		sphereTraceColliderTriangleImposedSphereCollisionTest(&pTerrainCollider->triangles[minIndex], imposedPosition, imposedRadius, &pContactInfo->sphereTriangleContactInfo);
		sphereTraceIntListFree(&penetratingIndices);
		return 1;
	}
	else
	{
		return 0;
	}
}

b32 sphereTraceColliderUniformTerrainSphereFindMaxPenetratingTriangle(const ST_UniformTerrainCollider* const pTerrainCollider, const ST_SphereCollider* const pSphereCollider, ST_SphereTerrainTriangleContactInfo* const pContactInfo)
{
	b32 res = sphereTraceColliderUniformTerrainImposedSphereFindMaxPenetratingTriangle(pTerrainCollider, pSphereCollider->pRigidBody->position, pSphereCollider->radius, pContactInfo);
	if (res)
	{
		pContactInfo->sphereTriangleContactInfo.pSphereCollider = pSphereCollider;
	}
	return res;
}


b32 sphereTraceColliderTriangleSphereTrace(ST_Vector3 from, ST_Vector3 dir, float radius, const ST_TriangleCollider* const pTriangleCollider, ST_SphereTraceData* const pSphereCastData)
{
	ST_SpherePlaneContactInfo contactInfo;
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
		if (fpclassify(pSphereCastData->rayTraceData.distance) == -FP_INFINITE)
			return 0;
		ST_Vector3 fromToPlane = sphereTraceVector3Subtract(pTriangleCollider->centroid, from);
		if (sphereTraceVector3Dot(dir, fromToPlane) < 0.0f)
			return 0;
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
			ST_Vector3 sphereDir = sphereTraceVector3Normalize(sphereTraceVector3Cross(right, edgeDir));
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


b32 sphereTraceColliderUniformTerrainRayTrace(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 from, ST_Vector3 dir, ST_RayTraceData* const pRayTraceData)
{
	dir = sphereTraceVector3Normalize(dir);
	if (dir.x == 0.0f && dir.z == 0.0f)
	{
		int triangleInd = sphereTraceColliderUniformTerrainSampleFirstTriangleIndex(pTerrainCollider, from);
		if (triangleInd != -1)
		{
			if (sphereTraceColliderTriangleRayTrace(from, dir, &pTerrainCollider->triangles[triangleInd], pRayTraceData))
			{
				return 1;
			}
			else if (sphereTraceColliderTriangleRayTrace(from, dir, &pTerrainCollider->triangles[triangleInd + 1], pRayTraceData))
			{
				return 1;
			}
		}
		return 0;
	}
	ST_RayTraceData rtcPlane;
	b32 doesIntersectWithTerrainBoundingBox = 0;
	b32 xIncrementing = 0;
	b32 yIncrementing = 0;
	b32 zIncrementing = 0;
	float magX = sphereTraceVector3Dot(dir, pTerrainCollider->rightPlane.normal);
	float magY = sphereTraceVector3Dot(dir, pTerrainCollider->topPlane.normal);
	float magZ = sphereTraceVector3Dot(dir, pTerrainCollider->forwardPlane.normal);
	ST_Vector3 intersectingDirection;
	ST_Vector3 intersection = from;
	ST_Vector3 dp = sphereTraceVector3Subtract(from, pTerrainCollider->position);
	float dpx = intersection.x - pTerrainCollider->position.x;
	float dpz = intersection.z - pTerrainCollider->position.z;
	float xDist = (dpx * pTerrainCollider->right.x + dpz * pTerrainCollider->right.z);
	float zDist = (dpx * pTerrainCollider->forward.x + dpz * pTerrainCollider->forward.z);
	float height = intersection.y;
	if (xDist <= pTerrainCollider->xSize && zDist <= pTerrainCollider->zSize && height <= pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y && xDist >= 0.0f && zDist >= 0.0f && height >= pTerrainCollider->aabb.leftDownBackTransformedVertex.y)
	{
		doesIntersectWithTerrainBoundingBox = 1;
		if (magX > 0.0f)
			xIncrementing = 1;
		if (magY > 0.0f)
			yIncrementing = 1;
		if (magZ > 0.0f)
			zIncrementing = 1;
	}
	else
	{
		if (magX <= 0.0f)
		{
			if (sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->rightPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		else
		{
			xIncrementing = 1;
			if (sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->leftPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		if (magY <= 0.0f)
		{
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->topPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		else
		{
			yIncrementing = 1;
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->bottomPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		if (magZ < 0.0f)
		{
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->forwardPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		else
		{
			zIncrementing = 1;
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->backPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
	}
	if (doesIntersectWithTerrainBoundingBox)
	{
		dpx = intersection.x - pTerrainCollider->position.x;
		dpz = intersection.z - pTerrainCollider->position.z;
		xDist = (dpx * pTerrainCollider->right.x + dpz * pTerrainCollider->right.z);
		zDist = (dpx * pTerrainCollider->forward.x + dpz * pTerrainCollider->forward.z);
		height = intersection.y;
		float tMin = FLT_MAX;
		b32 xChanged = 0;
		b32 zChanged = 0;
		ST_AABB heightBox;
		int xInd = (int)(xDist / pTerrainCollider->cellSize);
		int zInd = (int)(zDist / pTerrainCollider->cellSize);
		while (xDist <= pTerrainCollider->xSize && zDist <= pTerrainCollider->zSize && height <= pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y && xDist >= 0.0f && zDist >= 0.0f && height >= pTerrainCollider->aabb.leftDownBackTransformedVertex.y)
		{


			b32 incomingFromX = 1;
			if (xIncrementing)
			{
				if (xChanged)
					xInd++;
				else
					xInd = (int)(xDist / pTerrainCollider->cellSize) + 1;
				float tentativeX = (xInd * pTerrainCollider->cellSize - xDist) / magX;
				if (tentativeX < tMin && tentativeX > 0.0f)
				{
					tMin = tentativeX;
					intersectingDirection = pTerrainCollider->rightPlane.normal;
				}
			}
			else
			{
				if (xChanged)
					xInd--;
				else
					xInd = (int)(xDist / pTerrainCollider->cellSize);
				float tentativeX = (xInd * pTerrainCollider->cellSize - xDist) / magX;
				if (tentativeX < tMin && tentativeX > 0.0f)
				{

					tMin = tentativeX;
					intersectingDirection = pTerrainCollider->leftPlane.normal;
				}
			}

			if (zIncrementing)
			{
				if (zChanged)
					zInd++;
				else
					zInd = (int)(zDist / pTerrainCollider->cellSize) + 1;
				float tentativeT = (zInd * pTerrainCollider->cellSize - zDist) / magZ;
				if (tentativeT < tMin && tentativeT > 0.0f)
				{

					tMin = tentativeT;
					intersectingDirection = pTerrainCollider->forwardPlane.normal;
					incomingFromX = 0;
				}
			}
			else
			{
				if (zChanged)
					zInd--;
				else
					zInd = (int)(zDist / pTerrainCollider->cellSize);
				float tentativeT = (zInd * pTerrainCollider->cellSize - zDist) / magZ;
				if (tentativeT < tMin && tentativeT > 0.0f)
				{

					tMin = tentativeT;
					intersectingDirection = pTerrainCollider->backPlane.normal;
					incomingFromX = 0;
				}
			}
			ST_Vector3 nextIntersection = sphereTraceVector3AddAndScale(intersection, dir, tMin);
			//need to adjust the intersection so we dont get stuck with floating point error
			if (incomingFromX)
			{
				xChanged = 1;
				xDist = xInd * pTerrainCollider->cellSize;
				dpx = nextIntersection.x - pTerrainCollider->position.x;
				dpz = nextIntersection.z - pTerrainCollider->position.z;
				zDist = (dpx * pTerrainCollider->forward.x + dpz * pTerrainCollider->forward.z);
			}
			else
			{
				zChanged = 1;
				dpx = nextIntersection.x - pTerrainCollider->position.x;
				dpz = nextIntersection.z - pTerrainCollider->position.z;
				zDist = zInd * pTerrainCollider->cellSize;
				xDist = (dpx * pTerrainCollider->right.x + dpz * pTerrainCollider->right.z);
			}
			float nextHeight = nextIntersection.y;
			if (yIncrementing)
			{
				heightBox.rightTopForwardsTransformedVertex.y = nextHeight;
				heightBox.leftDownBackTransformedVertex.y = height;
			}
			else
			{
				heightBox.rightTopForwardsTransformedVertex.y = height;
				heightBox.leftDownBackTransformedVertex.y = nextHeight;
			}
			ST_Vector3 samplePos = sphereTraceVector3Average(intersection, nextIntersection);
			int triIndex = sphereTraceColliderUniformTerrainSampleFirstTriangleIndex(pTerrainCollider, samplePos);
			if (triIndex != -1)
			{
				ST_TriangleCollider* pt1 = &pTerrainCollider->triangles[triIndex];
				ST_TriangleCollider* pt2 = &pTerrainCollider->triangles[triIndex + 1];

				if (sphereTraceColliderAABBIntersectAABBVertically(&pt1->aabb, &heightBox))
				{
					if (sphereTraceColliderTriangleRayTrace(from, dir, pt1, pRayTraceData))
					{
						return 1;
					}
				}
				if (sphereTraceColliderAABBIntersectAABBVertically(&pt2->aabb, &heightBox))
				{
					if (sphereTraceColliderTriangleRayTrace(from, dir, pt2, pRayTraceData))
					{
						return 1;
					}
				}
			}

			intersection = nextIntersection;
			height = intersection.y;
			tMin = FLT_MAX;


		}
	}
	return 0;
}

b32 sphereTraceColliderUniformTerrainSphereTraceDown(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 from, float radius, ST_SphereTraceData* const pSphereTraceData)
{
	//ST_AABB testAABB;
	//testAABB.halfExtents = (ST_Vector3){ radius, radius, radius };
	//testAABB.rightTopForwardsTransformedVertex = sphereTraceVector3Add(from, testAABB.halfExtents);
	//testAABB.leftDownBackTransformedVertex = sphereTraceVector3Subtract(from, testAABB.halfExtents);
	ST_IntList triangleIndices = sphereTraceColliderUniformTerrainSampleTriangleIndicesForSphere(pTerrainCollider, from, radius);
	ST_IntListData* pild = triangleIndices.pFirst;
	float maxHeight = -FLT_MAX;
	ST_SphereTraceData dataTest;
	int triangleMin = 0;
	for (int i = 0; i < triangleIndices.count; i++)
	{
		if (sphereTraceColliderTriangleSphereTrace(from, gVector3Down, radius, &pTerrainCollider->triangles[pild->value], &dataTest))
		{
			//rendererDrawSphere(dataTest.rayTraceData.hitPoint, (ST_Vector3) { 0.1f, 0.1f, 0.1f }, gQuaternionIdentity, gVector4ColorRed);
			//sceneDrawTriangleOutline(&pTerrainCollider->triangles[pild->value], gVector4ColorGreen);
			if (dataTest.sphereCenter.y > maxHeight)
			{
				maxHeight = dataTest.sphereCenter.y;
				*pSphereTraceData = dataTest;
				triangleMin = pild->value;
			}
		}
		//if (sphereTraceColliderTriangleSphereTrace(from, gVector3Down, radius, &pTerrainCollider->triangles[pild->value+1], &dataTest))
		//{
		//	rendererDrawSphere(dataTest.rayTraceData.hitPoint, (ST_Vector3) { 0.1f, 0.1f, 0.1f }, gQuaternionIdentity, gVector4ColorRed);
		//	sceneDrawTriangleOutline(&pTerrainCollider->triangles[pild->value+1], gVector4ColorGreen);
		//	float sphereDist = sphereTraceVector3Length(sphereTraceVector3Subtract(dataTest.sphereCenter, from));
		//	if (dataTest.sphereCenter.y > maxHeight)
		//	{
		//		maxHeight = dataTest.sphereCenter.y;
		//		*pSphereTraceData = dataTest;
		//		triangleMin = pild->value+1;
		//	}
		//}
		pild = pild->pNext;
	}
	if (maxHeight > -FLT_MAX)
	{
		sphereTraceColliderTriangleSphereTrace(from, gVector3Down, radius, &pTerrainCollider->triangles[triangleMin], &dataTest);
		return 1;
	}

	return 0;
}

ST_IntList sphereTraceColliderUniformTerrainSampleTrianglesIndicesForSphereTrace(const ST_UniformTerrainCollider* const terrainCollider, ST_SphereTraceData* const pSphereTraceData)
{
	float minRight, minForward, dx, dz;
	float maxCheckDistRight;
	float maxCheckDistFwd;
	float rightDot1 = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereTraceData->rayTraceData.startPoint, terrainCollider->position), terrainCollider->right);
	float rightDot2 = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereTraceData->sphereCenter, terrainCollider->position), terrainCollider->right);
	float fwdDot1 = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereTraceData->rayTraceData.startPoint, terrainCollider->position), terrainCollider->forward);
	float fwdDot2 = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereTraceData->sphereCenter, terrainCollider->position), terrainCollider->forward);
	ST_Vector3 posStart;
	ST_Vector3 dir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pSphereTraceData->sphereCenter, pSphereTraceData->rayTraceData.startPoint));
	ST_Vector3 sphereRight = sphereTraceVector3Normalize(sphereTraceVector3Cross(gVector3Up, dir));
	ST_Vector3 rightRadius = sphereTraceVector3Scale(sphereRight, pSphereTraceData->radius);
	ST_Vector3 sphereForward = sphereTraceVector3Normalize((ST_Vector3) { dir.x, 0.0f, dir.z });
	ST_Vector3 forwardRadius = sphereTraceVector3Scale(sphereForward, pSphereTraceData->radius);
	float rightRight = fabsf(sphereTraceVector3Dot(rightRadius, terrainCollider->rightPlane.normal));
	float rightForward = fabsf(sphereTraceVector3Dot(rightRadius, terrainCollider->forwardPlane.normal));
	float forwardRight = fabsf(sphereTraceVector3Dot(forwardRadius, terrainCollider->rightPlane.normal));
	float forwardForward = fabsf(sphereTraceVector3Dot(forwardRadius, terrainCollider->forwardPlane.normal));
	float signX = 1.0f;
	float signZ = 1.0f;
	if (rightDot1 <= rightDot2)
	{
		minRight = rightDot1;
		minRight -= rightRight;
		dx = rightDot2 - minRight;
		maxCheckDistRight = dx + rightRight + forwardRight + terrainCollider->cellSize;
	}
	else
	{
		minRight = rightDot2;
		minRight -= rightRight;
		dx = rightDot1 - rightDot2 + rightRight;
		maxCheckDistRight = dx + rightRight + forwardRight + terrainCollider->cellSize;
		//signX = -1.0f;
	}
	if (fwdDot1 <= fwdDot2)
	{
		minForward = fwdDot1;
		minForward -= rightForward;
		dz = fwdDot2 - minForward;
		maxCheckDistFwd = dz + rightForward + forwardForward + terrainCollider->cellSize;
	}
	else
	{
		minForward = fwdDot2;
		minForward -= rightForward;
		dz = fwdDot1 - fwdDot2 + rightForward;
		maxCheckDistFwd = dz + rightForward + forwardForward + terrainCollider->cellSize;
		//signZ = -1.0f;
	}
	float r2 = pSphereTraceData->radius * pSphereTraceData->radius;


	//ST_Vector2 maxCheckPos = { paabb->rightTopForwardsTransformedVertex.x + terrainCollider->cellSize, paabb->rightTopForwardsTransformedVertex.z + terrainCollider->cellSize };
	ST_IntList indices = sphereTraceIntListConstruct();
	float distRight = 0.0f;
	float distFwd = 0.0f;
	//float theta = acosf();
	float cTheta = sphereTraceVector3Dot(dir, sphereForward);
	posStart = sphereTraceVector3AddAndScale(sphereTraceVector3AddAndScale(terrainCollider->position, terrainCollider->right, minRight), terrainCollider->forward, minForward);

	//rendererDrawSphere(posStart, sphereTraceVector3Scale(gVector3One, 0.1f), gQuaternionIdentity, gVector4ColorGreen);
	//ST_Vector3 pos1 = sphereTraceVector3AddAndScale(posStart, terrainCollider->forwardPlane.normal, signZ * maxCheckDistFwd);
	//ST_Vector3 pos3 = sphereTraceVector3AddAndScale(posStart, terrainCollider->rightPlane.normal, signX * maxCheckDistRight);
	//ST_Vector3 pos2 = sphereTraceVector3AddAndScale(sphereTraceVector3AddAndScale(posStart, terrainCollider->rightPlane.normal, signX * maxCheckDistRight), terrainCollider->forwardPlane.normal, signZ * maxCheckDistFwd);
	//rendererDrawSphere(pos1, sphereTraceVector3Scale(gVector3One, 0.1f), gQuaternionIdentity, gVector4ColorRed);
	//rendererDrawSphere(pos2, sphereTraceVector3Scale(gVector3One, 0.1f), gQuaternionIdentity, gVector4ColorBlue);
	//rendererDrawSphere(pos3, sphereTraceVector3Scale(gVector3One, 0.1f), gQuaternionIdentity, gVector4Zero);
	//rendererDrawLineFromTo(posStart, pos1, gVector4ColorRed);
	//rendererDrawLineFromTo(pos1, pos2, gVector4ColorRed);
	//rendererDrawLineFromTo(pos2, pos3, gVector4ColorRed);
	//rendererDrawLineFromTo(pos3, posStart, gVector4ColorRed);
	for (float distFwd = 0.0f; distFwd < maxCheckDistFwd; distFwd += terrainCollider->cellSize)
	{
		for (float distRight = 0.0f; distRight < maxCheckDistRight; distRight += terrainCollider->cellSize)
		{
			ST_Vector3 pos = sphereTraceVector3AddAndScale(sphereTraceVector3AddAndScale(posStart, terrainCollider->rightPlane.normal, signX * distRight), terrainCollider->forwardPlane.normal, signZ * distFwd);
			float dirDist = sphereTraceVector3Dot(dir, sphereTraceVector3Subtract(pos, pSphereTraceData->rayTraceData.startPoint));
			ST_Vector3 centerPos = sphereTraceVector3AddAndScale(pSphereTraceData->rayTraceData.startPoint, dir, fabsf(dirDist));
			float distFromCenterLine = sphereTraceVector3Dot(sphereTraceVector3Subtract(centerPos, pos), sphereRight);
			float height = centerPos.y - pSphereTraceData->radius / cTheta;
			if (fabsf(distFromCenterLine) <= pSphereTraceData->radius + terrainCollider->cellSize)
			{
				int index = sphereTraceColliderUniformTerrainSampleFirstTriangleIndex(terrainCollider, (ST_Vector3) { pos.x, terrainCollider->aabb.leftDownBackTransformedVertex.y, pos.z });
				if (index != -1)
				{
					if (terrainCollider->triangles[index].aabb.rightTopForwardsTransformedVertex.y >= height)
						sphereTraceIntListAddFirst(&indices, index);
					if (terrainCollider->triangles[index + 1].aabb.rightTopForwardsTransformedVertex.y >= height)
						sphereTraceIntListAddFirst(&indices, index + 1);
				}
			}

		}
	}
	return indices;
}

b32 sphereTraceColliderUniformTerrainSphereTraceByStartEndPoint(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 startPoint, ST_Vector3 endPoint, float radius, ST_SphereTraceData* const pSphereTraceData)
{
	float minDist = FLT_MAX;
	ST_SphereTraceData stdDummy;
	stdDummy.rayTraceData.startPoint = startPoint;
	stdDummy.sphereCenter = endPoint;
	stdDummy.radius = radius;
	ST_Vector3 dir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(endPoint, startPoint));
	ST_SphereTerrainTriangleContactInfo sttci;
	if (sphereTraceColliderUniformTerrainImposedSphereFindMaxPenetratingTriangle(pTerrainCollider, startPoint, radius, &sttci))
	{
		*pSphereTraceData = sttci.downSphereTraceData;
		return 1;
	}
	//if (sphereTraceVector3Equal(dir, gVector3Down))
	//{
	//	return sphereTraceColliderUniformTerrainSphereTraceDown(pTerrainCollider, stdDummy.rayTraceData.startPoint, stdDummy.radius, pSphereTraceData);
	//}
	ST_IntList il = sphereTraceColliderUniformTerrainSampleTrianglesIndicesForSphereTrace(pTerrainCollider, &stdDummy);
	if (il.count == 0)
		return 0;
	ST_IntListData* pild = il.pFirst;

	for (int i = 0; i < il.count; i++)
	{
		if (sphereTraceColliderTriangleSphereTrace(startPoint, dir, radius, &pTerrainCollider->triangles[pild->value], &stdDummy))
		{
			float sphereLength = sphereTraceVector3Distance(stdDummy.sphereCenter, startPoint);
			if (sphereLength < minDist)
			{
				minDist = sphereLength;
				*pSphereTraceData = stdDummy;
			}
		}
		pild = pild->pNext;
	}
	sphereTraceIntListFree(&il);
	if (minDist != FLT_MAX)
		return 1;
	else
		return 0;
}


b32 sphereTraceColliderUniformTerrainSphereTrace(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 from, ST_Vector3 dir, float radius, ST_SphereTraceData* const pSphereTraceData)
{
	dir = sphereTraceVector3Normalize(dir);
	if (dir.x == 0.0f && dir.z == 0.0f)
	{
		return sphereTraceColliderUniformTerrainSphereTraceDown(pTerrainCollider, from, radius, pSphereTraceData);
	}
	ST_RayTraceData rtcPlane;
	b32 doesIntersectWithTerrainBoundingBox = 0;
	b32 xIncrementing = 0;
	b32 yIncrementing = 0;
	b32 zIncrementing = 0;
	float magX = sphereTraceVector3Dot(dir, pTerrainCollider->rightPlane.normal);
	float magY = sphereTraceVector3Dot(dir, pTerrainCollider->topPlane.normal);
	float magZ = sphereTraceVector3Dot(dir, pTerrainCollider->forwardPlane.normal);
	ST_Vector3 intersectingDirection;
	ST_Vector3 intersection = from;
	ST_Vector3 dp = sphereTraceVector3Subtract(from, pTerrainCollider->position);
	float dpx = intersection.x - pTerrainCollider->position.x;
	float dpz = intersection.z - pTerrainCollider->position.z;
	float xDist = (dpx * pTerrainCollider->right.x + dpz * pTerrainCollider->right.z);
	float zDist = (dpx * pTerrainCollider->forward.x + dpz * pTerrainCollider->forward.z);
	float height = intersection.y;
	if (xDist <= pTerrainCollider->xSize && zDist <= pTerrainCollider->zSize && height <= pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y && xDist >= 0.0f && zDist >= 0.0f && height >= pTerrainCollider->aabb.leftDownBackTransformedVertex.y)
	{
		doesIntersectWithTerrainBoundingBox = 1;
		if (magX > 0.0f)
			xIncrementing = 1;
		if (magY > 0.0f)
			yIncrementing = 1;
		if (magZ > 0.0f)
			zIncrementing = 1;
	}
	else
	{
		if (magX <= 0.0f)
		{
			if (sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->rightPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		else
		{
			xIncrementing = 1;
			if (sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->leftPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		if (magY <= 0.0f)
		{
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->topPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		else
		{
			yIncrementing = 1;
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->bottomPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		if (magZ < 0.0f)
		{
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->forwardPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		else
		{
			zIncrementing = 1;
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->backPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
	}
	if (doesIntersectWithTerrainBoundingBox)
	{
		dpx = intersection.x - pTerrainCollider->position.x;
		dpz = intersection.z - pTerrainCollider->position.z;
		xDist = (dpx * pTerrainCollider->right.x + dpz * pTerrainCollider->right.z);
		zDist = (dpx * pTerrainCollider->forward.x + dpz * pTerrainCollider->forward.z);
		height = intersection.y;
		float tMin = FLT_MAX;
		b32 xChanged = 0;
		b32 zChanged = 0;
		ST_AABB heightBox;
		int xInd = (int)(xDist / pTerrainCollider->cellSize);
		int zInd = (int)(zDist / pTerrainCollider->cellSize);
		while (xDist <= pTerrainCollider->xSize && zDist <= pTerrainCollider->zSize && height <= pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y && xDist >= 0.0f && zDist >= 0.0f && height >= pTerrainCollider->aabb.leftDownBackTransformedVertex.y)
		{


			b32 incomingFromX = 1;
			if (xIncrementing)
			{
				if (xChanged)
					xInd++;
				else
					xInd = (int)(xDist / pTerrainCollider->cellSize) + 1;
				float tentativeX = (xInd * pTerrainCollider->cellSize - xDist) / magX;
				if (tentativeX < tMin && tentativeX > 0.0f)
				{
					tMin = tentativeX;
					intersectingDirection = pTerrainCollider->rightPlane.normal;
				}
			}
			else
			{
				if (xChanged)
					xInd--;
				else
					xInd = (int)(xDist / pTerrainCollider->cellSize);
				float tentativeX = (xInd * pTerrainCollider->cellSize - xDist) / magX;
				if (tentativeX < tMin && tentativeX > 0.0f)
				{

					tMin = tentativeX;
					intersectingDirection = pTerrainCollider->leftPlane.normal;
				}
			}

			if (zIncrementing)
			{
				if (zChanged)
					zInd++;
				else
					zInd = (int)(zDist / pTerrainCollider->cellSize) + 1;
				float tentativeT = (zInd * pTerrainCollider->cellSize - zDist) / magZ;
				if (tentativeT < tMin && tentativeT > 0.0f)
				{

					tMin = tentativeT;
					intersectingDirection = pTerrainCollider->forwardPlane.normal;
					incomingFromX = 0;
				}
			}
			else
			{
				if (zChanged)
					zInd--;
				else
					zInd = (int)(zDist / pTerrainCollider->cellSize);
				float tentativeT = (zInd * pTerrainCollider->cellSize - zDist) / magZ;
				if (tentativeT < tMin && tentativeT > 0.0f)
				{

					tMin = tentativeT;
					intersectingDirection = pTerrainCollider->backPlane.normal;
					incomingFromX = 0;
				}
			}
			ST_Vector3 nextIntersection = sphereTraceVector3AddAndScale(intersection, dir, tMin);
			//need to adjust the intersection so we dont get stuck with floating point error
			if (incomingFromX)
			{
				xChanged = 1;
				xDist = xInd * pTerrainCollider->cellSize;
				dpx = nextIntersection.x - pTerrainCollider->position.x;
				dpz = nextIntersection.z - pTerrainCollider->position.z;
				zDist = (dpx * pTerrainCollider->forward.x + dpz * pTerrainCollider->forward.z);
			}
			else
			{
				zChanged = 1;
				dpx = nextIntersection.x - pTerrainCollider->position.x;
				dpz = nextIntersection.z - pTerrainCollider->position.z;
				zDist = zInd * pTerrainCollider->cellSize;
				xDist = (dpx * pTerrainCollider->right.x + dpz * pTerrainCollider->right.z);
			}
			float nextHeight = nextIntersection.y;
			if (yIncrementing)
			{
				heightBox.rightTopForwardsTransformedVertex.y = nextHeight;
				heightBox.leftDownBackTransformedVertex.y = height;
			}
			else
			{
				heightBox.rightTopForwardsTransformedVertex.y = height;
				heightBox.leftDownBackTransformedVertex.y = nextHeight;
			}
			//if (sphereTraceColliderUniformTerrainSphereTraceByStartEndPoint(pTerrainCollider, intersection, nextIntersection, radius, pSphereTraceData))
			{
				float minDist = FLT_MAX;
				ST_SphereTraceData stdDummy;
				stdDummy.rayTraceData.startPoint = intersection;
				stdDummy.sphereCenter = nextIntersection;
				stdDummy.radius = radius;
				//ST_Vector3 dir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(endPoint, startPoint));
				ST_IntList il = sphereTraceColliderUniformTerrainSampleTrianglesIndicesForSphereTrace(pTerrainCollider, &stdDummy);
				//if (il.count == 0)
				//	return 0;
				ST_IntListData* pild = il.pFirst;

				for (int i = 0; i < il.count; i++)
				{
					//sceneDrawTriangleOutline(&pTerrainCollider->triangles[pild->value], gVector4ColorGreen);
					//sceneDrawAABB(&pTerrainCollider->triangles[pild->value].aabb, gVector4ColorRed);
					if (sphereTraceColliderTriangleSphereTrace(from, dir, radius, &pTerrainCollider->triangles[pild->value], &stdDummy))
					{
						float sphereLength = sphereTraceVector3Distance(stdDummy.sphereCenter, from);
						if (sphereLength < minDist)
						{
							minDist = sphereLength;
							*pSphereTraceData = stdDummy;
						}
					}
					pild = pild->pNext;
				}
				sphereTraceIntListFree(&il);
				if (minDist != FLT_MAX)
				{
					int triIndex;
					ST_TriangleCollider* ptc;
					stdDummy.rayTraceData.startPoint = pSphereTraceData->sphereCenter;
					stdDummy.sphereCenter = sphereTraceVector3AddAndScale(stdDummy.rayTraceData.startPoint, dir, radius);
					il = sphereTraceColliderUniformTerrainSampleTrianglesIndicesForSphereTrace(pTerrainCollider, &stdDummy);
					ST_IntListData* pild = il.pFirst;
					for (int i = 0; i < il.count; i++)
					{
						//sceneDrawTriangleOutline(&pTerrainCollider->triangles[pild->value], gVector4ColorGreen);
						//sceneDrawAABB(&pTerrainCollider->triangles[pild->value].aabb, gVector4ColorRed);
						if (sphereTraceColliderTriangleSphereTrace(from, dir, radius, &pTerrainCollider->triangles[pild->value], &stdDummy))
						{
							float sphereLength = sphereTraceVector3Distance(stdDummy.sphereCenter, from);
							if (sphereLength < minDist)
							{
								minDist = sphereLength;
								*pSphereTraceData = stdDummy;
								triIndex = pild->value;
								ptc = &pTerrainCollider->triangles[triIndex];
							}
						}
						pild = pild->pNext;
					}
					//sphereTraceColliderTriangleSphereTrace(from, dir, radius, ptc, &stdDummy);
					ST_Vector3 adjustedDir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pSphereTraceData->sphereCenter, pSphereTraceData->rayTraceData.startPoint));
					printf("%i\n", sphereTraceVector3Equal(dir, adjustedDir));
					sphereTraceIntListFree(&il);
					return 1;
				}
			}
			//ST_Vector3 samplePos = sphereTraceVector3Average(intersection, nextIntersection);
			//int triIndex = sphereTraceColliderUniformTerrainSampleFirstTriangleIndex(pTerrainCollider, samplePos);
			//if (triIndex != -1)
			//{
			//	ST_TriangleCollider* pt1 = &pTerrainCollider->triangles[triIndex];
			//	ST_TriangleCollider* pt2 = &pTerrainCollider->triangles[triIndex + 1];

			//	if (sphereTraceColliderAABBIntersectAABBVertically(&pt1->aabb, &heightBox))
			//	{
			//		if (sphereTraceColliderTriangleRayTrace(from, dir, pt1, pSphereTraceData))
			//		{
			//			return 1;
			//		}
			//	}
			//	if (sphereTraceColliderAABBIntersectAABBVertically(&pt2->aabb, &heightBox))
			//	{
			//		if (sphereTraceColliderTriangleRayTrace(from, dir, pt2, pSphereTraceData))
			//		{
			//			return 1;
			//		}
			//	}
			//}

			intersection = nextIntersection;
			height = intersection.y;
			tMin = FLT_MAX;


		}
	}
	return 0;
}