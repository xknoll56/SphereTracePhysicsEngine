#pragma once
#include "SphereTraceMath.h"
#include "SphereTraceLists.h"
#include "SphereTraceRigidBody.h"

#define COLLIDER_TOLERANCE 1e-6

typedef enum ST_ColliderType
{
	COLLIDER_SPHERE = 0,
	COLLIDER_PLANE = 1,
	COLLIDER_TERRAIN = 2
} ST_ColliderType;

typedef int ST_ColliderIndex;

typedef struct ST_SubscriberEntry
{
	void (*callback)();
	struct ST_CallbackEntry* pNext;

} ST_SubscriberEntry;

typedef struct ST_SubscriberList
{
	int count;

} ST_SubscriberList;

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

typedef enum ST_CollisionType
{
	COLLISION_FACE_FACE = 0,
	COLLISION_FACE_EDGE = 1,
	COLLISION_FACE_POINT = 2,
	COLLISION_EDGE_FACE = 3,
	COLLISION_EDGE_EDGE = 4,
	COLLISION_EDGE_POINT = 5,
	COLLISION_POINT_FACE = 6,
	COLLISION_POINT_EDGE = 7,
	COLLISION_POINT_POINT = 8
} ST_CollisionType;


typedef struct ST_SpherePlaneContactInfo
{
	float penetrationDistance;
	ST_Vector3 normal;
	ST_Vector3 point;
	ST_SphereCollider* pSphereCollider;
	ST_PlaneCollider* pPlaneCollider;
	ST_CollisionType collisionType;
} ST_SpherePlaneContactInfo;

typedef struct ST_SphereTriangleContactInfo
{
	float penetrationDistance;
	ST_Vector3 normal;
	ST_Vector3 point;
	ST_SphereCollider* pSphereCollider;
	ST_TriangleCollider* pTriangleCollider;
	ST_CollisionType collisionType;
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

ST_Edge sphereTraceEdgeConstruct(ST_Vector3 p1, ST_Vector3 p2);

void sphereTraceColliderPlaneSetTransformedVerticesAndEdges(ST_PlaneCollider* const pPlaneCollider);


ST_PlaneEdgeDirection sphereTraceColliderPlaneGetClosestTransformedEdgeToPoint(const ST_PlaneCollider* const pPlaneCollider, ST_Vector3 point);

ST_PlaneEdgeDirection sphereTraceColliderPlaneGetClosestTransformedEdgeToSphereTrace(const ST_PlaneCollider* const pPlaneCollider, ST_Vector3 point, ST_Vector3 dir, float radius, ST_Vector3* closestPoint);

ST_PlaneVertexDirection sphereTraceColliderPlaneGetClosestTransformedVertexToPoint(const ST_PlaneCollider* const pPlaneCollider, ST_Vector3 point);

int sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToPoint(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point);


int sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToLine(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point, ST_Vector3 dir);

//TODO FIX THIS AWFUL CODE!
int sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToSphereTrace(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point, float radius, ST_Vector3 dir, ST_Vector3* const closestPoint);

int sphereTraceColliderTriangleGetClosestTransformedVertexIndexToPoint(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point);


void sphereTraceColliderPlaneSetAABBExtents(ST_PlaneCollider* const pPlaneCollider);

void sphereTraceColliderPlaneAABBSetTransformedVertices(ST_PlaneCollider* const pPlaneCollider);

void sphereTraceColliderSphereAABBSetTransformedVertices(ST_SphereCollider* const pSphereCollider);

void sphereTraceColliderTriangleSetAABB(ST_TriangleCollider* const pTriangleCollider);

b32 sphereTraceColliderAABBIsPointInside(const ST_AABB* const aabb, ST_Vector3 point);

void sphereTraceColliderAABBSetHalfExtents(ST_AABB* const aabb);

void sphereTraceColliderAABBResizeAABBToContainAnotherAABB(ST_AABB* const aabbToResize, const ST_AABB* const aabbToContain);

b32 sphereTraceColliderAABBIntersectAABB(const ST_AABB* const aabb1, const ST_AABB* const aabb2);

b32 sphereTraceColliderAABBIntersectAABBHorizontally(const ST_AABB* const aabb1, const ST_AABB* const aabb2);

b32 sphereTraceColliderAABBIntersectAABBVertically(const ST_AABB* const aabb1, const ST_AABB* const aabb2);

ST_Vector3 sphereTraceColliderAABBMidPoint(const ST_AABB* const aabb);

//todo, check the closest edge instead
b32 sphereTraceColliderAABBIntersectImposedSphere(const ST_AABB* const aabb, ST_Vector3 imposedPosition, float imposedRadius);

b32 sphereTraceColliderAABBIntersectSphere(const ST_AABB* const aabb, const ST_SphereCollider* const pSphereCollider);



void sphereTraceColliderResizeAABBWithSpherecast(const ST_SphereTraceData* const pSphereCastData, ST_AABB* const aabb);

ST_PlaneCollider sphereTraceColliderPlaneConstruct(ST_Vector3 normal, float angle, float xHalfExtent, float zHalfExtent, ST_Vector3 position);

ST_PlaneCollider sphereTraceColliderPlaneConstructWithRotationMatrix(ST_Matrix4 rotMat, float xHalfExtent, float zHalfExtent, ST_Vector3 position);


ST_SphereCollider sphereTraceColliderSphereConstruct(float radius, ST_RigidBody* const pRigidBody);

ST_SphereCollider sphereTraceColliderSphereConstructWithPosition(float radius, ST_RigidBody* const pRigidBody, ST_Vector3 position);

void sphereTraceColliderTriangleSetVertexAndEdgeData(ST_TriangleCollider* const pTriangleCollider, ST_Vector3 v1, ST_Vector3 v2, ST_Vector3 v3);

ST_TriangleCollider sphereTraceColliderTriangleConstruct(ST_Vector3 v1, ST_Vector3 v2, ST_Vector3 v3);

void sphereTraceColliderInfinitePlaneRayTrace(ST_Vector3 from, ST_Vector3 dir, ST_Vector3 planeNormal, ST_Vector3 pointOnPlane, ST_RayTraceData* const pRaycastData);

b32 sphereTraceColliderPlaneRayTrace(ST_Vector3 from, ST_Vector3 dir, const ST_PlaneCollider* const pPlaneCollider, ST_RayTraceData* const pRaycastData);

b32 sphereTraceColliderTriangleIsProjectedPointContained(ST_Vector3 projectedPoint, const ST_TriangleCollider* const pTriangleCollider);

b32 sphereTraceColliderTriangleRayTrace(ST_Vector3 from, ST_Vector3 dir, const ST_TriangleCollider* const pTriangleCollider, ST_RayTraceData* const pRaycastData);


b32 sphereTraceColliderSphereRayTrace(ST_Vector3 start, ST_Vector3 dir, const ST_SphereCollider* const pSphere, ST_RayTraceData* const pData);

b32 sphereTraceColliderImposedSphereRayTrace(ST_Vector3 start, ST_Vector3 dir, ST_Vector3 imposedPosition, float imposedRadius, ST_RayTraceData* const pData);

b32 sphereTraceColliderSphereSphereTrace(ST_Vector3 start, ST_Vector3 dir, float radius, const ST_SphereCollider* const pSphere, ST_SphereTraceData* const pData);



b32 sphereTraceColliderPlaneSphereCollisionTest(ST_PlaneCollider* const pPlaneCollider, ST_SphereCollider* const pSphereCollider, ST_SpherePlaneContactInfo* const contactInfo);


b32 sphereTraceColliderTriangleSphereCollisionTest(ST_TriangleCollider* const pTriangleCollider, ST_SphereCollider* const pSphereCollider, ST_SphereTriangleContactInfo* const contactInfo);

b32 sphereTraceColliderPlaneImposedSphereCollisionTest(ST_PlaneCollider* const pPlaneCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SpherePlaneContactInfo* const contactInfo);

b32 sphereTraceColliderTriangleImposedSphereCollisionTest(ST_TriangleCollider* const pTriangleCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereTriangleContactInfo* const contactInfo);

b32 sphereTraceColliderPlaneSphereTrace(ST_Vector3 from, ST_Vector3 dir, float radius, ST_PlaneCollider* const pPlaneCollider, ST_SphereTraceData* const pSphereCastData);


//b32 sphereTraceColliderTriangleSphereTrace(ST_Vector3 from, ST_Vector3 dir, float radius, const ST_TriangleCollider* const pTriangleCollider, ST_SphereTraceData* const pSphereCastData);
b32 sphereTraceColliderTriangleSphereTrace(ST_Vector3 from, ST_Vector3 dir, float radius, ST_TriangleCollider* const pTriangleCollider, ST_SphereTraceData* const pSphereCastData);

b32 sphereTraceColliderSphereSphereCollisionTest(ST_SphereCollider* const pSphereColliderA, ST_SphereCollider* const pSphereColliderB, ST_SphereSphereContactInfo* const pContactInfo);

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

ST_UniformTerrainCollider sphereTraceColliderUniformTerrainConstruct(int xCells, int zCells, float cellSize);

void sphereTraceColliderUniformTerrainSetTransform(ST_UniformTerrainCollider* const pTerrainCollider, float angle, ST_Vector3 position);

void sphereTraceColliderUniformTerrainFillTrianglesWithFunction(ST_UniformTerrainCollider* const terrainCollider, float (*fxz)(float, float));

void sphereTraceColliderUniformTerrainFillTrianglesWithFunctionAndConditionalFunction(ST_UniformTerrainCollider* const terrainCollider, float (*fxz)(float, float), b32(*conditionalFunc)(float (*fxz)(float, float), float, float));

int sphereTraceColliderUniformTerrainSampleFirstTriangleIndex(const ST_UniformTerrainCollider* const terrainCollider, ST_Vector3 samplePosition);

ST_IntList sphereTraceColliderUniformTerrainSampleTriangleIndicesForSphere(const ST_UniformTerrainCollider* const terrainCollider, ST_Vector3 spherePosition, float radius);


b32 sphereTraceColliderUniformTerrainImposedSphereFindMaxPenetratingTriangle(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereTerrainTriangleContactInfo* const pContactInfo);

b32 sphereTraceColliderUniformTerrainSphereFindMaxPenetratingTriangle(const ST_UniformTerrainCollider* const pTerrainCollider, ST_SphereCollider* const pSphereCollider, ST_SphereTerrainTriangleContactInfo* const pContactInfo);




b32 sphereTraceColliderUniformTerrainRayTrace(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 from, ST_Vector3 dir, ST_RayTraceData* const pRayTraceData);

b32 sphereTraceColliderUniformTerrainSphereTraceDown(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 from, float radius, ST_SphereTraceData* const pSphereTraceData);

ST_IntList sphereTraceColliderUniformTerrainSampleTrianglesIndicesForSphereTrace(const ST_UniformTerrainCollider* const terrainCollider, ST_SphereTraceData* const pSphereTraceData);

b32 sphereTraceColliderUniformTerrainSphereTraceByStartEndPoint(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 startPoint, ST_Vector3 endPoint, float radius, ST_SphereTraceData* const pSphereTraceData);


b32 sphereTraceColliderUniformTerrainSphereTrace(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 from, ST_Vector3 dir, float radius, ST_SphereTraceData* const pSphereTraceData);