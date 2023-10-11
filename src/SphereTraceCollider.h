#pragma once
#include "SphereTraceMath.h"
#include "SphereTraceLists.h"
#include "SphereTraceRigidBody.h"


#define COLLIDER_TOLERANCE 1e-6

typedef enum ST_ColliderType
{
	COLLIDER_SPHERE = 0,
	COLLIDER_PLANE = 1,
	COLLIDER_TERRAIN = 2,
	COLLIDER_TRIANGLE = 3,
	COLLIDER_BOWL = 4,
	COLLIDER_PIPE = 5
} ST_ColliderType;


//for simplicity dynamic spheres will be considered
//a face.
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
	COLLISION_POINT_POINT = 8,
	COLLISION_FACE_INWARD_SPHEREICAL = 9,
	COLLISION_FACE_OUTWARD_SPHEREICAL = 10,
	COLLISION_FACE_INWARD_CIRCULAR = 11,
	COLLISION_FACE_OUTWARD_CIRCULAR = 12
} ST_CollisionType;

typedef struct ST_Contact
{
	float penetrationDistance;
	ST_Direction normal;
	ST_Vector3 point;
	enum ST_CollisionType collisionType;
	enum ST_ColliderType contactAType;
	void* contactA;
	enum ST_ColliderType contactBType;
	void* contactB;
	float radiusOfCurvature;
	ST_Direction bitangent;
} ST_Contact;


typedef int ST_ColliderIndex;

typedef struct ST_Subscriber
{
	void (*callback)();
	struct ST_Subscriber* pNext;

} ST_Subscriber;

typedef struct ST_SubscriberList
{
	int count;
	ST_Subscriber pFirst;
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
	ST_Direction dir;
	float dist;
} ST_Edge;

typedef struct ST_Ring
{
	ST_Vector3 centroid;
	ST_Direction normal;
	ST_Direction right;
	ST_Direction forward;
	float radius;
} ST_Ring;

typedef struct ST_Collider
{
	ST_ColliderType colliderType;
	ST_IndexList bucketIndices;
	ST_Index colliderIndex;
} ST_Collider;

typedef struct ST_PlaneCollider
{
	ST_Collider collider;
	//ST_ColliderType colliderType; 
	ST_Direction normal;
	ST_Direction right;
	ST_Direction forward;
	float xHalfExtent;
	float zHalfExtent;
	ST_Vector3 position;
	ST_Quaternion rotation;
	ST_Vector3 transformedVertices[4];
	ST_Edge transformedEdges[4];
	ST_AABB aabb;
	//ST_IndexList bucketIndices;
	//ST_Index planeColliderIndex;
} ST_PlaneCollider;

typedef struct ST_TriangleCollider
{
	ST_Collider collider;
	//ST_ColliderType colliderType;
	ST_Vector3 transformedVertices[3];
	ST_Edge transformedEdges[3];
	ST_Direction vertexDirs[3];
	ST_Direction edgeOrthogonalDirs[3];
	float vertexDists[3];
	ST_Direction normal;
	ST_Vector3 centroid;
	ST_AABB aabb;

	//members needed for terrain optimizations
	ST_Index terrainIndex;
	ST_Vector2Integer terrainCoords;
	ST_Index lowestVertIndex;
	ST_Index highestVertIndex;
	float circularRadius;

	b32 ignoreCollisions;
} ST_TriangleCollider;

typedef struct ST_SphereCollider
{
	ST_Collider collider;
	//ST_ColliderType colliderType;
	b32 restingContact;
	float radius;
	ST_RigidBody rigidBody;
	ST_AABB aabb;
	//ST_IndexList bucketIndices;
	//ST_Index sphereColliderIndex;
	b32 ignoreCollisions;
} ST_SphereCollider;

typedef struct ST_BowlCollider
{
	ST_Collider collider;
	ST_Vector3 position;
	float radius;
	ST_Direction normal;
	ST_AABB aabb;
} ST_BowlCollider;

typedef struct ST_PipeCollider
{
	ST_Collider collider;
	ST_Vector3 position;
	float radius;
	float length;
	ST_Direction up;
	ST_Direction right;
	ST_Direction forward;
	ST_AABB aabb;
} ST_PipeCollider;

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
	ST_Contact contact;
	ST_Vector3 startPoint;
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


//typedef struct ST_SpherePlaneContactInfo
//{
//	float penetrationDistance;
//	ST_Direction normal;
//	ST_Vector3 point;
//	ST_SphereCollider* pSphereCollider;
//	ST_PlaneCollider* pPlaneCollider;
//	ST_CollisionType collisionType;
//} ST_SpherePlaneContactInfo;

//typedef struct ST_SphereTriangleContactInfo
//{
//	float penetrationDistance;
//	ST_Direction normal;
//	ST_Vector3 point;
//	ST_SphereCollider* pSphereCollider;
//	ST_TriangleCollider* pTriangleCollider;
//	ST_CollisionType collisionType;
//} ST_SphereTriangleContactInfo;
//
//typedef struct ST_SphereTerrainTriangleContactInfo
//{
//	ST_SphereTriangleContactInfo sphereTriangleContactInfo;
//	ST_SphereTraceData downSphereTraceData;
//} ST_SphereTerrainTriangleContactInfo;
//
//typedef struct ST_SphereSphereContactInfo
//{
//	float penetrationDistance;
//	ST_Direction normal;
//	ST_Vector3 point;
//	ST_SphereCollider* pA;
//	ST_SphereCollider* pB;
//} ST_SphereSphereContactInfo;

ST_Edge sphereTraceEdgeConstruct(ST_Vector3 p1, ST_Vector3 p2);

ST_Ring sphereTraceRingConstruct(ST_Vector3 centroid, ST_Direction normal, float radius);



b32 sphereTraceColliderAABBIsPointInside(const ST_AABB* const aabb, ST_Vector3 point);

void sphereTraceColliderAABBSetHalfExtents(ST_AABB* const aabb);

void sphereTraceColliderAABBResizeAABBToContainAnotherAABB(ST_AABB* const aabbToResize, const ST_AABB* const aabbToContain);

b32 sphereTraceColliderAABBIntersectAABB(const ST_AABB* const aabb1, const ST_AABB* const aabb2);

b32 sphereTraceColliderAABBIntersectAABBHorizontally(const ST_AABB* const aabb1, const ST_AABB* const aabb2);

b32 sphereTraceColliderAABBIntersectAABBVertically(const ST_AABB* const aabb1, const ST_AABB* const aabb2);

ST_Vector3 sphereTraceColliderAABBMidPoint(const ST_AABB* const aabb);

ST_Vector3 sphereTraceColliderAABBGetRightBottomBackExtent(const ST_AABB* const paabb);

ST_Vector3 sphereTraceColliderAABBGetLeftTopBackExtent(const ST_AABB* const paabb);

ST_Vector3 sphereTraceColliderAABBGetLeftBottomForwardExtent(const ST_AABB* const paabb);

ST_Vector3 sphereTraceColliderAABBGetRightTopBackExtent(const ST_AABB* const paabb);

ST_Vector3 sphereTraceColliderAABBGetRightBottomForwardExtent(const ST_AABB* const paabb);

ST_Vector3 sphereTraceColliderAABBGetLeftTopForwardExtent(const ST_AABB* const paabb);

void sphereTraceColliderResizeAABBWithSpherecast(const ST_SphereTraceData* const pSphereCastData, ST_AABB* const aabb);


b32 sphereTraceColliderPointSphereTrace(ST_Vector3 from, ST_Direction dir, float radius, ST_Vector3 point, ST_SphereTraceData* const pSphereTraceData);

b32 sphereTraceColliderEdgeSphereTrace(ST_Vector3 from, ST_Direction dir, float radius, ST_Edge* const pEdge, ST_SphereTraceData* const pSphereTraceData);

b32 sphereTraceColliderRingSphereTrace(ST_Vector3 from, ST_Direction dir, float radius, ST_Ring* const pRing, ST_SphereTraceData* const pSphereTraceData);

ST_SphereCollider* sphereTraceColliderSphereGetFromContact(const ST_Contact* const pContact);
ST_PlaneCollider* sphereTraceColliderPlaneGetFromContact(const ST_Contact* const pContact);
ST_TriangleCollider* sphereTraceColliderTriangleGetFromContact(const ST_Contact* const pContact);
//ST_UniformTerrainCollider* sphereTraceColliderTerrainGetFromContact(const ST_Contact* const pContact);

typedef struct ST_UniformTerrainCollider
{
	ST_Collider collider;
	//ST_ColliderType colliderType;	
	ST_Index triangleCount;
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
	//ST_IndexList bucketIndices;
	ST_Vector3 halfExtents;

	//needed for quick raytrace
	//replace with obb when implemented
	ST_PlaneCollider rightPlane;
	ST_PlaneCollider topPlane;
	ST_PlaneCollider forwardPlane;
	ST_PlaneCollider leftPlane;
	ST_PlaneCollider bottomPlane;
	ST_PlaneCollider backPlane;
	//ST_Index uniformTerrainColliderIndex;
} ST_UniformTerrainCollider;

typedef struct ST_UniformTerrainSpherePrecomputedSampler
{
	ST_UniformTerrainCollider* pUniformTerrainCollider;
	float sphereRadius;
	int** sampledIndexes;
	ST_Index subIncrements;
	float subCellSize;

}ST_UniformTerrainSpherePrecomputedSampler;

#include "SphereTraceColliderPlane.h"
#include "SphereTraceColliderSphere.h"
#include "SphereTraceColliderTerrain.h"
#include "SphereTraceColliderTriangle.h"