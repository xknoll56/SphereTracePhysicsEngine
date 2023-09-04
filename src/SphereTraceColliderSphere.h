#pragma once
#include "SphereTraceCollider.h"

void sphereTraceColliderSphereAABBSetTransformedVertices(ST_SphereCollider* const pSphereCollider);
//
b32 sphereTraceColliderAABBIntersectImposedSphere(const ST_AABB* const aabb, ST_Vector3 imposedPosition, float imposedRadius);
//
b32 sphereTraceColliderAABBIntersectSphere(const ST_AABB* const aabb, const ST_SphereCollider* const pSphereCollider);
//
b32 sphereTraceColliderInfinitePlaneImposedSphereCollisionTest(ST_Vector3 imposedPosition, float imposedRadius, ST_Direction planeNormal, ST_Vector3 pointOnPlane, ST_Contact* const pContact);
//
ST_SphereCollider sphereTraceColliderSphereConstruct(float radius, ST_RigidBody* const pRigidBody);
//
ST_SphereCollider sphereTraceColliderSphereConstructWithPosition(float radius, ST_RigidBody* const pRigidBody, ST_Vector3 position);
//
b32 sphereTraceColliderSphereRayTrace(ST_Vector3 start, ST_Direction dir, const ST_SphereCollider* const pSphere, ST_RayTraceData* const pData);
//
b32 sphereTraceColliderImposedSphereRayTrace(ST_Vector3 start, ST_Direction dir, ST_Vector3 imposedPosition, float imposedRadius, ST_RayTraceData* const pData);
//
b32 sphereTraceColliderSphereSphereTrace(ST_Vector3 start, ST_Direction dir, float radius, const ST_SphereCollider* const pSphere, ST_SphereTraceData* const pData);
//
b32 sphereTraceColliderPointImposedSphereCollisionTest(ST_Vector3 point, ST_Vector3 imposedPosition, float imposedRadius, ST_Contact* const pContact);
//
b32 sphereTraceColliderEdgeImposedSphereCollisionTest(const ST_Edge* const pEdge, ST_Vector3 imposedPosition, float imposedRadius, ST_Contact* const pContact);
//
b32 sphereTraceColliderPlaneSphereCollisionTest(ST_PlaneCollider* const pPlaneCollider, ST_SphereCollider* const pSphereCollider, ST_SpherePlaneContactInfo* const contactInfo);
//
b32 sphereTraceColliderTriangleSphereCollisionTest(ST_TriangleCollider* const pTriangleCollider, ST_SphereCollider* const pSphereCollider, ST_SphereTriangleContactInfo* const contactInfo);
//
b32 sphereTraceColliderPlaneImposedSphereCollisionTest(ST_PlaneCollider* const pPlaneCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SpherePlaneContactInfo* const contactInfo);
//
b32 sphereTraceColliderTriangleImposedSphereCollisionTest(ST_TriangleCollider* const pTriangleCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereTriangleContactInfo* const contactInfo);
//
b32 sphereTraceColliderSphereSphereCollisionTest(ST_SphereCollider* const pSphereColliderA, ST_SphereCollider* const pSphereColliderB, ST_SphereSphereContactInfo* const pContactInfo);
