#pragma once

#include "SphereTraceCollider.h"
//
ST_PlaneCollider sphereTraceColliderPlaneConstruct(ST_Vector3 normal, float angle, float xHalfExtent, float zHalfExtent, ST_Vector3 position);
//
ST_PlaneCollider sphereTraceColliderPlaneConstructWithRotationMatrix(ST_Matrix4 rotMat, float xHalfExtent, float zHalfExtent, ST_Vector3 position);
//
void sphereTraceColliderPlaneSetTransformWithRotationMatrix(ST_PlaneCollider* const pPlaneCollider, ST_Matrix4 rotMat, float xHalfExtent, float zHalfExtent, ST_Vector3 position);
//
void sphereTraceColliderPlaneSetTransformedVerticesAndEdges(ST_PlaneCollider* const pPlaneCollider);
//
ST_PlaneEdgeDirection sphereTraceColliderPlaneGetClosestTransformedEdgeToPoint(const ST_PlaneCollider* const pPlaneCollider, ST_Vector3 point);
//
ST_PlaneEdgeDirection sphereTraceColliderPlaneGetClosestTransformedEdgeToSphereTrace(const ST_PlaneCollider* const pPlaneCollider, ST_Vector3 point, ST_Vector3 dir, float radius, ST_Vector3* closestPoint);
//
ST_PlaneVertexDirection sphereTraceColliderPlaneGetClosestTransformedVertexToPoint(const ST_PlaneCollider* const pPlaneCollider, ST_Vector3 point);
//
void sphereTraceColliderPlaneTranslate(ST_PlaneCollider* const pPlaneCollider, ST_Vector3 translation);
//
void sphereTraceColliderPlaneRotateAround(ST_PlaneCollider* const pPlaneCollider, ST_Vector3 point, ST_Quaternion rotation);
//
void sphereTraceColliderPlaneSetPosition(ST_PlaneCollider* const pPlaneCollider, ST_Vector3 position);
//
void sphereTraceColliderPlaneSetRotation(ST_PlaneCollider* const pPlaneCollider, ST_Quaternion rotation);
//
void sphereTraceColliderPlaneSetRotationWithMatrix(ST_PlaneCollider* const pPlaneCollider, ST_Matrix4 rotMat);
//
void sphereTraceColliderPlaneSetAABB(ST_PlaneCollider* const pPlaneCollider);
//
void sphereTraceColliderPlaneSetAABBExtents(ST_PlaneCollider* const pPlaneCollider);
//
void sphereTraceColliderPlaneAABBSetTransformedVertices(ST_PlaneCollider* const pPlaneCollider);
//
void sphereTraceColliderInfinitePlaneRayTrace(ST_Vector3 from, ST_Direction dir, ST_Direction planeNormal, ST_Vector3 pointOnPlane, ST_RayTraceData* const pRaycastData);
//
b32 sphereTraceColliderInfinitePlaneEdgeTrace(ST_Edge* pEdge, ST_Direction dir, ST_Direction planeNormal, ST_Vector3 pointOnPlane, ST_EdgeTraceData* const pEdgeTraceData);
//
void sphereTraceColliderInfiniteZPlaneRayTrace(ST_Vector3 from, ST_Direction dir, ST_Vector3 pointOnPlane, ST_RayTraceData* const pRaycastData);
void sphereTraceColliderInfiniteYPlaneRayTrace(ST_Vector3 from, ST_Direction dir, ST_Vector3 pointOnPlane, ST_RayTraceData* const pRaycastData);
void sphereTraceColliderInfiniteXPlaneRayTrace(ST_Vector3 from, ST_Direction dir, ST_Vector3 pointOnPlane, ST_RayTraceData* const pRaycastData);

b32 sphereTraceColliderInfiniteZPlaneSphereTrace(ST_Vector3 from, ST_Direction dir, float radius, ST_Vector3 pointOnPlane, ST_SphereTraceData* const pData);
b32 sphereTraceColliderInfiniteYPlaneSphereTrace(ST_Vector3 from, ST_Direction dir, float radius, ST_Vector3 pointOnPlane, ST_SphereTraceData* const pData);
b32 sphereTraceColliderInfiniteXPlaneSphereTrace(ST_Vector3 from, ST_Direction dir, float radius, ST_Vector3 pointOnPlane, ST_SphereTraceData* const pData);
//
b32 sphereTraceColliderPlaneRayTrace(ST_Vector3 from, ST_Direction dir, const ST_PlaneCollider* const pPlaneCollider, ST_RayTraceData* const pRaycastData);
//
b32 sphereTraceColliderPlaneEdgeTrace(ST_Edge* const pEdge, ST_Direction dir, const ST_PlaneCollider* const pPlaneCollider, ST_EdgeTraceData* const pEdgeTraceData);
//
b32 sphereTraceColliderPlaneIsProjectedPointContained(ST_Vector3 projectedPoint, const ST_PlaneCollider* const pPlaneCollider);
//
b32 sphereTraceColliderInfinitePlaneSphereTrace(ST_Vector3 from, ST_Direction dir, float radius, ST_Vector3 pointOnPlane, ST_Direction planeNormal, ST_SphereTraceData* const pSphereTraceData);
//
b32 sphereTraceColliderPlaneSphereTrace(ST_Vector3 from, ST_Direction dir, float radius, ST_PlaneCollider* const pPlaneCollider, ST_SphereTraceData* const pSphereCastData);
//
b32 sphereTraceColliderPlaneSphereTraceOut(ST_Vector3 spherePos, float sphereRadius, ST_Direction clipoutDir, ST_PlaneCollider* const pPlaneCollider, ST_SphereTraceData* const pSphereCastData);
