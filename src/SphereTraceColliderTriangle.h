#include "SphereTraceCollider.h"


int sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToPoint(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point);
//
int sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToLine(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point, ST_Vector3 dir);
//
int sphereTraceColliderTriangleGetClosestTransformedEdgeIndexToSphereTrace(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point, float radius, ST_Vector3 dir, ST_Vector3* const closestPoint);
//
int sphereTraceColliderTriangleGetClosestTransformedVertexIndexToPoint(const ST_TriangleCollider* const pTriangleCollider, ST_Vector3 point);
//
void sphereTraceColliderTriangleSetAABB(ST_TriangleCollider* const pTriangleCollider);
//
void sphereTraceColliderTriangleSetVertexAndEdgeData(ST_TriangleCollider* const pTriangleCollider, ST_Vector3 v1, ST_Vector3 v2, ST_Vector3 v3);
//
ST_TriangleCollider sphereTraceColliderTriangleConstruct(ST_Vector3 v1, ST_Vector3 v2, ST_Vector3 v3);
//
b32 sphereTraceColliderTriangleIsProjectedPointContained(ST_Vector3 projectedPoint, const ST_TriangleCollider* const pTriangleCollider);
//
b32 sphereTraceColliderTriangleRayTrace(ST_Vector3 from, ST_Vector3 dir, const ST_TriangleCollider* const pTriangleCollider, ST_RayTraceData* const pRaycastData);
//
b32 sphereTraceColliderTriangleSphereTrace(ST_Vector3 from, ST_Vector3 dir, float radius, ST_TriangleCollider* const pTriangleCollider, ST_SphereTraceData* const pSphereCastData);
