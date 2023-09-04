#pragma once
#include "SphereTraceCollider.h"

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