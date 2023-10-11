#pragma once

#include "SphereTraceCollider.h"


ST_PipeCollider sphereTraceColliderPipeConstruct(ST_Vector3 position, float radius, float length, ST_Direction up);

b32 sphereTraceColliderPipeImposedSphereCollisionTest(ST_PipeCollider* const pPipeCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_Contact* pContact);

b32 sphereTraceColliderPipeSphereCollisionTest(ST_PipeCollider* const pPipeCollider, ST_SphereCollider* const pSphere, ST_Contact* pContact);

void sphereTraceColliderPipeSetAABB(ST_PipeCollider* const pPipeCollider);