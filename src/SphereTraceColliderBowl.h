#pragma once

#include "SphereTraceCollider.h"
#include "SphereTraceGlobals.h"

ST_BowlCollider sphereTraceColliderBowlConstruct(ST_Vector3 position, float radius, ST_Direction normal);

b32 sphereTraceColliderBowlImposedSphereCollisionTest(ST_BowlCollider* const pBowlCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereContact* pContact);

b32 sphereTraceColliderBowlSphereCollisionTest(ST_BowlCollider* const pBowlCollider, ST_SphereCollider* const pSphere, ST_SphereContact* pContact);

void sphereTraceColliderBowlsSetAABB(ST_BowlCollider* const pBowlCollider);