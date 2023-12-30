#pragma once
#include "SphereTraceCollider.h"


ST_BoxCollider sphereTraceColliderBoxConstruct(ST_Vector3 halfExtents);

void sphereTraceColliderBoxSetAABB(ST_BoxCollider* const pBoxCollider);