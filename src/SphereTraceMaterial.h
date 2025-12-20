#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#include "SphereTraceMath.h"

typedef struct ST_Material
{
	float restitution;
	float staticFriction;
	float kineticFriction;
} ST_Material;


ST_Material sphereTraceMaterialConstruct(float restitution, float staticFriction, float kineticFriction);

#ifdef __cplusplus
}
#endif