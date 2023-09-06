#pragma once

#include "SphereTraceMath.h"
#include "SphereTraceLists.h"
#include "SphereTraceRigidBody.h"
#include "SphereTraceCollider.h"
#include "SphereTraceColliderPlane.h"
#include "SphereTraceColliderTerrain.h"
#include "SphereTraceColliderSphere.h"
#include "SphereTraceColliderTriangle.h"
#include "SphereTraceDynamicArray.h"
#include "SphereTraceSpacialPartition.h"
#include "SphereTraceAI.h"


#define BOUNCE_FORCE_MAX 5000.0f
#define VELOCITY_THRESHOLD 0.5f
#define AUTO_DT_FACTOR 1.0/10.0f
#define SKIN_WIDTH 0.005f


typedef struct ST_SimulationSpace
{
	ST_VectorArrayPointers sphereColliders;
	ST_VectorArrayPointers planeColliders;
	ST_VectorArrayPointers triangleColliders;
	ST_VectorArrayPointers uniformTerrainColliders;
	ST_VectorArrayContactInfo spherePlaneContactInfos;
	ST_VectorArrayContactInfo sphereSphereContactInfos;
	ST_VectorArrayContactInfo sphereTriangleContactInfos;
	ST_VectorArrayContactInfo sphereTerrainTriangleContactInfos;
	float eta;
	ST_Vector3 gravitationalAcceleration;
	float friction;
	ST_SpacialPartitionStaticContainer spacialPartitionContainer;
} ST_SimulationSpace;

ST_SimulationSpace sphereTraceSimulationConstruct(float eta, float friction, ST_Vector3 gravitationalAcceleration);

void sphereTraceSimulationInsertPlaneCollider(ST_SimulationSpace* const pSimulationSpace, ST_PlaneCollider* const pPlaneCollider);

void sphereTraceSimulationInsertSphereCollider(ST_SimulationSpace* const pSimulationSpace, ST_SphereCollider* const pSphereCollider);

void sphereTraceSimulationInsertUniformTerrainCollider(ST_SimulationSpace* const pSimulationSpace, ST_UniformTerrainCollider* const pTerrainCollider);

void sphereTraceSimulationUpdateSphereColliderBucketIndices(ST_SimulationSpace* const pSimulationSpace, ST_SphereCollider* const pSphereCollider);

void sphereTraceSimulationApplyForcesAndTorques(const ST_SimulationSpace* const pSimulationSpace, ST_RigidBody* const pRigidBody, float dt, b32 incrementGravity);

void sphereTraceSimulationStepQuantity(const ST_SimulationSpace* const pSimulationSpace, ST_RigidBody* const pRigidBody, float dt);

ST_Vector3 sphereTraceSimulationImposedStepPosition(const ST_SimulationSpace* const pSimulationSpace, ST_RigidBody* const pRigidBody, float dt);



void sphereTraceSimulationStepQuantities(const ST_SimulationSpace* const pSimulationSpace, float dt);

void sphereTraceSimulationSpherePlaneResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_SpherePlaneContactInfo* const pContactInfo, float dt);

void sphereTraceSimulationSphereTriangleResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_SphereTriangleContactInfo* const pContactInfo, float dt);

void sphereTraceSimulationSphereTerrainTriangleResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_SphereTerrainTriangleContactInfo* const pContactInfo, float dt);

void sphereTraceSimulationSphereSphereResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_SphereSphereContactInfo* const pContactInfo, float dt);

void sphereTraceSimulationGlobalSolveBruteForce(ST_SimulationSpace* const pSimulationSpace, float dt);

void sphereTraceSimulationGlobalSolveImposedPosition(ST_SimulationSpace* const pSimulationSpace, float dt);

b32 sphereTraceSimulationRayTrace(const ST_SimulationSpace* const pSimulationSpace, ST_Vector3 start, ST_Direction dir, ST_RayTraceData* const pRayCastData);

void sphereTraceSimulationSolveImposedPositionStaticSpacialPartition(ST_SimulationSpace* const pSimulationSpace, float dt);

