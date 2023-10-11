#define _CRT_SECURE_NO_WARNINGS
#include <stdlib.h>
#pragma once

#include "SphereTraceMath.h"
#include "SphereTraceLists.h"
#include "SphereTraceAllocator.h"
#include "SphereTraceRigidBody.h"
#include "SphereTraceCollider.h"
#include "SphereTraceColliderPlane.h"
#include "SphereTraceColliderTerrain.h"
#include "SphereTraceColliderSphere.h"
#include "SphereTraceColliderTriangle.h"
#include "SphereTraceColliderBowl.h"
#include "SphereTraceColliderPipe.h"
#include "SphereTraceDynamicArray.h"
#include "SphereTraceSpacialPartition.h"
#include "SphereTraceAI.h"


#define ST_BOUNCE_FORCE_MAX 5000.0f
#define ST_VELOCITY_THRESHOLD 0.5f
#define ST_AUTO_DT_FACTOR 1.0/10.0f
#define ST_SKIN_WIDTH 0.005f
#define ST_RESTING_SPEED_SQUARED 0.05f
#define ST_SPHERE_RESTING_SLOPE 0.005f
#define ST_SLIP_THRESHOLD 0.5f;
#define ST_FRICTION_MODIFIER 1.0f/5.0f


typedef struct ST_SimulationSpace
{
	ST_IndexList sphereColliders;
	ST_IndexList planeColliders;
	ST_IndexList triangleColliders;
	ST_IndexList uniformTerrainColliders;
	ST_IndexList bowlColliders;
	ST_IndexList pipeColliders;
	ST_VectorArrayContact spherePlaneContactInfos;
	ST_VectorArrayContact sphereSphereContactInfos;
	ST_VectorArrayContact sphereTriangleContactInfos;
	ST_VectorArrayContact sphereTerrainTriangleContactInfos;
	ST_VectorArrayContact sphereBowlContactInfos;
	float eta;
	ST_Vector3 gravitationalAcceleration;
	float friction;
	float minDeltaTime;
	ST_SpacialPartitionStaticContainer spacialPartitionContainer;
} ST_SimulationSpace;

ST_SimulationSpace sphereTraceSimulationConstruct(float eta, float friction, ST_Vector3 gravitationalAcceleration);

void sphereTraceSimulationInsertPlaneCollider(ST_SimulationSpace* const pSimulationSpace, ST_PlaneCollider* const pPlaneCollider);

void sphereTraceSimulationInsertBowlCollider(ST_SimulationSpace* const pSimulationSpace, ST_BowlCollider* const pBowlCollider);

void sphereTraceSimulationInsertPipeCollider(ST_SimulationSpace* const pSimulationSpace, ST_PipeCollider* const pPipeCollider);

void sphereTraceSimulationInsertSphereCollider(ST_SimulationSpace* const pSimulationSpace, ST_SphereCollider* const pSphereCollider);

void sphereTraceSimulationInsertUniformTerrainCollider(ST_SimulationSpace* const pSimulationSpace, ST_UniformTerrainCollider* const pTerrainCollider);

void sphereTraceSimulationUpdateSphereColliderBucketIndices(ST_SimulationSpace* const pSimulationSpace, ST_SphereCollider* const pSphereCollider);

void sphereTraceSimulationApplyForcesAndTorques(const ST_SimulationSpace* const pSimulationSpace, ST_RigidBody* const pRigidBody, float dt, b32 incrementGravity);

void sphereTraceSimulationStepQuantity(const ST_SimulationSpace* const pSimulationSpace, ST_RigidBody* const pRigidBody, float dt);

ST_Vector3 sphereTraceSimulationImposedStepPosition(const ST_SimulationSpace* const pSimulationSpace, ST_RigidBody* const pRigidBody, float dt);



void sphereTraceSimulationStepQuantities(const ST_SimulationSpace* const pSimulationSpace, float dt);

void sphereTraceSimulationSpherePlaneResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_Contact* const pContactInfo, float dt);

void sphereTraceSimulationSphereContactResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_Contact* const pContactInfo, float dt);

void sphereTraceSimulationSphereTriangleResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_Contact* const pContactInfo, float dt);

void sphereTraceSimulationSphereTerrainTriangleResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_Contact* const pContactInfo, float dt);

void sphereTraceSimulationSphereSphereResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_Contact* const pContactInfo, float dt);

void sphereTraceSimulationGlobalSolveBruteForce(ST_SimulationSpace* const pSimulationSpace, float dt);

void sphereTraceSimulationGlobalSolveBruteForceSpacialPartitioned(ST_SimulationSpace* const pSimulationSpace, float dt);

void sphereTraceSimulationGlobalSolveImposedPosition(ST_SimulationSpace* const pSimulationSpace, float dt);

b32 sphereTraceSimulationRayTrace(const ST_SimulationSpace* const pSimulationSpace, ST_Vector3 start, ST_Direction dir, ST_RayTraceData* const pRayCastData);

void sphereTraceSimulationSolveImposedPositionStaticSpacialPartition(ST_SimulationSpace* const pSimulationSpace, float dt);

