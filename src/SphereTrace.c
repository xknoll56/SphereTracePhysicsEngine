
#include "SphereTrace.h"

extern const ST_Vector3 gVector3Up;
extern const ST_Vector3 gVector3Right;
extern const ST_Vector3 gVector3Forward;
extern const ST_Vector3 gVector3Zero;
extern const ST_Vector3 gVector3One;
extern const ST_Vector3 gVector3Max;
extern const ST_Vector3 gVector3Left;
extern const ST_Vector3 gVector3Down;
extern const ST_Vector3 gVector3Back;
extern const ST_Vector4 gVector4Zero;
extern const ST_Vector4 gVector4One;
extern const ST_Vector4 gVector4ColorRed;
extern const ST_Vector4 gVector4ColorGreen;
extern const ST_Vector4 gVector4ColorBlue;
extern const ST_Quaternion gQuaternionIdentity;

ST_SimulationSpace sphereTraceSimulationConstruct(float eta, float friction, ST_Vector3 gravitationalAcceleration)
{
	ST_SimulationSpace simulationSpace;
	simulationSpace.eta = eta;
	simulationSpace.friction = friction;
	simulationSpace.gravitationalAcceleration = gravitationalAcceleration;
	simulationSpace.sphereColliders = sphereTraceVectorArrayPointersConstruct(0);
	simulationSpace.planeColliders = sphereTraceVectorArrayPointersConstruct(0);
	simulationSpace.triangleColliders = sphereTraceVectorArrayPointersConstruct(0);
	simulationSpace.uniformTerrainColliders = sphereTraceVectorArrayPointersConstruct(0);
	simulationSpace.spherePlaneContactInfos = sphereTracePointerVectorArrayContactInfoConstruct(0, CONTACT_SPHERE_PLANE);
	simulationSpace.sphereSphereContactInfos = sphereTracePointerVectorArrayContactInfoConstruct(0, CONTACT_SPHERE_SPHERE);
	simulationSpace.sphereTriangleContactInfos = sphereTracePointerVectorArrayContactInfoConstruct(0, CONTACT_SPHERE_TRIANGLE);
	simulationSpace.sphereTerrainTriangleContactInfos = sphereTracePointerVectorArrayContactInfoConstruct(0, CONTACT_SPHERE_TERRAIN_TRIANGLE);
	simulationSpace.spacialPartitionContainer = sphereTraceSpacialPartitionStaticConstruct(20.0f);
	return simulationSpace;
}

void sphereTraceSimulationInsertPlaneCollider(ST_SimulationSpace* const pSimulationSpace, ST_PlaneCollider* const pPlaneCollider)
{
	ST_ColliderIndex index = pSimulationSpace->planeColliders.count;
	pPlaneCollider->planeColliderIndex = index;
	sphereTraceVectorArrayPointersPushBack(&pSimulationSpace->planeColliders, pPlaneCollider);
	pPlaneCollider->bucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pPlaneCollider->aabb);
	ST_IntListData* ild = pPlaneCollider->bucketIndices.pFirst;
	for (int i = 0; i < pPlaneCollider->bucketIndices.count; i++)
	{
		if (ild->value != -1)
		{
			sphereTraceIntListAddUnique(&pSimulationSpace->spacialPartitionContainer.buckets[ild->value].planeColliderIndices, index);

		}
		else
		{
			sphereTraceIntListAddUnique(&pSimulationSpace->spacialPartitionContainer.outsideBucket.planeColliderIndices, index);
		}
		ild = ild->pNext;
	}
}

void sphereTraceSimulationInsertSphereCollider(ST_SimulationSpace* const pSimulationSpace, ST_SphereCollider* const pSphereCollider)
{
	ST_ColliderIndex index = pSimulationSpace->sphereColliders.count;
	pSphereCollider->sphereColliderIndex = index;
	sphereTraceVectorArrayPointersPushBack(&pSimulationSpace->sphereColliders, pSphereCollider);
	pSphereCollider->bucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pSphereCollider->aabb);
	ST_IntListData* ild = pSphereCollider->bucketIndices.pFirst;
	for (int i = 0; i < pSphereCollider->bucketIndices.count; i++)
	{
		if (ild->value != -1)
		{
			sphereTraceIntListAddUnique(&pSimulationSpace->spacialPartitionContainer.buckets[ild->value].sphereColliderIndices, index);

		}
		else
		{
			sphereTraceIntListAddUnique(&pSimulationSpace->spacialPartitionContainer.outsideBucket.sphereColliderIndices, index);
		}
		ild = ild->pNext;
	}
}

void sphereTraceSimulationUpdateSphereColliderBucketIndices(ST_SimulationSpace* const pSimulationSpace, ST_SphereCollider* const pSphereCollider)
{
	ST_IntList newBucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pSphereCollider->aabb);
	ST_IntList bucketsToDeleteSphereFrom = sphereTraceIntListConstructForDeletedValues(&pSphereCollider->bucketIndices, &newBucketIndices);
	ST_IntListData* ild = bucketsToDeleteSphereFrom.pFirst;
	for (int i = 0; i < bucketsToDeleteSphereFrom.count; i++)
	{
		if (ild->value != -1)
		{
			sphereTraceIntListRemoveFirstInstance(&pSimulationSpace->spacialPartitionContainer.buckets[ild->value].sphereColliderIndices, pSphereCollider->sphereColliderIndex);

		}
		else
		{
			sphereTraceIntListRemoveFirstInstance(&pSimulationSpace->spacialPartitionContainer.outsideBucket.sphereColliderIndices, pSphereCollider->sphereColliderIndex);
		}
		ild = ild->pNext;
	}
	ild = newBucketIndices.pFirst;
	for (int i = 0; i < newBucketIndices.count; i++)
	{
		if (ild->value != -1)
		{
			sphereTraceIntListAddUnique(&pSimulationSpace->spacialPartitionContainer.buckets[ild->value].sphereColliderIndices, pSphereCollider->sphereColliderIndex);

		}
		else
		{
			sphereTraceIntListAddUnique(&pSimulationSpace->spacialPartitionContainer.outsideBucket.sphereColliderIndices, pSphereCollider->sphereColliderIndex);
		}
		ild = ild->pNext;
	}
	sphereTraceIntListFree(&pSphereCollider->bucketIndices);
	sphereTraceIntListFree(&bucketsToDeleteSphereFrom);
	pSphereCollider->bucketIndices = newBucketIndices;
}

void sphereTraceSimulationApplyForcesAndTorques(const ST_SimulationSpace* const pSimulationSpace, ST_RigidBody* const pRigidBody, float dt, b32 incrementGravity)
{
	//apply gravity
	if (incrementGravity)
		pRigidBody->linearMomentum = sphereTraceVector3Add(pRigidBody->linearMomentum, sphereTraceVector3Scale(pSimulationSpace->gravitationalAcceleration, pRigidBody->mass * dt));

	sphereTraceRigidBodyApplyForces(pRigidBody, dt);
	sphereTraceRigidBodyApplyDeltaMomentums(pRigidBody);
	//set velocity
	pRigidBody->velocity = sphereTraceVector3Scale(pRigidBody->linearMomentum, pRigidBody->massInv);


	sphereTraceRigidBodyApplyTorques(pRigidBody, dt);
	sphereTraceRigidBodyApplyDeltaAngularMomentums(pRigidBody);
	//set angular velocity
	pRigidBody->angularVelocity = sphereTraceVector3Scale(pRigidBody->angularMomentum, pRigidBody->inertiaInv);
}

void sphereTraceSimulationStepQuantity(const ST_SimulationSpace* const pSimulationSpace, ST_RigidBody* const pRigidBody, float dt)
{
	sphereTraceSimulationApplyForcesAndTorques(pSimulationSpace, pRigidBody, dt, 1);

	pRigidBody->rotation = sphereTraceQuaternionAdd(pRigidBody->rotation, sphereTraceQuaternionMultiply(sphereTraceQuaternionConstruct(0.0f, 0.5f * dt * pRigidBody->angularVelocity.x, 0.5f * dt * pRigidBody->angularVelocity.y, 0.5f * dt * pRigidBody->angularVelocity.z), pRigidBody->rotation));
	pRigidBody->rotation = sphereTraceQuaternionNormalize(pRigidBody->rotation);
	pRigidBody->rotationMatrix = sphereTraceMatrixFromQuaternion(pRigidBody->rotation);

	pRigidBody->position = sphereTraceVector3Add(pRigidBody->position, sphereTraceVector3Scale(pRigidBody->velocity, dt));
}

ST_Vector3 sphereTraceSimulationImposedStepPosition(const ST_SimulationSpace* const pSimulationSpace, ST_RigidBody* const pRigidBody, float dt)
{
	//apply gravity
	ST_Vector3 imposedLinearMomentum = sphereTraceVector3Add(pRigidBody->linearMomentum, sphereTraceVector3Scale(pSimulationSpace->gravitationalAcceleration, pRigidBody->mass * dt));

	//apply forces
	ST_Vector3ListData* pData = pRigidBody->appliedForces.pFirst;
	for (int i = 0; i < pRigidBody->appliedForces.count; i++)
	{
		imposedLinearMomentum = sphereTraceVector3Add(imposedLinearMomentum, sphereTraceVector3Scale(pData->value, dt));
		pData = pData->pNext;
	}

	//apply dps
	pData = pRigidBody->appliedDeltaMomentums.pFirst;
	for (int i = 0; i < pRigidBody->appliedDeltaMomentums.count; i++)
	{
		imposedLinearMomentum = sphereTraceVector3Add(imposedLinearMomentum, pData->value);
		pData = pData->pNext;
	}

	//set velocity
	ST_Vector3 imposedVelocity = sphereTraceVector3Scale(imposedLinearMomentum, pRigidBody->massInv);

	//return the imposed next position
	return sphereTraceVector3Add(pRigidBody->position, sphereTraceVector3Scale(imposedVelocity, dt));
}



void sphereTraceSimulationStepQuantities(const ST_SimulationSpace* const pSimulationSpace, float dt)
{
	for (int i = 0; i < pSimulationSpace->sphereColliders.count; i++)
	{
		sphereTraceSimulationStepQuantity(pSimulationSpace, ((ST_SphereCollider*)pSimulationSpace->sphereColliders.data[i])->pRigidBody, dt);
	}
}

void sphereTraceSimulationSpherePlaneResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_SpherePlaneContactInfo* const pContactInfo, float dt)
{
	ST_SphereCollider* pSphereCollider = pContactInfo->pSphereCollider;
	ST_PlaneCollider* pPlaneCollider = pContactInfo->pPlaneCollider;
	pSphereCollider->pRigidBody->position = sphereTraceVector3Add(pSphereCollider->pRigidBody->position, sphereTraceVector3Scale(pContactInfo->normal, pContactInfo->penetrationDistance));
	ST_Vector3 r = sphereTraceVector3Subtract(pContactInfo->point, pSphereCollider->pRigidBody->position);
	float vnMag = sphereTraceVector3Dot(pContactInfo->normal, pSphereCollider->pRigidBody->velocity);
	ST_Vector3 vn = sphereTraceVector3Scale(pContactInfo->normal, vnMag);
	ST_Vector3 vt = sphereTraceVector3Subtract(pSphereCollider->pRigidBody->velocity, vn);
	float j = -(1.0f + pSimulationSpace->eta) * vnMag * pSphereCollider->pRigidBody->mass;
	ST_Vector3 dp = sphereTraceVector3Scale(pContactInfo->normal, j);
	float slope = sphereTraceVector3Dot(pContactInfo->normal, gVector3Up);

	if (fabsf(vnMag) > VELOCITY_THRESHOLD || slope < 0.0f)
	{
		sphereTraceRigidBodyAddDeltaMomentum(pSphereCollider->pRigidBody, dp);
		ST_Vector3 ft = sphereTraceVector3Scale(sphereTraceVector3Normalize(vt), -sphereTraceVector3Length(vt) * j / vnMag);
		ST_Vector3 dl = sphereTraceVector3Cross(r, sphereTraceVector3Scale(ft, -pSphereCollider->radius));
		//torque will be nan when ft is in direction of normal
		if (!sphereTraceVector3Nan(dl))
			sphereTraceRigidBodyAddDeltaAngularMomentum(pSphereCollider->pRigidBody, dl);
	}
	else
	{
		pSphereCollider->pRigidBody->linearMomentum = sphereTraceVector3Cross(sphereTraceVector3Cross(pContactInfo->normal, pSphereCollider->pRigidBody->linearMomentum), pContactInfo->normal);
		sphereTraceRigidBodySetAngularVelocity(pSphereCollider->pRigidBody, sphereTraceVector3Cross(pContactInfo->normal, sphereTraceVector3Scale(pSphereCollider->pRigidBody->velocity, 1.0f / pSphereCollider->radius)));
		sphereTraceRigidBodyAddForce(pSphereCollider->pRigidBody, sphereTraceVector3Scale(sphereTraceVector3Normalize(sphereTraceVector3Scale(pSphereCollider->pRigidBody->velocity, sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->normal))), pSimulationSpace->friction));
	}
}

void sphereTraceSimulationSphereTriangleResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_SphereTriangleContactInfo* const pContactInfo, float dt)
{
	ST_SphereCollider* pSphereCollider = pContactInfo->pSphereCollider;
	ST_TriangleCollider* pTriangleCollider = pContactInfo->pTriangleCollider;
	pSphereCollider->pRigidBody->position = sphereTraceVector3Add(pSphereCollider->pRigidBody->position, sphereTraceVector3Scale(pContactInfo->normal, pContactInfo->penetrationDistance));
	ST_Vector3 r = sphereTraceVector3Subtract(pContactInfo->point, pSphereCollider->pRigidBody->position);
	float vnMag = sphereTraceVector3Dot(pContactInfo->normal, pSphereCollider->pRigidBody->velocity);
	ST_Vector3 vn = sphereTraceVector3Scale(pContactInfo->normal, vnMag);
	ST_Vector3 vt = sphereTraceVector3Subtract(pSphereCollider->pRigidBody->velocity, vn);
	float j = -(1.0f + pSimulationSpace->eta) * vnMag * pSphereCollider->pRigidBody->mass;
	ST_Vector3 dp = sphereTraceVector3Scale(pContactInfo->normal, j);
	float slope = sphereTraceVector3Dot(pContactInfo->normal, gVector3Up);

	if (fabsf(vnMag) > VELOCITY_THRESHOLD || slope < 0.0f)
	{
		sphereTraceRigidBodyAddDeltaMomentum(pSphereCollider->pRigidBody, dp);
		ST_Vector3 ft = sphereTraceVector3Scale(sphereTraceVector3Normalize(vt), -sphereTraceVector3Length(vt) * j / vnMag);
		ST_Vector3 dl = sphereTraceVector3Cross(r, sphereTraceVector3Scale(ft, -pSphereCollider->radius));
		//torque will be nan when ft is in direction of normal
		if (!sphereTraceVector3Nan(dl))
			sphereTraceRigidBodyAddDeltaAngularMomentum(pSphereCollider->pRigidBody, dl);
	}
	else
	{
		pSphereCollider->pRigidBody->linearMomentum = sphereTraceVector3Cross(sphereTraceVector3Cross(pContactInfo->normal, pSphereCollider->pRigidBody->linearMomentum), pContactInfo->normal);
		sphereTraceRigidBodySetAngularVelocity(pSphereCollider->pRigidBody, sphereTraceVector3Cross(pContactInfo->normal, sphereTraceVector3Scale(pSphereCollider->pRigidBody->velocity, 1.0f / pSphereCollider->radius)));
		ST_Vector3 f = sphereTraceVector3Scale(sphereTraceVector3Normalize(sphereTraceVector3Scale(pSphereCollider->pRigidBody->velocity, sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->normal))), pSimulationSpace->friction);
		if (!sphereTraceVector3Nan(f))
			sphereTraceRigidBodyAddForce(pSphereCollider->pRigidBody, f);
	}
}

void sphereTraceSimulationSphereTerrainTriangleResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_SphereTerrainTriangleContactInfo* const pContactInfo, float dt)
{
	ST_SphereCollider* pSphereCollider = pContactInfo->sphereTriangleContactInfo.pSphereCollider;
	ST_TriangleCollider* pTriangleCollider = pContactInfo->sphereTriangleContactInfo.pTriangleCollider;
	//pSphereCollider->pRigidBody->position = sphereTraceVector3Add(pSphereCollider->pRigidBody->position, sphereTraceVector3Scale(pContactInfo->normal, pContactInfo->penetrationDistance));
	pSphereCollider->pRigidBody->position = pContactInfo->downSphereTraceData.sphereCenter;
	ST_Vector3 r = sphereTraceVector3Subtract(pContactInfo->downSphereTraceData.rayTraceData.hitPoint, pSphereCollider->pRigidBody->position);
	float vnMag = sphereTraceVector3Dot(pContactInfo->downSphereTraceData.rayTraceData.normal, pSphereCollider->pRigidBody->velocity);
	ST_Vector3 vn = sphereTraceVector3Scale(pContactInfo->downSphereTraceData.rayTraceData.normal, vnMag);
	ST_Vector3 vt = sphereTraceVector3Subtract(pSphereCollider->pRigidBody->velocity, vn);
	float j = -(1.0f + pSimulationSpace->eta) * vnMag * pSphereCollider->pRigidBody->mass;
	ST_Vector3 dp = sphereTraceVector3Scale(pContactInfo->downSphereTraceData.rayTraceData.normal, j);
	float slope = sphereTraceVector3Dot(pContactInfo->downSphereTraceData.rayTraceData.normal, gVector3Up);

	if (fabsf(vnMag) > VELOCITY_THRESHOLD || slope < 0.0f)
	{
		sphereTraceRigidBodyAddDeltaMomentum(pSphereCollider->pRigidBody, dp);
		ST_Vector3 ft = sphereTraceVector3Scale(sphereTraceVector3Normalize(vt), -sphereTraceVector3Length(vt) * j / vnMag);
		ST_Vector3 dl = sphereTraceVector3Cross(r, sphereTraceVector3Scale(ft, -pSphereCollider->radius));
		//torque will be nan when ft is in direction of normal
		if (!sphereTraceVector3Nan(dl))
			sphereTraceRigidBodyAddDeltaAngularMomentum(pSphereCollider->pRigidBody, dl);
	}
	else
	{
		pSphereCollider->pRigidBody->linearMomentum = sphereTraceVector3Cross(sphereTraceVector3Cross(pContactInfo->downSphereTraceData.rayTraceData.normal, pSphereCollider->pRigidBody->linearMomentum), pContactInfo->downSphereTraceData.rayTraceData.normal);
		sphereTraceRigidBodySetAngularVelocity(pSphereCollider->pRigidBody, sphereTraceVector3Cross(pContactInfo->downSphereTraceData.rayTraceData.normal, sphereTraceVector3Scale(pSphereCollider->pRigidBody->velocity, 1.0f / pSphereCollider->radius)));
		ST_Vector3 f = sphereTraceVector3Scale(sphereTraceVector3Normalize(sphereTraceVector3Scale(pSphereCollider->pRigidBody->velocity, sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->downSphereTraceData.rayTraceData.normal))), pSimulationSpace->friction);
		if (!sphereTraceVector3Nan(f))
			sphereTraceRigidBodyAddForce(pSphereCollider->pRigidBody, f);
	}
}

void sphereTraceSimulationSphereSphereResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_SphereSphereContactInfo* const pContactInfo, float dt)
{
	ST_SphereCollider* pA = pContactInfo->pA;
	ST_SphereCollider* pB = pContactInfo->pB;
	pA->pRigidBody->position = sphereTraceVector3AddAndScale(pA->pRigidBody->position, pContactInfo->normal, -pContactInfo->penetrationDistance * 0.5f);
	pB->pRigidBody->position = sphereTraceVector3AddAndScale(pB->pRigidBody->position, pContactInfo->normal, pContactInfo->penetrationDistance * 0.5f);
	ST_Vector3 relativeMomentum = sphereTraceVector3Subtract(pB->pRigidBody->linearMomentum, pA->pRigidBody->linearMomentum);
	ST_Vector3 jdt = sphereTraceVector3Scale(pContactInfo->normal, -(1.0f + pSimulationSpace->eta) * sphereTraceVector3Dot(relativeMomentum, pContactInfo->normal) / ((1.0f / pA->pRigidBody->mass) + (1.0f / pB->pRigidBody->mass)));
	sphereTraceRigidBodyAddDeltaMomentum(pB->pRigidBody, jdt);
	sphereTraceRigidBodyAddDeltaMomentum(pA->pRigidBody, sphereTraceVector3Negative(jdt));
}

void sphereTraceSimulationGlobalSolveBruteForce(ST_SimulationSpace* const pSimulationSpace, float dt)
{
	//update all sphere aabb's 
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSimulationSpace->sphereColliders.data[sphereColliderIndex];
		sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
	}

	sphereTraceSimulationStepQuantities(pSimulationSpace, dt);

	//clear the contact infos (if not empty)
	sphereTracePointerVectorArrayContactInfoFree(&pSimulationSpace->spherePlaneContactInfos);

	//allocate the contact infos for as many sphere colliders we have
	pSimulationSpace->spherePlaneContactInfos = sphereTracePointerVectorArrayContactInfoConstruct(pSimulationSpace->sphereColliders.count, CONTACT_SPHERE_PLANE);

	//clear the contact infos (if not empty)
	sphereTracePointerVectorArrayContactInfoFree(&pSimulationSpace->sphereSphereContactInfos);

	//allocate the contact infos for as many sphere colliders we have
	pSimulationSpace->sphereSphereContactInfos = sphereTracePointerVectorArrayContactInfoConstruct(pSimulationSpace->sphereColliders.count, CONTACT_SPHERE_SPHERE);

	//clear the contact infos (if not empty)
	sphereTracePointerVectorArrayContactInfoFree(&pSimulationSpace->sphereTriangleContactInfos);

	//allocate the contact infos for as many sphere colliders we have
	pSimulationSpace->sphereTriangleContactInfos = sphereTracePointerVectorArrayContactInfoConstruct(pSimulationSpace->sphereColliders.count, CONTACT_SPHERE_TRIANGLE);

	//clear the contact infos (if not empty)
	sphereTracePointerVectorArrayContactInfoFree(&pSimulationSpace->sphereTerrainTriangleContactInfos);

	//allocate the contact infos for as many sphere colliders we have
	pSimulationSpace->sphereTerrainTriangleContactInfos = sphereTracePointerVectorArrayContactInfoConstruct(pSimulationSpace->sphereColliders.count, CONTACT_SPHERE_TERRAIN_TRIANGLE);

	//check for all sphere plane collisions
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSimulationSpace->sphereColliders.data[sphereColliderIndex];
		for (int planeColliderIndex = 0; planeColliderIndex < pSimulationSpace->planeColliders.count; planeColliderIndex++)
		{
			ST_PlaneCollider* pPlaneCollider = (ST_PlaneCollider*)pSimulationSpace->planeColliders.data[planeColliderIndex];
			ST_SpherePlaneContactInfo contactInfo;
			if (sphereTraceColliderPlaneSphereCollisionTest(pPlaneCollider, pSphereCollider, &contactInfo))
			{
				sphereTracePointerVectorArrayContactInfoPushBack(&pSimulationSpace->spherePlaneContactInfos, &contactInfo, CONTACT_SPHERE_PLANE);
			}
		}

		for (int triangleColliderIndex = 0; triangleColliderIndex < pSimulationSpace->triangleColliders.count; triangleColliderIndex++)
		{
			ST_TriangleCollider* pTriangleCollider = (ST_TriangleCollider*)pSimulationSpace->triangleColliders.data[triangleColliderIndex];
			ST_SphereTriangleContactInfo contactInfo;
			if (sphereTraceColliderTriangleSphereCollisionTest(pTriangleCollider, pSphereCollider, &contactInfo))
			{
				sphereTracePointerVectorArrayContactInfoPushBack(&pSimulationSpace->sphereTriangleContactInfos, &contactInfo, CONTACT_SPHERE_TRIANGLE);
			}
		}

		for (int terrainColliderIndex = 0; terrainColliderIndex < pSimulationSpace->uniformTerrainColliders.count; terrainColliderIndex++)
		{
			ST_UniformTerrainCollider* pTerrainCollider = (ST_UniformTerrainCollider*)pSimulationSpace->uniformTerrainColliders.data[terrainColliderIndex];
			ST_SphereTerrainTriangleContactInfo contactInfo;
			if (sphereTraceColliderUniformTerrainSphereFindMaxPenetratingTriangle(pTerrainCollider, pSphereCollider, &contactInfo))
			{
				sphereTracePointerVectorArrayContactInfoPushBack(&pSimulationSpace->sphereTerrainTriangleContactInfos, &contactInfo, CONTACT_SPHERE_TERRAIN_TRIANGLE);
			}
		}

		//check for all sphere sphere collisions
		for (int sphereColliderBIndex = sphereColliderIndex + 1; sphereColliderBIndex < pSimulationSpace->sphereColliders.count; sphereColliderBIndex++)
		{
			ST_SphereCollider* pSphereColliderB = (ST_SphereCollider*)pSimulationSpace->sphereColliders.data[sphereColliderBIndex];
			ST_SphereSphereContactInfo contactInfo;
			if (sphereTraceColliderSphereSphereCollisionTest(pSphereCollider, pSphereColliderB, &contactInfo))
			{
				sphereTracePointerVectorArrayContactInfoPushBack(&pSimulationSpace->sphereSphereContactInfos, &contactInfo, CONTACT_SPHERE_SPHERE);
			}
		}
	}


	//resolve all sphere plane collisions
	for (int i = 0; i < pSimulationSpace->spherePlaneContactInfos.count; i++)
	{
		ST_SpherePlaneContactInfo* contactInfo = (ST_SpherePlaneContactInfo*)sphereTracePointerVectorArrayContactInfoGet(&pSimulationSpace->spherePlaneContactInfos, i, CONTACT_SPHERE_PLANE);
		sphereTraceSimulationSpherePlaneResponse(pSimulationSpace, contactInfo, dt);
	}

	//resolve all sphere plane collisions
	for (int i = 0; i < pSimulationSpace->sphereTriangleContactInfos.count; i++)
	{
		ST_SphereTriangleContactInfo* contactInfo = (ST_SphereTriangleContactInfo*)sphereTracePointerVectorArrayContactInfoGet(&pSimulationSpace->sphereTriangleContactInfos, i, CONTACT_SPHERE_TRIANGLE);
		sphereTraceSimulationSphereTriangleResponse(pSimulationSpace, contactInfo, dt);
	}

	//resolve all sphere triangle collisions
	for (int i = 0; i < pSimulationSpace->sphereTerrainTriangleContactInfos.count; i++)
	{
		ST_SphereTerrainTriangleContactInfo* contactInfo = (ST_SphereTerrainTriangleContactInfo*)sphereTracePointerVectorArrayContactInfoGet(&pSimulationSpace->sphereTerrainTriangleContactInfos, i, CONTACT_SPHERE_TERRAIN_TRIANGLE);
		sphereTraceSimulationSphereTerrainTriangleResponse(pSimulationSpace, contactInfo, dt);
	}

	//resolve all sphere plane collisions
	for (int i = 0; i < pSimulationSpace->sphereSphereContactInfos.count; i++)
	{
		ST_SphereSphereContactInfo* contactInfo = (ST_SphereSphereContactInfo*)sphereTracePointerVectorArrayContactInfoGet(&pSimulationSpace->sphereSphereContactInfos, i, CONTACT_SPHERE_SPHERE);
		sphereTraceSimulationSphereSphereResponse(pSimulationSpace, contactInfo, dt);
	}
}

void sphereTraceSimulationGlobalSolveImposedPosition(ST_SimulationSpace* const pSimulationSpace, float dt)
{

	//update all sphere aabb's 
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSimulationSpace->sphereColliders.data[sphereColliderIndex];
		sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
	}

	//check for all sphere plane collisions
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		float accumulatedDt = 0.0f;
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSimulationSpace->sphereColliders.data[sphereColliderIndex];
		//apply all pre existing forces and torques
		sphereTraceSimulationApplyForcesAndTorques(pSimulationSpace, pSphereCollider->pRigidBody, dt, 0);
		while (accumulatedDt < dt)
		{
			//sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);

			ST_SphereCollider* pClosestSphere = NULL;
			ST_PlaneCollider* pClosestPlane = NULL;
			ST_UniformTerrainCollider* pClosestTerrain = NULL;
			float closestCollider = FLT_MAX;
			float closestPoint = FLT_MAX;
			ST_SphereTraceData sphereCastData;
			ST_SphereTraceData sphereCastDataClosestPlane;
			ST_SphereTraceData sphereCastDataClosestSphere;
			ST_SphereTraceData sphereCastDataClosestTerrain;

			ST_Vector3 imposedNextPosition = sphereTraceSimulationImposedStepPosition(pSimulationSpace, pSphereCollider->pRigidBody, dt - accumulatedDt);
			float imposedNextLength = sphereTraceVector3Length(sphereTraceVector3Subtract(imposedNextPosition, pSphereCollider->pRigidBody->position));

			//check for the closest plane
			for (int planeColliderIndex = 0; planeColliderIndex < pSimulationSpace->planeColliders.count; planeColliderIndex++)
			{
				ST_PlaneCollider* pPlaneCollider = (ST_PlaneCollider*)pSimulationSpace->planeColliders.data[planeColliderIndex];
				if (sphereTraceColliderPlaneSphereTrace(pSphereCollider->pRigidBody->position, pSphereCollider->pRigidBody->velocity, pSphereCollider->radius, pPlaneCollider, &sphereCastData))
				{
					//float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastData.sphereCenter, pSphereCollider->pRigidBody->position));
					if (sphereCastData.traceDistance <= imposedNextLength)
					{
						if (sphereCastData.rayTraceData.distance < closestCollider)
						{
							pClosestPlane = pPlaneCollider;
							closestCollider = sphereCastData.rayTraceData.distance;
							sphereCastDataClosestPlane = sphereCastData;
						}
					}
				}
			}

			//check for the closest terrain
			for (int terrainColliderIndex = 0; terrainColliderIndex < pSimulationSpace->uniformTerrainColliders.count; terrainColliderIndex++)
			{
				ST_UniformTerrainCollider* pTerrainCollider = (ST_UniformTerrainCollider*)pSimulationSpace->uniformTerrainColliders.data[terrainColliderIndex];
				//if (sphereTraceColliderUniformTerrainSphereTrace(pTerrainCollider, pSphereCollider->pRigidBody->position, pSphereCollider->pRigidBody->velocity, pSphereCollider->radius, &sphereCastData))
				//ST_SphereTerrainTriangleContactInfo contactInfo;
				//if (sphereTraceColliderUniformTerrainSphereTraceByStartEndPoint(pTerrainCollider,pSphereCollider->pRigidBody->position, imposedNextPosition, pSphereCollider->radius, &sphereCastData))
				if (sphereTraceColliderUniformTerrainSphereTraceByStartEndPoint(pTerrainCollider, pSphereCollider->pRigidBody->position, imposedNextPosition, pSphereCollider->radius, &sphereCastData))
				{
					if (sphereCastData.traceDistance <= imposedNextLength)
					{
						if (sphereCastData.rayTraceData.distance < closestCollider)
						{
							pClosestTerrain = pTerrainCollider;
							closestCollider = sphereCastData.rayTraceData.distance;
							sphereCastDataClosestTerrain = sphereCastData;
							pClosestPlane = NULL;
						}
					}
				}
			}
			//check for all sphere sphere collisions
			for (int sphereColliderOtherIndex = sphereColliderIndex + 1; sphereColliderOtherIndex < pSimulationSpace->sphereColliders.count; sphereColliderOtherIndex++)
			{
				ST_SphereCollider* pSphereColliderOther = (ST_SphereCollider*)pSimulationSpace->sphereColliders.data[sphereColliderOtherIndex];
				if (sphereTraceColliderSphereSphereTrace(pSphereCollider->pRigidBody->position, pSphereCollider->pRigidBody->velocity, pSphereCollider->radius, pSphereColliderOther, &sphereCastData))
				{
					//ST_Vector3 nextPos = 
					//float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastData.sphereCenter, pSphereCollider->pRigidBody->position));
					if (sphereCastData.traceDistance <= imposedNextLength)
					{
						if (sphereCastData.rayTraceData.distance < closestCollider)
						{
							pClosestSphere = pSphereColliderOther;
							pClosestPlane = NULL;
							pClosestTerrain = NULL;
							closestCollider = sphereCastData.rayTraceData.distance;
							sphereCastDataClosestSphere = sphereCastData;
						}
					}
				}
			}
			//sphereTraceColliderUniformTerrainSphereTraceByStartEndPoint()
			if (closestCollider != FLT_MAX)
			{
				if (pClosestPlane != NULL)
				{
					float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastDataClosestPlane.sphereCenter, pSphereCollider->pRigidBody->position));
					//float test = pSphereCollider->radius - sphereCastDataClosestPlane.rayTraceData.distance;
					//if(test>0)
					//printf("test: %f\n", test);
					float adjustedDt = fminf((dt - accumulatedDt) * sphereCastDistance / imposedNextLength + AUTO_DT_FACTOR * dt, dt - accumulatedDt);

					//step the simulation
					sphereTraceSimulationStepQuantity(pSimulationSpace, pSphereCollider->pRigidBody, adjustedDt);
					//sphereTraceVector3Print(pSphereCollider->pRigidBody->linearMomentum);

					//resolve the collisions if any
					ST_SpherePlaneContactInfo contactInfo;
					if (sphereTraceColliderPlaneSphereCollisionTest(pClosestPlane, pSphereCollider, &contactInfo))
					{
						sphereTraceSimulationSpherePlaneResponse(pSimulationSpace, &contactInfo, adjustedDt);
					}
					//printf("after: ");
					//sphereTraceVector3Print(pSphereCollider->pRigidBody->velocity);
					accumulatedDt += adjustedDt;
				}
				else if (pClosestSphere != NULL)
				{
					float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastDataClosestSphere.sphereCenter, pSphereCollider->pRigidBody->position));
					float adjustedDt = fminf((dt - accumulatedDt) * sphereCastDistance / imposedNextLength + AUTO_DT_FACTOR * dt, dt - accumulatedDt);

					//step the simulation
					sphereTraceSimulationStepQuantity(pSimulationSpace, pSphereCollider->pRigidBody, adjustedDt);

					ST_SphereSphereContactInfo contactInfo;
					if (sphereTraceColliderSphereSphereCollisionTest(pSphereCollider, pClosestSphere, &contactInfo))
					{
						sphereTraceSimulationSphereSphereResponse(pSimulationSpace, &contactInfo, adjustedDt);
					}
					accumulatedDt += adjustedDt;
				}
				else if (pClosestTerrain != NULL)
				{
					float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastDataClosestTerrain.sphereCenter, pSphereCollider->pRigidBody->position));
					float adjustedDt = fminf((dt - accumulatedDt) * sphereCastDistance / imposedNextLength + AUTO_DT_FACTOR * dt, dt - accumulatedDt);

					//step the simulation
					sphereTraceSimulationStepQuantity(pSimulationSpace, pSphereCollider->pRigidBody, adjustedDt);

					ST_SphereTerrainTriangleContactInfo contactInfo;
					if (sphereTraceColliderUniformTerrainSphereFindMaxPenetratingTriangle(pClosestTerrain, pSphereCollider, &contactInfo))
					{
						sphereTraceSimulationSphereTerrainTriangleResponse(pSimulationSpace, &contactInfo, adjustedDt);
					}
					accumulatedDt += adjustedDt;
				}
			}
			else
			{
				float adjustedDt = dt - accumulatedDt;
				sphereTraceSimulationStepQuantity(pSimulationSpace, pSphereCollider->pRigidBody, adjustedDt);
				accumulatedDt += adjustedDt;
			}
		}
	}
}

b32 sphereTraceSimulationRayTrace(const ST_SimulationSpace* const pSimulationSpace, ST_Vector3 start, ST_Vector3 dir, ST_RayTraceData* const pRayCastData)
{
	int currentBucketIndex = 0;
	ST_Vector3 intersection = start;
	dir = sphereTraceVector3Normalize(dir);
	ST_Vector3 incomingDir;
	currentBucketIndex = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(&pSimulationSpace->spacialPartitionContainer, intersection);
	ST_SpacialPartitionBucket bucket; //= pSimulationSpace->spacialPartitionContainer.buckets[currentBucketIndex];
	ST_RayTraceData tentativeData;
	pRayCastData->distance = FLT_MAX;
	//tentativeData.distance = FLT_MAX;
	while (currentBucketIndex != -1)
	{
		bucket = pSimulationSpace->spacialPartitionContainer.buckets[currentBucketIndex];
		b32 hit = 0;
		ST_IntListData* pild = bucket.planeColliderIndices.pFirst;
		for (int i = 0; i < bucket.planeColliderIndices.count; i++)
		{
			if (sphereTraceColliderPlaneRayTrace(start, dir, (ST_PlaneCollider*)pSimulationSpace->planeColliders.data[pild->value], &tentativeData))
			{
				if (tentativeData.distance < pRayCastData->distance)
				{
					hit = 1;
					*pRayCastData = tentativeData;
				}
			}
			pild = pild->pNext;
		}
		pild = bucket.sphereColliderIndices.pFirst;
		for (int i = 0; i < bucket.sphereColliderIndices.count; i++)
		{
			if (sphereTraceColliderSphereRayTrace(start, dir, (ST_SphereCollider*)pSimulationSpace->sphereColliders.data[pild->value], &tentativeData))
			{
				if (tentativeData.distance < pRayCastData->distance)
				{
					hit = 1;
					*pRayCastData = tentativeData;
				}
			}
			pild = pild->pNext;
		}
		if (hit)
		{
			return 1;
		}
		intersection = sphereTraceSpacialPartitionStaticGetNearestBucketIntersectionFromPositionAndDirection(&bucket, intersection, dir, &incomingDir);
		ST_Vector3 checkPoint = sphereTraceVector3AddAndScale(intersection, incomingDir, pSimulationSpace->spacialPartitionContainer.partitionSize * 0.5f);
		currentBucketIndex = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(&pSimulationSpace->spacialPartitionContainer, checkPoint);
	}
	//check outside
	b32 hit = 0;
	bucket = pSimulationSpace->spacialPartitionContainer.outsideBucket;
	ST_IntListData* pild = bucket.planeColliderIndices.pFirst;
	for (int i = 0; i < bucket.planeColliderIndices.count; i++)
	{
		if (sphereTraceColliderPlaneRayTrace(start, dir, (ST_PlaneCollider*)pSimulationSpace->planeColliders.data[pild->value], &tentativeData))
		{
			if (tentativeData.distance < pRayCastData->distance)
			{
				hit = 1;
				*pRayCastData = tentativeData;
			}
		}
		pild = pild->pNext;
	}
	pild = bucket.sphereColliderIndices.pFirst;
	for (int i = 0; i < bucket.sphereColliderIndices.count; i++)
	{
		if (sphereTraceColliderSphereRayTrace(start, dir, (ST_SphereCollider*)pSimulationSpace->sphereColliders.data[pild->value], &tentativeData))
		{
			if (tentativeData.distance < pRayCastData->distance)
			{
				hit = 1;
				*pRayCastData = tentativeData;
			}
		}
		pild = pild->pNext;
	}
	return hit;
}

void sphereTraceSimulationSolveImposedPositionStaticSpacialPartition(ST_SimulationSpace* const pSimulationSpace, float dt)
{
	//update all sphere aabb's and buckets
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSimulationSpace->sphereColliders.data[sphereColliderIndex];
		sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
		sphereTraceSimulationUpdateSphereColliderBucketIndices(pSimulationSpace, pSphereCollider);
	}

	//check for all sphere plane collisions
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		float accumulatedDt = 0.0f;
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSimulationSpace->sphereColliders.data[sphereColliderIndex];
		ST_AABB imposedSpherePathAABB;
		ST_SphereTraceData imposedPathSphereCastData;
		//apply all pre existing forces and torques
		sphereTraceSimulationApplyForcesAndTorques(pSimulationSpace, pSphereCollider->pRigidBody, dt, 0);
		while (accumulatedDt < dt)
		{
			if (!pSphereCollider->ignoreCollisions)
			{
				ST_SphereCollider* pClosestSphere = NULL;
				ST_PlaneCollider* pClosestPlane = NULL;
				float closestCollider = FLT_MAX;
				float closestPoint = FLT_MAX;
				ST_SphereTraceData sphereCastData;
				ST_SphereTraceData sphereCastDataClosestPlane;
				ST_SphereTraceData sphereCastDataClosestSphere;

				imposedPathSphereCastData.radius = pSphereCollider->radius;
				imposedPathSphereCastData.rayTraceData.startPoint = pSphereCollider->pRigidBody->position;
				ST_Vector3 imposedNextPosition = sphereTraceSimulationImposedStepPosition(pSimulationSpace, pSphereCollider->pRigidBody, dt - accumulatedDt);
				imposedPathSphereCastData.sphereCenter = imposedNextPosition;
				sphereTraceColliderResizeAABBWithSpherecast(&imposedPathSphereCastData, &imposedSpherePathAABB);
				float imposedNextLength = sphereTraceVector3Length(sphereTraceVector3Subtract(imposedNextPosition, pSphereCollider->pRigidBody->position));

				ST_IntList bucketsToCheck = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &imposedSpherePathAABB);

				ST_IntListData* pild = bucketsToCheck.pFirst;
				ST_IntList checkedPlaneColliders = sphereTraceIntListConstruct();
				ST_IntList checkedSphereColliders = sphereTraceIntListConstruct();
				for (int bucketListIndex = 0; bucketListIndex < bucketsToCheck.count; bucketListIndex++)
				{
					ST_SpacialPartitionBucket bucket;
					if (pild->value == -1)
					{
						bucket = pSimulationSpace->spacialPartitionContainer.outsideBucket;
					}
					else
					{
						bucket = pSimulationSpace->spacialPartitionContainer.buckets[pild->value];
					}

					if (bucket.planeColliderIndices.count > 0)
					{
						ST_IntListData* pPlaneColliderILD = bucket.planeColliderIndices.pFirst;
						//check for the closest plane
						for (int i = 0; i < bucket.planeColliderIndices.count; i++)
						{
							ST_PlaneCollider* pPlaneCollider = (ST_PlaneCollider*)pSimulationSpace->planeColliders.data[pPlaneColliderILD->value];
							if (sphereTraceIntListAddUnique(&checkedPlaneColliders, pPlaneCollider->planeColliderIndex))
							{
								if (sphereTraceColliderPlaneSphereTrace(pSphereCollider->pRigidBody->position, pSphereCollider->pRigidBody->velocity, pSphereCollider->radius, pPlaneCollider, &sphereCastData))
								{
									float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastData.sphereCenter, pSphereCollider->pRigidBody->position));
									if (sphereCastDistance <= imposedNextLength)
									{
										if (sphereCastData.rayTraceData.distance < closestCollider)
										{
											pClosestPlane = pPlaneCollider;
											closestCollider = sphereCastData.rayTraceData.distance;
											sphereCastDataClosestPlane = sphereCastData;
										}
									}
								}
							}
							pPlaneColliderILD = pPlaneColliderILD->pNext;

						}
					}
					if (bucket.sphereColliderIndices.count > 0)
					{
						ST_IntListData* pSphereColliderILD = bucket.sphereColliderIndices.pFirst;
						//check for all sphere sphere collisions
						for (int i = 0; i < bucket.sphereColliderIndices.count; i++)
						{
							if (pSphereColliderILD->value != pSphereCollider->sphereColliderIndex)
							{
								ST_SphereCollider* pSphereColliderOther = (ST_SphereCollider*)pSimulationSpace->sphereColliders.data[pSphereColliderILD->value];
								if (sphereTraceIntListAddUnique(&checkedSphereColliders, pSphereColliderOther->sphereColliderIndex))
								{
									if (sphereTraceColliderSphereSphereTrace(pSphereCollider->pRigidBody->position, pSphereCollider->pRigidBody->velocity, pSphereCollider->radius, pSphereColliderOther, &sphereCastData))
									{
										float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastData.sphereCenter, pSphereCollider->pRigidBody->position));
										if (sphereCastDistance <= imposedNextLength)
										{
											if (sphereCastData.rayTraceData.distance < closestCollider)
											{
												pClosestSphere = pSphereColliderOther;
												pClosestPlane = NULL;
												closestCollider = sphereCastData.rayTraceData.distance;
												sphereCastDataClosestSphere = sphereCastData;
											}
										}
									}
								}
							}
							pSphereColliderILD = pSphereColliderILD->pNext;
						}
					}
					pild = pild->pNext;
				}
				sphereTraceIntListFree(&checkedPlaneColliders);
				sphereTraceIntListFree(&checkedSphereColliders);
				sphereTraceIntListFree(&bucketsToCheck);
				if (closestCollider != FLT_MAX)
				{
					if (pClosestPlane != NULL)
					{
						float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastDataClosestPlane.sphereCenter, pSphereCollider->pRigidBody->position));
						float adjustedDt = fminf((dt - accumulatedDt) * sphereCastDistance / imposedNextLength + AUTO_DT_FACTOR * dt, dt - accumulatedDt);

						//step the simulation
						sphereTraceSimulationStepQuantity(pSimulationSpace, pSphereCollider->pRigidBody, adjustedDt);

						//resolve the collisions if any
						ST_SpherePlaneContactInfo contactInfo;
						if (sphereTraceColliderPlaneSphereCollisionTest(pClosestPlane, pSphereCollider, &contactInfo))
						{
							sphereTraceSimulationSpherePlaneResponse(pSimulationSpace, &contactInfo, adjustedDt);
						}
						accumulatedDt += adjustedDt;
					}
					else if (pClosestSphere != NULL)
					{
						float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastDataClosestSphere.sphereCenter, pSphereCollider->pRigidBody->position));
						float adjustedDt = fminf((dt - accumulatedDt) * sphereCastDistance / imposedNextLength + AUTO_DT_FACTOR * dt, dt - accumulatedDt);

						//step the simulation
						sphereTraceSimulationStepQuantity(pSimulationSpace, pSphereCollider->pRigidBody, adjustedDt);

						ST_SphereSphereContactInfo contactInfo;
						if (sphereTraceColliderSphereSphereCollisionTest(pSphereCollider, pClosestSphere, &contactInfo))
						{
							sphereTraceSimulationSphereSphereResponse(pSimulationSpace, &contactInfo, adjustedDt);
						}
						accumulatedDt += adjustedDt;
					}
				}
				else
				{
					float adjustedDt = dt - accumulatedDt;
					sphereTraceSimulationStepQuantity(pSimulationSpace, pSphereCollider->pRigidBody, adjustedDt);
					accumulatedDt += adjustedDt;
				}
			}
			else
			{
				float adjustedDt = dt - accumulatedDt;
				sphereTraceSimulationStepQuantity(pSimulationSpace, pSphereCollider->pRigidBody, adjustedDt);
				accumulatedDt += adjustedDt;
			}
			sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
			sphereTraceSimulationUpdateSphereColliderBucketIndices(pSimulationSpace, pSphereCollider);
		}
	}
}