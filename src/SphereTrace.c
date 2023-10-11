
#include "SphereTrace.h"
#include "SphereTraceGlobals.h"

ST_SimulationSpace sphereTraceSimulationConstruct(float eta, float friction, ST_Vector3 gravitationalAcceleration)
{
	ST_SimulationSpace simulationSpace;
	simulationSpace.eta = eta;
	simulationSpace.friction = friction;
	simulationSpace.gravitationalAcceleration = gravitationalAcceleration;
	simulationSpace.sphereColliders = sphereTraceIndexListConstruct();
	simulationSpace.planeColliders = sphereTraceIndexListConstruct();
	simulationSpace.triangleColliders = sphereTraceIndexListConstruct();
	simulationSpace.uniformTerrainColliders = sphereTraceIndexListConstruct();
	simulationSpace.bowlColliders = sphereTraceIndexListConstruct();
	simulationSpace.pipeColliders = sphereTraceIndexListConstruct();
	simulationSpace.spherePlaneContactInfos = sphereTraceVectorArrayContactInfoConstruct(0, CONTACT_SPHERE_PLANE);
	simulationSpace.sphereSphereContactInfos = sphereTraceVectorArrayContactInfoConstruct(0, CONTACT_SPHERE_SPHERE);
	simulationSpace.sphereTriangleContactInfos = sphereTraceVectorArrayContactInfoConstruct(0, CONTACT_SPHERE_TRIANGLE);
	simulationSpace.sphereTerrainTriangleContactInfos = sphereTraceVectorArrayContactInfoConstruct(0, CONTACT_SPHERE_TERRAIN_TRIANGLE);
	simulationSpace.spacialPartitionContainer = sphereTraceSpacialPartitionStaticConstruct(20.0f);
	simulationSpace.minDeltaTime = 1.0f / 60.0f;
	return simulationSpace;
}

void sphereTraceSimulationInsertPlaneCollider(ST_SimulationSpace* const pSimulationSpace, ST_PlaneCollider* const pPlaneCollider)
{
	//ST_ColliderIndex index = pSimulationSpace->planeColliders.count;
	pPlaneCollider->collider.colliderIndex = pPlaneCollider;
	sphereTraceIndexListAddFirst(&pSimulationSpace->planeColliders, pPlaneCollider);
	pPlaneCollider->collider.bucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pPlaneCollider->aabb);
	ST_IndexListData* ild = pPlaneCollider->collider.bucketIndices.pFirst;
	for (int i = 0; i < pPlaneCollider->collider.bucketIndices.count; i++)
	{
		if (ild->value != -1)
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.buckets[ild->value].planeColliderIndices, pPlaneCollider);

		}
		else
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.outsideBucket.planeColliderIndices, pPlaneCollider);
		}
		ild = ild->pNext;
	}
}

void sphereTraceSimulationInsertBowlCollider(ST_SimulationSpace* const pSimulationSpace, ST_BowlCollider* const pBowlCollider)
{
	//ST_ColliderIndex index = pSimulationSpace->planeColliders.count;
	pBowlCollider->collider.colliderIndex = pBowlCollider;
	sphereTraceIndexListAddFirst(&pSimulationSpace->bowlColliders, pBowlCollider);
	pBowlCollider->collider.bucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pBowlCollider->aabb);
	ST_IndexListData* ild = pBowlCollider->collider.bucketIndices.pFirst;
	for (int i = 0; i < pBowlCollider->collider.bucketIndices.count; i++)
	{
		if (ild->value != -1)
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.buckets[ild->value].bowlColliderIndices, pBowlCollider);

		}
		else
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.outsideBucket.bowlColliderIndices, pBowlCollider);
		}
		ild = ild->pNext;
	}
}

void sphereTraceSimulationInsertPipeCollider(ST_SimulationSpace* const pSimulationSpace, ST_PipeCollider* const pPipeCollider)
{
	//ST_ColliderIndex index = pSimulationSpace->planeColliders.count;
	pPipeCollider->collider.colliderIndex = pPipeCollider;
	sphereTraceIndexListAddFirst(&pSimulationSpace->pipeColliders, pPipeCollider);
	pPipeCollider->collider.bucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pPipeCollider->aabb);
	ST_IndexListData* ild = pPipeCollider->collider.bucketIndices.pFirst;
	for (int i = 0; i < pPipeCollider->collider.bucketIndices.count; i++)
	{
		if (ild->value != -1)
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.buckets[ild->value].pipeColliderIndices, pPipeCollider);

		}
		else
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.outsideBucket.pipeColliderIndices, pPipeCollider);
		}
		ild = ild->pNext;
	}
}

void sphereTraceSimulationInsertSphereCollider(ST_SimulationSpace* const pSimulationSpace, ST_SphereCollider* const pSphereCollider)
{
	//ST_ColliderIndex index = pSimulationSpace->sphereColliders.count;
	pSphereCollider->collider.colliderIndex = pSphereCollider;
	sphereTraceIndexListAddFirst(&pSimulationSpace->sphereColliders, pSphereCollider);
	pSphereCollider->collider.bucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pSphereCollider->aabb);
	ST_IndexListData* ild = pSphereCollider->collider.bucketIndices.pFirst;
	for (int i = 0; i < pSphereCollider->collider.bucketIndices.count; i++)
	{
		if (ild->value != -1)
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.buckets[ild->value].sphereColliderIndices, pSphereCollider);

		}
		else
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.outsideBucket.sphereColliderIndices, pSphereCollider);
		}
		ild = ild->pNext;
	}
}

void sphereTraceSimulationInsertUniformTerrainCollider(ST_SimulationSpace* const pSimulationSpace, ST_UniformTerrainCollider* const pTerrainCollider)
{
	//ST_ColliderIndex index = pSimulationSpace->uniformTerrainColliders.count;
	pTerrainCollider->collider.colliderIndex = pTerrainCollider;
	sphereTraceIndexListAddFirst(&pSimulationSpace->uniformTerrainColliders, pTerrainCollider);
	pTerrainCollider->collider.bucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pTerrainCollider->aabb);
	ST_IndexListData* ild = pTerrainCollider->collider.bucketIndices.pFirst;
	for (int i = 0; i < pTerrainCollider->collider.bucketIndices.count; i++)
	{
		if (ild->value != -1)
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.buckets[ild->value].uniformTerrainColliderIndices, pTerrainCollider);

		}
		else
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.outsideBucket.uniformTerrainColliderIndices, pTerrainCollider);
		}
		ild = ild->pNext;
	}
}

void sphereTraceSimulationUpdateSphereColliderBucketIndices(ST_SimulationSpace* const pSimulationSpace, ST_SphereCollider* const pSphereCollider)
{
	ST_IndexList newBucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pSphereCollider->aabb);
	ST_IndexList bucketsToDeleteSphereFrom = sphereTraceIndexListConstructForDeletedValues(&pSphereCollider->collider.bucketIndices, &newBucketIndices);
	ST_IndexListData* ild = bucketsToDeleteSphereFrom.pFirst;
	for (int i = 0; i < bucketsToDeleteSphereFrom.count; i++)
	{
		if (ild->value != -1)
		{
			sphereTraceIndexListRemoveFirstInstance(&pSimulationSpace->spacialPartitionContainer.buckets[ild->value].sphereColliderIndices, pSphereCollider->collider.colliderIndex);

		}
		else
		{
			sphereTraceIndexListRemoveFirstInstance(&pSimulationSpace->spacialPartitionContainer.outsideBucket.sphereColliderIndices, pSphereCollider->collider.colliderIndex);
		}
		ild = ild->pNext;
	}
	ild = newBucketIndices.pFirst;
	for (int i = 0; i < newBucketIndices.count; i++)
	{
		if (ild->value != -1)
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.buckets[ild->value].sphereColliderIndices, pSphereCollider->collider.colliderIndex);

		}
		else
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.outsideBucket.sphereColliderIndices, pSphereCollider->collider.colliderIndex);
		}
		ild = ild->pNext;
	}
	sphereTraceIndexListFree(&pSphereCollider->collider.bucketIndices);
	sphereTraceIndexListFree(&bucketsToDeleteSphereFrom);
	pSphereCollider->collider.bucketIndices = newBucketIndices;
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
	ST_IndexListData* pild = pSimulationSpace->sphereColliders.pFirst;
	for (int i = 0; i < pSimulationSpace->sphereColliders.count; i++)
	{
		sphereTraceSimulationStepQuantity(pSimulationSpace, &((ST_SphereCollider*)pild->value)->rigidBody, dt);
		pild = pild->pNext;
	}
}

void sphereTraceSimulationSpherePlaneResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_Contact* const pContactInfo, float dt)
{
	ST_SphereCollider* pSphereCollider = sphereTraceColliderSphereGetFromContact(pContactInfo);
	ST_PlaneCollider* pPlaneCollider = sphereTraceColliderPlaneGetFromContact(pContactInfo);
	pSphereCollider->rigidBody.position = sphereTraceVector3Add(pSphereCollider->rigidBody.position, sphereTraceVector3Scale(pContactInfo->normal.v, pContactInfo->penetrationDistance));
	ST_Vector3 r = sphereTraceVector3Subtract(pContactInfo->point, pSphereCollider->rigidBody.position);
	float vnMag = sphereTraceVector3Dot(pContactInfo->normal.v, pSphereCollider->rigidBody.velocity);
	ST_Vector3 vn = sphereTraceVector3Scale(pContactInfo->normal.v, vnMag);
	ST_Vector3 vt = sphereTraceVector3Subtract(pSphereCollider->rigidBody.velocity, vn);
	float j = -(1.0f + pSimulationSpace->eta) * vnMag * pSphereCollider->rigidBody.mass;
	ST_Vector3 dp = sphereTraceVector3Scale(pContactInfo->normal.v, j);
	float slope = sphereTraceVector3Dot(pContactInfo->normal.v, gVector3Up);

	if (fabsf(vnMag) > ST_VELOCITY_THRESHOLD || slope < 0.0f)
	{
		pSphereCollider->restingContact = ST_FALSE;
		sphereTraceRigidBodyAddDeltaMomentum(&pSphereCollider->rigidBody, dp);
		ST_Vector3 ft = sphereTraceVector3Scale(sphereTraceVector3Normalize(vt), -sphereTraceVector3Length(vt) * j / vnMag);
		ST_Vector3 dl = sphereTraceVector3Cross(r, sphereTraceVector3Scale(ft, -pSphereCollider->radius));
		//torque will be nan when ft is in direction of normal
		if (!sphereTraceVector3Nan(dl))
			sphereTraceRigidBodyAddDeltaAngularMomentum(&pSphereCollider->rigidBody, dl);
	}
	else
	{
		if((1.0f-slope)<ST_SPHERE_RESTING_SLOPE)
			pSphereCollider->restingContact = ST_TRUE;
		pSphereCollider->rigidBody.linearMomentum = sphereTraceVector3Cross(sphereTraceVector3Cross(pContactInfo->normal.v, pSphereCollider->rigidBody.linearMomentum), pContactInfo->normal.v);
		sphereTraceRigidBodySetAngularVelocity(&pSphereCollider->rigidBody, sphereTraceVector3Cross(pContactInfo->normal.v, sphereTraceVector3Scale(pSphereCollider->rigidBody.velocity, 1.0f / pSphereCollider->radius)));
		sphereTraceRigidBodyAddForce(&pSphereCollider->rigidBody, sphereTraceVector3Scale(sphereTraceVector3Normalize(sphereTraceVector3Scale(pSphereCollider->rigidBody.velocity, sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->normal.v))), pSimulationSpace->friction));
	}
}

void sphereTraceSimulationSphereTriangleResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_Contact* const pContactInfo, float dt)
{
	ST_SphereCollider* pSphereCollider = sphereTraceColliderSphereGetFromContact(pContactInfo);;
	ST_TriangleCollider* pTriangleCollider = sphereTraceColliderTriangleGetFromContact(pContactInfo);;
	pSphereCollider->rigidBody.position = sphereTraceVector3Add(pSphereCollider->rigidBody.position, sphereTraceVector3Scale(pContactInfo->normal.v, pContactInfo->penetrationDistance));
	ST_Vector3 r = sphereTraceVector3Subtract(pContactInfo->point, pSphereCollider->rigidBody.position);
	float vnMag = sphereTraceVector3Dot(pContactInfo->normal.v, pSphereCollider->rigidBody.velocity);
	ST_Vector3 vn = sphereTraceVector3Scale(pContactInfo->normal.v, vnMag);
	ST_Vector3 vt = sphereTraceVector3Subtract(pSphereCollider->rigidBody.velocity, vn);
	float j = -(1.0f + pSimulationSpace->eta) * vnMag * pSphereCollider->rigidBody.mass;
	ST_Vector3 dp = sphereTraceVector3Scale(pContactInfo->normal.v, j);
	float slope = sphereTraceVector3Dot(pContactInfo->normal.v, gVector3Up);

	if (fabsf(vnMag) > ST_VELOCITY_THRESHOLD || slope < 0.0f)
	{
		pSphereCollider->restingContact = ST_FALSE;
		sphereTraceRigidBodyAddDeltaMomentum(&pSphereCollider->rigidBody, dp);
		ST_Vector3 ft = sphereTraceVector3Scale(sphereTraceVector3Normalize(vt), -sphereTraceVector3Length(vt) * j / vnMag);
		ST_Vector3 dl = sphereTraceVector3Cross(r, sphereTraceVector3Scale(ft, -pSphereCollider->radius));
		//torque will be nan when ft is in direction of normal
		if (!sphereTraceVector3Nan(dl))
			sphereTraceRigidBodyAddDeltaAngularMomentum(&pSphereCollider->rigidBody, dl);
	}
	else
	{
		if ((1.0f - slope) < ST_SPHERE_RESTING_SLOPE)
			pSphereCollider->restingContact = ST_TRUE;
		pSphereCollider->rigidBody.linearMomentum = sphereTraceVector3Cross(sphereTraceVector3Cross(pContactInfo->normal.v, pSphereCollider->rigidBody.linearMomentum), pContactInfo->normal.v);
		sphereTraceRigidBodySetAngularVelocity(&pSphereCollider->rigidBody, sphereTraceVector3Cross(pContactInfo->normal.v, sphereTraceVector3Scale(pSphereCollider->rigidBody.velocity, 1.0f / pSphereCollider->radius)));
		ST_Vector3 f = sphereTraceVector3Scale(sphereTraceVector3Normalize(sphereTraceVector3Scale(pSphereCollider->rigidBody.velocity, sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->normal.v))), pSimulationSpace->friction);
		if (!sphereTraceVector3Nan(f))
			sphereTraceRigidBodyAddForce(&pSphereCollider->rigidBody, f);
	}
}

void sphereTraceSimulationSphereContactResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_Contact* const pContactInfo, float dt)
{
	ST_SphereCollider* pSphereCollider = sphereTraceColliderSphereGetFromContact(pContactInfo);
	pSphereCollider->rigidBody.position = sphereTraceVector3Add(pSphereCollider->rigidBody.position, sphereTraceVector3Scale(pContactInfo->normal.v, pContactInfo->penetrationDistance));
	ST_Vector3 r = sphereTraceVector3Subtract(pContactInfo->point, pSphereCollider->rigidBody.position);
	float vnMag = sphereTraceVector3Dot(pContactInfo->normal.v, pSphereCollider->rigidBody.velocity);
	ST_Vector3 vn = sphereTraceVector3Scale(pContactInfo->normal.v, vnMag);
	ST_Vector3 vt = sphereTraceVector3Subtract(pSphereCollider->rigidBody.velocity, vn);
	float j = -(1.0f + pSimulationSpace->eta) * vnMag * pSphereCollider->rigidBody.mass;
	//float slope = sphereTraceVector3Dot(pContactInfo->normal.v, gVector3Up);
	b32 restingContactCondition = (fabsf(vnMag) < ST_VELOCITY_THRESHOLD);
	if (pContactInfo->collisionType == COLLISION_FACE_INWARD_CIRCULAR)
	{
		ST_Vector3 circularTangent = sphereTraceVector3Cross(pContactInfo->normal.v, pContactInfo->bitangent.v);
		float at = sphereTraceVector3Dot(pSphereCollider->rigidBody.velocity, circularTangent);
		if (fabsf(at) / fabsf(vnMag) > ST_VELOCITY_THRESHOLD)
		{
			at = at * at / pContactInfo->radiusOfCurvature - sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->normal.v);
			if (at < 0.0f)
				restingContactCondition = 0;
			else
				restingContactCondition = 1;
		}
	}
	else if (pContactInfo->collisionType == COLLISION_FACE_INWARD_SPHEREICAL)
	{
		float at = sphereTraceVector3Length(vt);
		if (at / fabsf(vnMag) > ST_VELOCITY_THRESHOLD)
		{
			at = at * at / pContactInfo->radiusOfCurvature - sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->normal.v);
			if (at < 0.0f)
				restingContactCondition = 0;
			else
				restingContactCondition = 1;
		}
		else
			printf("bounce");
	}
	if (!restingContactCondition)
	{
		pSphereCollider->restingContact = ST_FALSE;
		ST_Vector3 dp = sphereTraceVector3Scale(pContactInfo->normal.v, j);
		sphereTraceRigidBodyAddDeltaMomentum(&pSphereCollider->rigidBody, dp);
		ST_Vector3 ft = sphereTraceVector3Scale(sphereTraceVector3Normalize(vt), 
			j*sphereTraceVector3Length(vt)/sphereTraceVector3Length(pSphereCollider->rigidBody.velocity));
		ST_Vector3 dl = sphereTraceVector3Cross(r, sphereTraceVector3Scale(ft, -pSphereCollider->radius));
		if (!sphereTraceVector3NanAny(dl))
			sphereTraceRigidBodyAddDeltaAngularMomentum(&pSphereCollider->rigidBody, dl);
	}
	else
	{
		//if ((1.0f - slope) < ST_SPHERE_RESTING_SLOPE)
		pSphereCollider->restingContact = ST_TRUE;
		pSphereCollider->rigidBody.linearMomentum = sphereTraceVector3Scale(vt, pSphereCollider->rigidBody.mass);

		sphereTraceRigidBodySetAngularVelocity(&pSphereCollider->rigidBody, sphereTraceVector3Cross(pContactInfo->normal.v, sphereTraceVector3Scale(pSphereCollider->rigidBody.velocity, 1.0f / pSphereCollider->radius)));
		ST_Vector3 f = sphereTraceVector3Scale(sphereTraceVector3Scale(sphereTraceVector3Normalize(pSphereCollider->rigidBody.velocity), 
			sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->normal.v)), 
			pSphereCollider->rigidBody.mass*pSimulationSpace->friction*ST_FRICTION_MODIFIER);
		if (!sphereTraceVector3NanAny(f))
			sphereTraceRigidBodyAddForce(&pSphereCollider->rigidBody, f);
	}
}

void sphereTraceSimulationSphereTerrainTriangleResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_Contact* const pContactInfo, float dt)
{
	//ST_SphereCollider* pSphereCollider = pContactInfo->sphereTriangleContactInfo.pSphereCollider;
	//ST_TriangleCollider* pTriangleCollider = pContactInfo->sphereTriangleContactInfo.pTriangleCollider;
	////pSphereCollider->pRigidBody->position = sphereTraceVector3Add(pSphereCollider->pRigidBody->position, sphereTraceVector3Scale(pContactInfo->normal, pContactInfo->penetrationDistance));
	//pSphereCollider->rigidBody.position = pContactInfo->downSphereTraceData.sphereCenter;
	//ST_Vector3 r = sphereTraceVector3Subtract(pContactInfo->downSphereTraceData.rayTraceData.contact.point, pSphereCollider->rigidBody.position);
	//float vnMag = sphereTraceVector3Dot(pContactInfo->downSphereTraceData.rayTraceData.contact.normal.v, pSphereCollider->rigidBody.velocity);
	//ST_Vector3 vn = sphereTraceVector3Scale(pContactInfo->downSphereTraceData.rayTraceData.contact.normal.v, vnMag);
	//ST_Vector3 vt = sphereTraceVector3Subtract(pSphereCollider->rigidBody.velocity, vn);
	//float j = -(1.0f + pSimulationSpace->eta) * vnMag * pSphereCollider->rigidBody.mass;
	//ST_Vector3 dp = sphereTraceVector3Scale(pContactInfo->downSphereTraceData.rayTraceData.contact.normal.v, j);
	//float slope = sphereTraceVector3Dot(pContactInfo->downSphereTraceData.rayTraceData.contact.normal.v, gVector3Up);

	//if (fabsf(vnMag) > ST_VELOCITY_THRESHOLD || slope < 0.0f)
	//{
	//	sphereTraceRigidBodyAddDeltaMomentum(&pSphereCollider->rigidBody, dp);
	//	ST_Vector3 ft = sphereTraceVector3Scale(sphereTraceVector3Normalize(vt), -sphereTraceVector3Length(vt) * j / vnMag);
	//	ST_Vector3 dl = sphereTraceVector3Cross(r, sphereTraceVector3Scale(ft, -pSphereCollider->radius));
	//	//torque will be nan when ft is in direction of normal
	//	if (!sphereTraceVector3Nan(dl))
	//		sphereTraceRigidBodyAddDeltaAngularMomentum(&pSphereCollider->rigidBody, dl);
	//}
	//else
	//{
	//	pSphereCollider->rigidBody.linearMomentum = sphereTraceVector3Cross(sphereTraceVector3Cross(pContactInfo->downSphereTraceData.rayTraceData.contact.normal.v, pSphereCollider->rigidBody.linearMomentum), pContactInfo->downSphereTraceData.rayTraceData.contact.normal.v);
	//	sphereTraceRigidBodySetAngularVelocity(&pSphereCollider->rigidBody, sphereTraceVector3Cross(pContactInfo->downSphereTraceData.rayTraceData.contact.normal.v, sphereTraceVector3Scale(pSphereCollider->rigidBody.velocity, 1.0f / pSphereCollider->radius)));
	//	ST_Vector3 f = sphereTraceVector3Scale(sphereTraceVector3Normalize(sphereTraceVector3Scale(pSphereCollider->rigidBody.velocity, sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->downSphereTraceData.rayTraceData.contact.normal.v))), pSimulationSpace->friction);
	//	if (!sphereTraceVector3Nan(f))
	//		sphereTraceRigidBodyAddForce(&pSphereCollider->rigidBody, f);
	//}
}

void sphereTraceSimulationSphereSphereResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_Contact* const pContactInfo, float dt)
{
	ST_SphereCollider* pA = pContactInfo->contactA;
	ST_SphereCollider* pB = pContactInfo->contactB;
	pA->rigidBody.position = sphereTraceVector3AddAndScale(pA->rigidBody.position, pContactInfo->normal.v, -pContactInfo->penetrationDistance * 0.5f);
	pB->rigidBody.position = sphereTraceVector3AddAndScale(pB->rigidBody.position, pContactInfo->normal.v, pContactInfo->penetrationDistance * 0.5f);
	ST_Vector3 relativeMomentum = sphereTraceVector3Subtract(pB->rigidBody.linearMomentum, pA->rigidBody.linearMomentum);
	ST_Vector3 jdt = sphereTraceVector3Scale(pContactInfo->normal.v, -(1.0f + pSimulationSpace->eta) * sphereTraceVector3Dot(relativeMomentum, pContactInfo->normal.v) / ((1.0f / pA->rigidBody.mass) + (1.0f / pB->rigidBody.mass)));
	sphereTraceRigidBodyAddDeltaMomentum(&pB->rigidBody, jdt);
	sphereTraceRigidBodyAddDeltaMomentum(&pA->rigidBody, sphereTraceVector3Negative(jdt));
}

void sphereTraceSimulationGlobalSolveBruteForce(ST_SimulationSpace* const pSimulationSpace, float dt)
{
	dt = fminf(dt, pSimulationSpace->minDeltaTime);
	if (dt < 0.0f)
		dt = pSimulationSpace->minDeltaTime;
	//update all sphere aabb's 
	ST_IndexListData* pSphereIndexData;
	ST_IndexListData* pOtherSphereIndexData;
	ST_IndexListData* pPlaneIndexData;
	ST_IndexListData* pTerrainIndexData;
	ST_IndexListData* pTriangleIndexData;
	ST_IndexListData* pBowlIndexData;
	ST_IndexListData* pPipeIndexData;
	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
		pSphereIndexData = pSphereIndexData->pNext;
	}

	sphereTraceSimulationStepQuantities(pSimulationSpace, dt);

	////clear the contact infos (if not empty)
	//sphereTraceVectorArrayContactInfoFree(&pSimulationSpace->spherePlaneContactInfos);

	////allocate the contact infos for as many sphere colliders we have
	//pSimulationSpace->spherePlaneContactInfos = sphereTraceVectorArrayContactInfoConstruct(pSimulationSpace->sphereColliders.count, CONTACT_SPHERE_PLANE);

	////clear the contact infos (if not empty)
	//sphereTraceVectorArrayContactInfoFree(&pSimulationSpace->sphereSphereContactInfos);

	////allocate the contact infos for as many sphere colliders we have
	//pSimulationSpace->sphereSphereContactInfos = sphereTraceVectorArrayContactInfoConstruct(pSimulationSpace->sphereColliders.count, CONTACT_SPHERE_SPHERE);

	////clear the contact infos (if not empty)
	//sphereTraceVectorArrayContactInfoFree(&pSimulationSpace->sphereTriangleContactInfos);

	////allocate the contact infos for as many sphere colliders we have
	//pSimulationSpace->sphereTriangleContactInfos = sphereTraceVectorArrayContactInfoConstruct(pSimulationSpace->sphereColliders.count, CONTACT_SPHERE_TRIANGLE);

	////clear the contact infos (if not empty)
	//sphereTraceVectorArrayContactInfoFree(&pSimulationSpace->sphereTerrainTriangleContactInfos);

	////allocate the contact infos for as many sphere colliders we have
	//pSimulationSpace->sphereTerrainTriangleContactInfos = sphereTraceVectorArrayContactInfoConstruct(pSimulationSpace->sphereColliders.count, CONTACT_SPHERE_TERRAIN_TRIANGLE);


	////clear the contact infos (if not empty)
	//sphereTraceVectorArrayContactInfoFree(&pSimulationSpace->sphereBowlContactInfos);

	////allocate the contact infos for as many sphere colliders we have
	//pSimulationSpace->sphereBowlContactInfos = sphereTraceVectorArrayContactInfoConstruct(pSimulationSpace->sphereColliders.count, CONTACT_SPHERE_TERRAIN_TRIANGLE);

	//check for all sphere plane collisions
	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		pPlaneIndexData = pSimulationSpace->planeColliders.pFirst;
		for (int planeColliderIndex = 0; planeColliderIndex < pSimulationSpace->planeColliders.count; planeColliderIndex++)
		{
			ST_PlaneCollider* pPlaneCollider = (ST_PlaneCollider*)pPlaneIndexData->value;
			ST_Contact contactInfo;
			if (sphereTraceColliderPlaneSphereCollisionTest(pPlaneCollider, pSphereCollider, &contactInfo))
			{
				//sphereTraceVectorArrayContactInfoPushBack(&pSimulationSpace->spherePlaneContactInfos, &contactInfo);
				sphereTraceSimulationSpherePlaneResponse(pSimulationSpace, &contactInfo, dt);
			}
			pPlaneIndexData = pPlaneIndexData->pNext;
		}

		pTriangleIndexData = pSimulationSpace->triangleColliders.pFirst;
		for (int triangleColliderIndex = 0; triangleColliderIndex < pSimulationSpace->triangleColliders.count; triangleColliderIndex++)
		{
			ST_TriangleCollider* pTriangleCollider = (ST_TriangleCollider*)pTriangleIndexData->value;
			ST_Contact contactInfo;
			if (sphereTraceColliderTriangleSphereCollisionTest(pTriangleCollider, pSphereCollider, &contactInfo))
			{
				//sphereTraceVectorArrayContactInfoPushBack(&pSimulationSpace->sphereTriangleContactInfos, &contactInfo);
				sphereTraceSimulationSphereTriangleResponse(pSimulationSpace, &contactInfo, dt);
			}
			pTriangleIndexData = pTriangleIndexData->pNext;
		}

		pTerrainIndexData = pSimulationSpace->uniformTerrainColliders.pFirst;
		for (int terrainColliderIndex = 0; terrainColliderIndex < pSimulationSpace->uniformTerrainColliders.count; terrainColliderIndex++)
		{
			ST_UniformTerrainCollider* pTerrainCollider = (ST_UniformTerrainCollider*)pTerrainIndexData->value;
			ST_Contact contactInfo;
			if (sphereTraceColliderUniformTerrainSphereFindMaxPenetratingTriangle(pTerrainCollider, pSphereCollider, &contactInfo))
			{
				//sphereTraceVectorArrayContactInfoPushBack(&pSimulationSpace->sphereTerrainTriangleContactInfos, &contactInfo);
				sphereTraceSimulationSphereTriangleResponse(pSimulationSpace, &contactInfo, dt);
			}
			pTerrainIndexData = pTerrainIndexData->pNext;
		}

		pBowlIndexData = pSimulationSpace->bowlColliders.pFirst;
		for (int bowlColliderIndex = 0; bowlColliderIndex < pSimulationSpace->bowlColliders.count; bowlColliderIndex++)
		{
			ST_BowlCollider* pBowlCollider = (ST_BowlCollider*)pBowlIndexData->value;
			ST_Contact contactInfo;
			if (sphereTraceColliderBowlSphereCollisionTest(pBowlCollider, pSphereCollider, &contactInfo))
			{
				//sphereTraceVectorArrayContactInfoPushBack(&pSimulationSpace->sphereTerrainTriangleContactInfos, &contactInfo);
				sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, dt);
			}
			pBowlIndexData = pBowlIndexData->pNext;
		}

		pPipeIndexData = pSimulationSpace->pipeColliders.pFirst;
		for (int pipeColliderIndex = 0; pipeColliderIndex < pSimulationSpace->pipeColliders.count; pipeColliderIndex++)
		{
			ST_PipeCollider* pPipeCollider = (ST_PipeCollider*)pPipeIndexData->value;
			ST_Contact contactInfo;
			if (sphereTraceColliderPipeSphereCollisionTest(pPipeCollider, pSphereCollider, &contactInfo))
			{
				//sphereTraceVectorArrayContactInfoPushBack(&pSimulationSpace->sphereTerrainTriangleContactInfos, &contactInfo);
				sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, dt);
			}
			pPipeIndexData = pPipeIndexData->pNext;
		}

		//check for all sphere sphere collisions
		pOtherSphereIndexData = pSimulationSpace->sphereColliders.pFirst->pNext;
		for (int sphereColliderBIndex = sphereColliderIndex + 1; sphereColliderBIndex < pSimulationSpace->sphereColliders.count; sphereColliderBIndex++)
		{
			ST_SphereCollider* pSphereColliderB = (ST_SphereCollider*)pOtherSphereIndexData->value;
			ST_Contact contactInfo;
			if (sphereTraceColliderSphereSphereCollisionTest(pSphereCollider, pSphereColliderB, &contactInfo))
			{
				//sphereTraceVectorArrayContactInfoPushBack(&pSimulationSpace->sphereSphereContactInfos, &contactInfo);
				sphereTraceSimulationSphereSphereResponse(pSimulationSpace, &contactInfo, dt);
			}
			pOtherSphereIndexData = pOtherSphereIndexData->pNext;
		}
		pSphereIndexData = pSphereIndexData->pNext;
	}


	////resolve all sphere plane collisions
	//for (int i = 0; i < pSimulationSpace->spherePlaneContactInfos.count; i++)
	//{
	//	ST_Contact* contactInfo = sphereTraceVectorArrayContactInfoGet(&pSimulationSpace->spherePlaneContactInfos, i);
	//	sphereTraceSimulationSpherePlaneResponse(pSimulationSpace, contactInfo, dt);
	//}

	////resolve all sphere plane collisions
	//for (int i = 0; i < pSimulationSpace->sphereTriangleContactInfos.count; i++)
	//{
	//	ST_Contact* contactInfo = sphereTraceVectorArrayContactInfoGet(&pSimulationSpace->sphereTriangleContactInfos, i);
	//	sphereTraceSimulationSphereTriangleResponse(pSimulationSpace, contactInfo, dt);
	//}

	////resolve all sphere triangle collisions
	//for (int i = 0; i < pSimulationSpace->sphereTerrainTriangleContactInfos.count; i++)
	//{
	//	ST_Contact* contactInfo = sphereTraceVectorArrayContactInfoGet(&pSimulationSpace->sphereTerrainTriangleContactInfos, i);
	//	sphereTraceSimulationSphereTriangleResponse(pSimulationSpace, contactInfo, dt);
	//}

	////resolve all sphere plane collisions
	//for (int i = 0; i < pSimulationSpace->sphereSphereContactInfos.count; i++)
	//{
	//	ST_Contact* contactInfo = sphereTraceVectorArrayContactInfoGet(&pSimulationSpace->sphereSphereContactInfos, i);
	//	sphereTraceSimulationSphereSphereResponse(pSimulationSpace, contactInfo, dt);
	//}
}

void sphereTraceSimulationGlobalSolveBruteForceSpacialPartitioned(ST_SimulationSpace* const pSimulationSpace, float dt)
{
	dt = fminf(dt, pSimulationSpace->minDeltaTime);
	if (dt < 0.0f)
		dt = pSimulationSpace->minDeltaTime;
	//update all sphere aabb's 
	ST_IndexListData* pSphereIndexData;
	ST_IndexListData* pOtherIndexData;

	ST_IndexList checkedPlaneColliders = sphereTraceIndexListConstruct();
	ST_IndexList checkedSphereColliders = sphereTraceIndexListConstruct();
	ST_IndexList checkedTerrainColliders = sphereTraceIndexListConstruct();
	ST_IndexList checkedPipeColliders = sphereTraceIndexListConstruct();
	ST_IndexList checkedBowlColliders = sphereTraceIndexListConstruct();

	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
		sphereTraceSimulationUpdateSphereColliderBucketIndices(pSimulationSpace, pSphereCollider);
		pSphereIndexData = pSphereIndexData->pNext;
	}

	sphereTraceSimulationStepQuantities(pSimulationSpace, dt);

	//check for all sphere plane collisions
	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		ST_IndexList bucketsToCheck = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pSphereCollider->aabb);
		ST_IndexListData* pild = bucketsToCheck.pFirst;
		for (int bucketListIndex = 0; bucketListIndex < bucketsToCheck.count; bucketListIndex++)
		{
			ST_SpacialPartitionBucket bucket = sphereTraceSpacialPartitionGetBucketWithIndex(&pSimulationSpace->spacialPartitionContainer, pild->value);

			if (bucket.sphereColliderIndices.count > 0)
			{
				//check for all sphere sphere collisions
				pOtherIndexData = bucket.sphereColliderIndices.pFirst->pNext;
				for (int sphereColliderBIndex = sphereColliderIndex + 1; sphereColliderBIndex < bucket.sphereColliderIndices.count; sphereColliderBIndex++)
				{
					ST_SphereCollider* pSphereColliderB = (ST_SphereCollider*)pOtherIndexData->value;
					if (sphereTraceIndexListAddUnique(&checkedSphereColliders, pSphereColliderB))
					{
						ST_Contact contactInfo;
						if (sphereTraceColliderSphereSphereCollisionTest(pSphereCollider, pSphereColliderB, &contactInfo))
						{
							sphereTraceSimulationSphereSphereResponse(pSimulationSpace, &contactInfo, dt);
						}
					}
					pOtherIndexData = pOtherIndexData->pNext;
				}
			}
			if (bucket.planeColliderIndices.count > 0)
			{
				pOtherIndexData = bucket.planeColliderIndices.pFirst;
				for (int colliderIndex = 0; colliderIndex < bucket.planeColliderIndices.count; colliderIndex++)
				{
					ST_PlaneCollider* pCollider = (ST_PlaneCollider*)pOtherIndexData->value;
					if (sphereTraceIndexListAddUnique(&checkedPlaneColliders, pCollider))
					{
						ST_Contact contactInfo;
						if (sphereTraceColliderPlaneSphereCollisionTest(pCollider, pSphereCollider, &contactInfo))
						{
							sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, dt);
						}
					}
					pOtherIndexData = pOtherIndexData->pNext;
				}
			}
			if (bucket.uniformTerrainColliderIndices.count > 0)
			{
				pOtherIndexData = bucket.uniformTerrainColliderIndices.pFirst;
				for (int colliderIndex = 0; colliderIndex < bucket.uniformTerrainColliderIndices.count; colliderIndex++)
				{
					ST_UniformTerrainCollider* pCollider = (ST_UniformTerrainCollider*)pOtherIndexData->value;
					if (sphereTraceIndexListAddUnique(&checkedTerrainColliders, pCollider))
					{
						ST_Contact contactInfo;
						if (sphereTraceColliderUniformTerrainSphereFindMaxPenetratingTriangle(pCollider, pSphereCollider, &contactInfo))
						{
							sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, dt);
						}
					}
					pOtherIndexData = pOtherIndexData->pNext;
				}
			}
			if (bucket.pipeColliderIndices.count > 0)
			{
				pOtherIndexData = bucket.pipeColliderIndices.pFirst;
				for (int colliderIndex = 0; colliderIndex < bucket.pipeColliderIndices.count; colliderIndex++)
				{
					ST_PipeCollider* pCollider = (ST_PipeCollider*)pOtherIndexData->value;
					if (sphereTraceIndexListAddUnique(&checkedPipeColliders, pCollider))
					{
						ST_Contact contactInfo;
						if (sphereTraceColliderPipeSphereCollisionTest(pCollider, pSphereCollider, &contactInfo))
						{
							sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, dt);
						}
					}
					pOtherIndexData = pOtherIndexData->pNext;
				}
			}
			if (bucket.bowlColliderIndices.count > 0)
			{
				pOtherIndexData = bucket.bowlColliderIndices.pFirst;
				for (int colliderIndex = 0; colliderIndex < bucket.bowlColliderIndices.count; colliderIndex++)
				{
					ST_BowlCollider* pCollider = (ST_BowlCollider*)pOtherIndexData->value;
					if (sphereTraceIndexListAddUnique(&checkedBowlColliders, pCollider))
					{
						ST_Contact contactInfo;
						if (sphereTraceColliderBowlSphereCollisionTest(pCollider, pSphereCollider, &contactInfo))
						{
							sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, dt);
						}
					}
					pOtherIndexData = pOtherIndexData->pNext;
				}
			}
		}
		sphereTraceIndexListFree(&bucketsToCheck);
		sphereTraceIndexListFree(&checkedBowlColliders);
		sphereTraceIndexListFree(&checkedPipeColliders);
		sphereTraceIndexListFree(&checkedTerrainColliders);
		sphereTraceIndexListFree(&checkedPlaneColliders);
		sphereTraceIndexListFree(&checkedSphereColliders);
		pSphereIndexData = pSphereIndexData->pNext;
	}
}

void sphereTraceSimulationGlobalSolveImposedPosition(ST_SimulationSpace* const pSimulationSpace, float dt)
{
	dt = fminf(dt, pSimulationSpace->minDeltaTime);
	if (dt < 0.0f)
		dt = pSimulationSpace->minDeltaTime;
	ST_IndexListData* pSphereIndexData;
	ST_IndexListData* pOtherSphereIndexData;
	ST_IndexListData* pPlaneIndexData;
	ST_IndexListData* pTerrainIndexData;
	ST_IndexListData* pTriangleIndexData;

	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	//update all sphere aabb's 
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
		pSphereIndexData = pSphereIndexData->pNext;
	}

	//check for all sphere plane collisions
	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		float accumulatedDt = 0.0f;
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		//apply all pre existing forces and torques
		sphereTraceSimulationApplyForcesAndTorques(pSimulationSpace, &pSphereCollider->rigidBody, dt, 0);
		while (accumulatedDt < dt)
		{
			//sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);

			ST_SphereCollider* pClosestSphere = NULL;
			ST_PlaneCollider* pClosestPlane = NULL;
			ST_UniformTerrainCollider* pClosestTerrain = NULL;
			float closestCollider = FLT_MAX;
			//float closestRay = FLT_MAX;
			ST_SphereTraceData sphereCastData;
			ST_SphereTraceData sphereCastDataClosestPlane;
			ST_SphereTraceData sphereCastDataClosestSphere;
			ST_SphereTraceData sphereCastDataClosestTerrain;
			//b32 overrideTerrainBehaviour = 0;

			ST_Vector3 imposedNextPosition = sphereTraceSimulationImposedStepPosition(pSimulationSpace, &pSphereCollider->rigidBody, dt - accumulatedDt);
			float imposedNextLength = sphereTraceVector3Length(sphereTraceVector3Subtract(imposedNextPosition, pSphereCollider->rigidBody.position));

			//check for the closest plane
			pPlaneIndexData = pSimulationSpace->planeColliders.pFirst;
			for (int planeColliderIndex = 0; planeColliderIndex < pSimulationSpace->planeColliders.count; planeColliderIndex++)
			{
				ST_PlaneCollider* pPlaneCollider = (ST_PlaneCollider*)pPlaneIndexData->value;
				if (sphereTraceColliderPlaneSphereTrace(pSphereCollider->rigidBody.position, sphereTraceDirectionConstruct(pSphereCollider->rigidBody.velocity, 0), pSphereCollider->radius, pPlaneCollider, &sphereCastData))
				{
					//float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastData.sphereCenter, pSphereCollider->pRigidBody->position));
					if (sphereCastData.traceDistance <= imposedNextLength)
					{
						if (sphereCastData.rayTraceData.distance < closestCollider)
						{
						//if (sphereCastData.traceDistance < closestCollider)
						//{
							pClosestPlane = pPlaneCollider;
							//closestRay = sphereCastData.rayTraceData.distance;
							closestCollider = sphereCastData.rayTraceData.distance;
							sphereCastDataClosestPlane = sphereCastData;
						}
						//}
					}
				}
				pPlaneIndexData = pPlaneIndexData->pNext;
			}

			//check for the closest terrain
			pTerrainIndexData = pSimulationSpace->uniformTerrainColliders.pFirst;
			for (int terrainColliderIndex = 0; terrainColliderIndex < pSimulationSpace->uniformTerrainColliders.count; terrainColliderIndex++)
			{
				ST_UniformTerrainCollider* pTerrainCollider = (ST_UniformTerrainCollider*)pTerrainIndexData->value;
				//if (sphereTraceColliderUniformTerrainSphereTrace(pTerrainCollider, pSphereCollider->pRigidBody->position, pSphereCollider->pRigidBody->velocity, pSphereCollider->radius, &sphereCastData))
				//ST_SphereTerrainTriangleContactInfo contactInfo;
				//if (sphereTraceColliderUniformTerrainSphereTraceByStartEndPoint(pTerrainCollider,pSphereCollider->pRigidBody->position, imposedNextPosition, pSphereCollider->radius, &sphereCastData))
				ST_TriangleCollider* pTriangleCollider = NULL;
				if (sphereTraceColliderUniformTerrainSphereTraceByStartEndPoint(pTerrainCollider, pSphereCollider->rigidBody.position, imposedNextPosition, pSphereCollider->radius, &sphereCastData, &pTriangleCollider))
				{
					if (sphereCastData.traceDistance <= imposedNextLength)
					{
						if (sphereCastData.rayTraceData.distance < closestCollider)
						{
						////if (sphereCastData.rayTraceData.distance <= closestRay)
						////{
						//	pClosestTerrain = pTerrainCollider;
						//	closestRay = sphereCastData.rayTraceData.distance;
						//	closestCollider = sphereCastData.traceDistance;
						//	sphereCastDataClosestTerrain = sphereCastData;
						//	pClosestPlane = NULL;
						//}
						//else 
						//if (sphereCastData.traceDistance == closestCollider)
						//{
							pClosestTerrain = pTerrainCollider;
							//closestRay = sphereCastData.rayTraceData.distance;
							closestCollider = sphereCastData.rayTraceData.distance;
							sphereCastDataClosestTerrain = sphereCastData;
							pClosestPlane = NULL;
							//overrideTerrainBehaviour = 1;
						}
						//}
					}
				}
				pTerrainIndexData = pTerrainIndexData->pNext;
			}
			//check for all sphere sphere collisions
			pOtherSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
			for (int sphereColliderOtherIndex = sphereColliderIndex + 1; sphereColliderOtherIndex < pSimulationSpace->sphereColliders.count; sphereColliderOtherIndex++)
			{
				ST_SphereCollider* pSphereColliderOther = (ST_SphereCollider*)pOtherSphereIndexData->value;
				if (sphereTraceColliderSphereSphereTrace(pSphereCollider->rigidBody.position, sphereTraceDirectionConstructNormalized(pSphereCollider->rigidBody.velocity), pSphereCollider->radius, pSphereColliderOther, &sphereCastData))
				{
					//ST_Vector3 nextPos = 
					//float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastData.sphereCenter, pSphereCollider->pRigidBody->position));
					if (sphereCastData.traceDistance <= imposedNextLength)
					{
						if (sphereCastData.rayTraceData.distance < closestCollider)
						{
						//if (sphereCastData.traceDistance < closestCollider)
						//{
							pClosestSphere = pSphereColliderOther;
							pClosestPlane = NULL;
							pClosestTerrain = NULL;
							//closestRay = sphereCastData.rayTraceData.distance;
							closestCollider = sphereCastData.rayTraceData.distance;
							sphereCastDataClosestSphere = sphereCastData;
							//}
						}
					}
				}
				pOtherSphereIndexData = pOtherSphereIndexData->pNext;
			}
			//sphereTraceColliderUniformTerrainSphereTraceByStartEndPoint()
			if (closestCollider != FLT_MAX)
			{
				if (pClosestPlane != NULL)
				{
					//float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastDataClosestPlane.sphereCenter, pSphereCollider->pRigidBody->position));
					float sphereCastDistance = sphereCastDataClosestPlane.traceDistance;
					//float test = pSphereCollider->radius - sphereCastDataClosestPlane.rayTraceData.distance;
					//if(test>0)
					//printf("test: %f\n", test);
					float adjustedDt = fminf((dt - accumulatedDt) * sphereCastDistance / imposedNextLength + ST_AUTO_DT_FACTOR * dt, dt - accumulatedDt);

					//step the simulation
					sphereTraceSimulationStepQuantity(pSimulationSpace, &pSphereCollider->rigidBody, adjustedDt);
					//sphereTraceVector3Print(pSphereCollider->pRigidBody->linearMomentum);

					//resolve the collisions if any
					ST_Contact contactInfo;
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
					//float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastDataClosestSphere.sphereCenter, pSphereCollider->pRigidBody->position));
					float sphereCastDistance = sphereCastDataClosestSphere.traceDistance;
					float adjustedDt = fminf((dt - accumulatedDt) * sphereCastDistance / imposedNextLength + ST_AUTO_DT_FACTOR * dt, dt - accumulatedDt);

					//step the simulation
					sphereTraceSimulationStepQuantity(pSimulationSpace, &pSphereCollider->rigidBody, adjustedDt);

					ST_Contact contactInfo;
					if (sphereTraceColliderSphereSphereCollisionTest(pSphereCollider, pClosestSphere, &contactInfo))
					{
						sphereTraceSimulationSphereSphereResponse(pSimulationSpace, &contactInfo, adjustedDt);
					}
					accumulatedDt += adjustedDt;
				}
				else if (pClosestTerrain != NULL)
				{
					//float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastDataClosestTerrain.sphereCenter, pSphereCollider->pRigidBody->position));
					float sphereCastDistance = sphereCastDataClosestTerrain.traceDistance;
					float adjustedDt = fminf((dt - accumulatedDt) * sphereCastDistance / imposedNextLength + ST_AUTO_DT_FACTOR * dt, dt - accumulatedDt);

					//step the simulation
					sphereTraceSimulationStepQuantity(pSimulationSpace, &pSphereCollider->rigidBody, adjustedDt);

					ST_Contact contactInfo;
					if (sphereTraceColliderUniformTerrainSphereFindMaxPenetratingTriangle(pClosestTerrain, pSphereCollider, &contactInfo))
					{
						//if (!overrideTerrainBehaviour)
						//	sphereTraceSimulationSphereTerrainTriangleResponse(pSimulationSpace, &contactInfo, adjustedDt);
						//else
							sphereTraceSimulationSphereTriangleResponse(pSimulationSpace, &contactInfo, adjustedDt);
					}
					accumulatedDt += adjustedDt;
				}
			}
			else
			{
				float adjustedDt = dt - accumulatedDt;
				sphereTraceSimulationStepQuantity(pSimulationSpace, &pSphereCollider->rigidBody, adjustedDt);
				accumulatedDt += adjustedDt;
			}
		}
		pSphereIndexData = pSphereIndexData->pNext;
	}
}

b32 sphereTraceSimulationRayTrace(const ST_SimulationSpace* const pSimulationSpace, ST_Vector3 start, ST_Direction dir, ST_RayTraceData* const pRayCastData)
{
	int currentBucketIndex = 0;
	ST_Vector3 intersection = start;
	sphereTraceDirectionNormalizeIfNotNormalizedByRef(&dir);
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
		ST_IndexListData* pild = bucket.planeColliderIndices.pFirst;
		for (int i = 0; i < bucket.planeColliderIndices.count; i++)
		{
			if (sphereTraceColliderPlaneRayTrace(start, dir, (ST_PlaneCollider*)pild->value, &tentativeData))
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
			if (sphereTraceColliderSphereRayTrace(start, dir, (ST_SphereCollider*)pild->value, &tentativeData))
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
		intersection = sphereTraceSpacialPartitionStaticGetNearestBucketIntersectionFromPositionAndDirection(&bucket, intersection, dir.v, &incomingDir);
		ST_Vector3 checkPoint = sphereTraceVector3AddAndScale(intersection, incomingDir, pSimulationSpace->spacialPartitionContainer.partitionSize * 0.5f);
		currentBucketIndex = sphereTraceSpacialPartitionStaticGetBucketIndexFromPosition(&pSimulationSpace->spacialPartitionContainer, checkPoint);
	}
	//check outside
	b32 hit = 0;
	bucket = pSimulationSpace->spacialPartitionContainer.outsideBucket;
	ST_IndexListData* pild = bucket.planeColliderIndices.pFirst;
	for (int i = 0; i < bucket.planeColliderIndices.count; i++)
	{
		if (sphereTraceColliderPlaneRayTrace(start, dir, (ST_PlaneCollider*)pild->value, &tentativeData))
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
		if (sphereTraceColliderSphereRayTrace(start, dir, (ST_SphereCollider*)pild->value, &tentativeData))
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
	//dt = fminf(dt, pSimulationSpace->minDeltaTime);
	//if (dt < 0.0f)
	//	dt = pSimulationSpace->minDeltaTime;
	ST_IndexListData* pSphereIndexData;
	//update all sphere aabb's and buckets
	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
		sphereTraceSimulationUpdateSphereColliderBucketIndices(pSimulationSpace, pSphereCollider);
		pSphereIndexData = pSphereIndexData->pNext;
	}

	//check for all sphere plane collisions
	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		float accumulatedDt = 0.0f;
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		if (pSphereCollider->rigidBody.isAsleep)
		{
			pSphereIndexData = pSphereIndexData->pNext;
			continue;
		}
		ST_AABB imposedSpherePathAABB;
		ST_SphereTraceData imposedPathSphereCastData;
		//apply all pre existing forces and torques
		sphereTraceSimulationApplyForcesAndTorques(pSimulationSpace, &pSphereCollider->rigidBody, dt, 0);
		//ST_SphereCollider* pClosestSphere = NULL;
		//ST_PlaneCollider* pClosestPlane = NULL;
		//ST_UniformTerrainCollider* pClosestTerrain = NULL;
		ST_TriangleCollider* pClosestTerrainTriangle = NULL;
		ST_Collider* pClosestCollider = NULL;
		ST_IndexList touchedColliders = sphereTraceIndexListConstruct();
		b32 closestPlaneSame = ST_FALSE;
		ST_Index substepCount = 0;
		while (accumulatedDt < dt)
		{

			if (!pSphereCollider->ignoreCollisions)
			{
				float closestCollider = FLT_MAX;
				ST_SphereTraceData sphereCastData;
				ST_SphereTraceData sphereCastClosestData;
				//ST_SphereTraceData sphereCastDataClosestPlane;
				//ST_SphereTraceData sphereCastDataClosestSphere;
				//ST_SphereTraceData sphereCastDataClosestTerrain;

				imposedPathSphereCastData.radius = pSphereCollider->radius;
				imposedPathSphereCastData.rayTraceData.startPoint = pSphereCollider->rigidBody.position;
				ST_Vector3 imposedNextPosition = sphereTraceSimulationImposedStepPosition(pSimulationSpace, &pSphereCollider->rigidBody, dt - accumulatedDt);
				imposedPathSphereCastData.sphereCenter = imposedNextPosition;
				sphereTraceColliderResizeAABBWithSpherecast(&imposedPathSphereCastData, &imposedSpherePathAABB);
				float imposedNextLength = sphereTraceVector3Length(sphereTraceVector3Subtract(imposedNextPosition, pSphereCollider->rigidBody.position));

				ST_IndexList bucketsToCheck = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &imposedSpherePathAABB);

				ST_IndexListData* pild = bucketsToCheck.pFirst;
				ST_IndexList checkedPlaneColliders = sphereTraceIndexListConstruct();
				ST_IndexList checkedSphereColliders = sphereTraceIndexListConstruct();
				ST_IndexList checkedTerrainColliders = sphereTraceIndexListConstruct();
				ST_Direction vDir = sphereTraceDirectionConstructNormalized(pSphereCollider->rigidBody.velocity);
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
						ST_IndexListData* pPlaneColliderILD = bucket.planeColliderIndices.pFirst;
						//check for the closest plane
						for (int i = 0; i < bucket.planeColliderIndices.count; i++)
						{
							ST_PlaneCollider* pPlaneCollider = (ST_PlaneCollider*)pPlaneColliderILD->value;
							if (sphereTraceIndexListAddUnique(&checkedPlaneColliders, pPlaneCollider->collider.colliderIndex))
							{
								if (sphereTraceColliderPlaneSphereTrace(pSphereCollider->rigidBody.position, vDir, pSphereCollider->radius, pPlaneCollider, &sphereCastData))
								{
									//float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastData.sphereCenter, pSphereCollider->pRigidBody->position));
									if (sphereCastData.traceDistance <= imposedNextLength)
									{
										if (sphereCastData.rayTraceData.distance < closestCollider)
										{
											if(!sphereTraceIndexListAddUnique(&touchedColliders, pPlaneCollider))
												closestPlaneSame = ST_TRUE;
											pClosestCollider = pPlaneCollider;
											closestCollider = sphereCastData.rayTraceData.distance;
											sphereCastClosestData = sphereCastData;
										}
									}
								}
							}
							pPlaneColliderILD = pPlaneColliderILD->pNext;
						}
					}
					if (bucket.sphereColliderIndices.count > 0)
					{
						ST_IndexListData* pSphereColliderILD = bucket.sphereColliderIndices.pFirst;
						//check for all sphere sphere collisions
						for (int i = 0; i < bucket.sphereColliderIndices.count; i++)
						{
							if (pSphereColliderILD->value != pSphereIndexData->value)
							{
								ST_SphereCollider* pSphereColliderOther = (ST_SphereCollider*)pSphereColliderILD->value;
								if (sphereTraceIndexListAddUnique(&checkedSphereColliders, pSphereColliderOther->collider.colliderIndex))
								{
									if (sphereTraceColliderAABBIntersectAABB(&imposedSpherePathAABB, &pSphereColliderOther->aabb))
									{
										if (sphereTraceColliderSphereSphereTrace(pSphereCollider->rigidBody.position, vDir, pSphereCollider->radius, pSphereColliderOther, &sphereCastData))
										{
											//float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastData.sphereCenter, pSphereCollider->pRigidBody->position));
											if (sphereCastData.traceDistance <= imposedNextLength)
											{
												if (sphereCastData.rayTraceData.distance < closestCollider)
												{
													if (!sphereTraceIndexListAddUnique(&touchedColliders, pSphereColliderOther))
														closestPlaneSame = ST_TRUE;
													pClosestCollider = pSphereColliderOther;
													closestCollider = sphereCastData.rayTraceData.distance;
													sphereCastClosestData = sphereCastData;
												}
											}
										}
									}
								}
							}
							pSphereColliderILD = pSphereColliderILD->pNext;
						}
					}
					if (bucket.uniformTerrainColliderIndices.count > 0)
					{
						ST_IndexListData* pTerrainColliderILD = bucket.uniformTerrainColliderIndices.pFirst;
						//check for all sphere sphere collisions
						for (int i = 0; i < bucket.uniformTerrainColliderIndices.count; i++)
						{
							ST_UniformTerrainCollider* pUniformTerrainCollider = (ST_UniformTerrainCollider*)pTerrainColliderILD->value;
							if (sphereTraceIndexListAddUnique(&checkedTerrainColliders, pUniformTerrainCollider->collider.colliderIndex))
							{
								if (sphereTraceColliderUniformTerrainSphereTraceByStartEndPoint(pUniformTerrainCollider, pSphereCollider->rigidBody.position, imposedNextPosition, pSphereCollider->radius, &sphereCastData, &pClosestTerrainTriangle))
								{
									//float sphereCastDistance = sphereTraceVector3Length(sphereTraceVector3Subtract(sphereCastData.sphereCenter, pSphereCollider->pRigidBody->position));
									if (sphereCastData.traceDistance <= imposedNextLength)
									{
										if (sphereCastData.rayTraceData.distance < closestCollider)
										{
											if (!sphereTraceIndexListAddUnique(&touchedColliders, pUniformTerrainCollider))
												closestPlaneSame = ST_TRUE;
											pClosestCollider = pUniformTerrainCollider;
											closestCollider = sphereCastData.rayTraceData.distance;
											sphereCastClosestData = sphereCastData;
										}
									}
								}
							}
							pTerrainColliderILD = pTerrainColliderILD->pNext;
						}
					}
					pild = pild->pNext;
				}
				sphereTraceIndexListFree(&checkedPlaneColliders);
				sphereTraceIndexListFree(&checkedSphereColliders);
				sphereTraceIndexListFree(&checkedTerrainColliders);
				sphereTraceIndexListFree(&bucketsToCheck);
				if (closestCollider != FLT_MAX && !closestPlaneSame)
				{
					float sphereCastDistance = sphereCastClosestData.traceDistance;
					float adjustedDt = fminf((dt - accumulatedDt) * sphereCastDistance / imposedNextLength + ST_AUTO_DT_FACTOR * dt, dt - accumulatedDt);
					if (closestPlaneSame)
					{
						adjustedDt = dt - accumulatedDt;
					}
					//step the simulation
					sphereTraceSimulationStepQuantity(pSimulationSpace, &pSphereCollider->rigidBody, adjustedDt);

					//resolve the collisions if any
					ST_Contact contactInfo;
					switch (pClosestCollider->colliderType)
					{
					case COLLIDER_PLANE:
						if (sphereTraceColliderPlaneSphereCollisionTest(pClosestCollider, pSphereCollider, &contactInfo))
						{
							sphereTraceSimulationSpherePlaneResponse(pSimulationSpace, &contactInfo, adjustedDt);
						}
						break;
					case COLLIDER_SPHERE:
						if (sphereTraceColliderSphereSphereCollisionTest(pSphereCollider, pClosestCollider, &contactInfo))
						{
							sphereTraceSimulationSphereSphereResponse(pSimulationSpace, &contactInfo, adjustedDt);
						}
						break;
					case COLLIDER_TERRAIN:
						if (sphereTraceColliderTriangleSphereCollisionTest(pClosestTerrainTriangle, pSphereCollider, &contactInfo))
						{
							sphereTraceSimulationSphereTriangleResponse(pSimulationSpace, &contactInfo, adjustedDt);
						}
						break;
					}

					accumulatedDt += adjustedDt;
				}
				else
				{
					float adjustedDt = dt - accumulatedDt;
					sphereTraceSimulationStepQuantity(pSimulationSpace,&pSphereCollider->rigidBody, adjustedDt);
					accumulatedDt += adjustedDt;


					//if (closestPlaneSame && sphereColliderIndex == pSimulationSpace->sphereColliders.count-1)
					//	printf("closest planes: %i\n", touchedColliders.count);
				}
			}
			else
			{
				float adjustedDt = dt - accumulatedDt;
				sphereTraceSimulationStepQuantity(pSimulationSpace, &pSphereCollider->rigidBody, adjustedDt);
				accumulatedDt += adjustedDt;
			}

			sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
			sphereTraceSimulationUpdateSphereColliderBucketIndices(pSimulationSpace, pSphereCollider);
			substepCount++;
		}
		float s2 = sphereTraceVector3Length2(pSphereCollider->rigidBody.velocity);
		if ((s2 < ST_RESTING_SPEED_SQUARED) && pSphereCollider->restingContact)
			pSphereCollider->rigidBody.isAsleep = 1;
		//if (sphereColliderIndex == 0)
		//	printf("substep count: %i\n", substepCount);
		pSphereIndexData = pSphereIndexData->pNext;
		sphereTraceIndexListFree(&touchedColliders);
	}
}