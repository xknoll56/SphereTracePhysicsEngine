
#include "SphereTrace.h"
#include "SphereTraceGlobals.h"

void sphereTraceSubscriberListAddOnCollisionEnterCallback(ST_SimulationSpace* pSimSpace, ST_Collider* pCollider, 
	void (*callback)(const ST_SphereContact* const contact, ST_Collider* pOtherCollider, void* pContext), void* pContext)
{
	ST_CallbackFunction* pcf = sphereTraceAllocatorAllocateCallbackFunction();
	pcf->callback = callback;
	sphereTraceIndexListAddFirst(&pCollider->subscriberList.onCollisionEnterCallbacks, pcf);
	pCollider->subscriberList.hasSubscriber = 1;
	sphereTraceIndexListAddUnique(&pSimSpace->callbackColliders, pCollider);
	pCollider->subscriberList.pSubscriberContext = pContext;
}
void sphereTraceSubscriberListAddOnCollisionStayCallback(ST_SimulationSpace* pSimSpace, ST_Collider* pCollider, 
	void (*callback)(const ST_SphereContact* const contact, ST_Collider* pOtherCollider, void* pContext), void* pContext)
{
	ST_CallbackFunction* pcf = sphereTraceAllocatorAllocateCallbackFunction();
	pcf->callback = callback;
	sphereTraceIndexListAddFirst(&pCollider->subscriberList.onCollisionStayCallbacks, pcf);
	pCollider->subscriberList.hasSubscriber = 1;
	sphereTraceIndexListAddUnique(&pSimSpace->callbackColliders, pCollider);
	pCollider->subscriberList.pSubscriberContext = pContext;
}
void sphereTraceSubscriberListAddOnCollisionExitCallback(ST_SimulationSpace* pSimSpace, ST_Collider* pCollider, 
	void (*callback)(const ST_SphereContact* const contact, ST_Collider* pOtherCollider, void* pContext), void* pContext)
{
	ST_CallbackFunction* pcf = sphereTraceAllocatorAllocateCallbackFunction();
	pcf->callback = callback;
	sphereTraceIndexListAddFirst(&pCollider->subscriberList.onCollisionExitCallbacks, pcf);
	pCollider->subscriberList.hasSubscriber = 1;
	sphereTraceIndexListAddUnique(&pSimSpace->callbackColliders, pCollider);
	pCollider->subscriberList.pSubscriberContext = pContext;
}

ST_SimulationSpace sphereTraceSimulationConstruct()
{
	ST_SimulationSpace simulationSpace;
	simulationSpace.sphereColliders = sphereTraceIndexListConstruct();
	simulationSpace.planeColliders = sphereTraceIndexListConstruct();
	simulationSpace.triangleColliders = sphereTraceIndexListConstruct();
	simulationSpace.uniformTerrainColliders = sphereTraceIndexListConstruct();
	simulationSpace.callbackColliders = sphereTraceIndexListConstruct();
	simulationSpace.spacialPartitionContainer = sphereTraceSpacialPartitionStaticConstruct(20.0f);
	simulationSpace.minDeltaTime = 1.0f / 60.0f;
	simulationSpace.gravitationalAcceleration = sphereTraceVector3Construct(0.0f, -9.81f, 0.0f);
	simulationSpace.defaultMaterial = sphereTraceMaterialConstruct(0.2f, 0.8f, 0.2f);
	simulationSpace.worldAABB = gAABBOne;
	return simulationSpace;
}

void sphereTraceSimulationFree(ST_SimulationSpace* const pSimulationSpace)
{
	sphereTraceIndexListFree(&pSimulationSpace->sphereColliders);
	sphereTraceIndexListFree(&pSimulationSpace->planeColliders);
	sphereTraceIndexListFree(&pSimulationSpace->triangleColliders);
	sphereTraceIndexListFree(&pSimulationSpace->callbackColliders);
	ST_IndexListData* pild = pSimulationSpace->uniformTerrainColliders.pFirst;
	ST_UniformTerrainCollider* pTerrainCollider = NULL;
	for (int i = 0; i < pSimulationSpace->uniformTerrainColliders.count; i++)
	{
		pTerrainCollider = pild->value;
		sphereTraceColliderUniformTerrainFree(pTerrainCollider);
		pild = pild->pNext;
	}
}

void sphereTraceSimulationInsertPlaneCollider(ST_SimulationSpace* const pSimulationSpace, ST_PlaneCollider* const pPlaneCollider)
{
	//ST_ColliderIndex index = pSimulationSpace->planeColliders.count;
	pPlaneCollider->collider.colliderIndex = pPlaneCollider;
	sphereTraceIndexListAddFirst(&pSimulationSpace->planeColliders, pPlaneCollider);
	pPlaneCollider->collider.bucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pPlaneCollider->collider.aabb);
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
	sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&pSimulationSpace->worldAABB, &pPlaneCollider->collider.aabb);
}

void sphereTraceSimulationInsertTriangleCollider(ST_SimulationSpace* const pSimulationSpace, ST_TriangleCollider* const pTriangleCollider)
{
	//ST_ColliderIndex index = pSimulationSpace->planeColliders.count;
	pTriangleCollider->collider.colliderIndex = pTriangleCollider;
	sphereTraceIndexListAddFirst(&pSimulationSpace->triangleColliders, pTriangleCollider);
	pTriangleCollider->collider.bucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pTriangleCollider->collider.aabb);
	ST_IndexListData* ild = pTriangleCollider->collider.bucketIndices.pFirst;
	for (int i = 0; i < pTriangleCollider->collider.bucketIndices.count; i++)
	{
		if (ild->value != -1)
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.buckets[ild->value].triangleColliderIndices, pTriangleCollider);

		}
		else
		{
			sphereTraceIndexListAddUnique(&pSimulationSpace->spacialPartitionContainer.outsideBucket.triangleColliderIndices, pTriangleCollider);
		}
		ild = ild->pNext;
	}
	sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&pSimulationSpace->worldAABB, &pTriangleCollider->collider.aabb);
}

void sphereTraceSimulationInsertSphereCollider(ST_SimulationSpace* const pSimulationSpace, ST_SphereCollider* const pSphereCollider)
{
	//ST_ColliderIndex index = pSimulationSpace->sphereColliders.count;
	pSphereCollider->collider.colliderIndex = pSphereCollider;
	sphereTraceIndexListAddFirst(&pSimulationSpace->sphereColliders, pSphereCollider);
	pSphereCollider->collider.bucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pSphereCollider->collider.aabb);
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
	sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&pSimulationSpace->worldAABB, &pSphereCollider->collider.aabb);
}

void sphereTraceSimulationInsertUniformTerrainCollider(ST_SimulationSpace* const pSimulationSpace, ST_UniformTerrainCollider* const pTerrainCollider)
{
	//ST_ColliderIndex index = pSimulationSpace->uniformTerrainColliders.count;
	pTerrainCollider->collider.colliderIndex = pTerrainCollider;
	sphereTraceIndexListAddFirst(&pSimulationSpace->uniformTerrainColliders, pTerrainCollider);
	pTerrainCollider->collider.bucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pTerrainCollider->collider.aabb);
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
	sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&pSimulationSpace->worldAABB, &pTerrainCollider->collider.aabb);
}

void sphereTraceSimulationConstructOctTree(ST_SimulationSpace* const pSimulationSpace)
{
	//this assumes all of the static geometry has been inserted and the world
	//aabb has already been constructed
	pSimulationSpace->octTree = sphereTraceOctTreeConstruct(pSimulationSpace->worldAABB);

	ST_IndexListData* pild;

	pild = pSimulationSpace->sphereColliders.pFirst;
	for (int i = 0; i < pSimulationSpace->sphereColliders.count; i++)
	{
		sphereTraceOctTreeInsertCollider(&pSimulationSpace->octTree, pild->value, ST_TRUE);
		pild = pild->pNext;
	}

	pild = pSimulationSpace->triangleColliders.pFirst;
	for (int i = 0; i < pSimulationSpace->triangleColliders.count; i++)
	{
		sphereTraceOctTreeInsertCollider(&pSimulationSpace->octTree, pild->value, ST_TRUE);
		pild = pild->pNext;
	}

	pild = pSimulationSpace->planeColliders.pFirst;
	for (int i = 0; i < pSimulationSpace->planeColliders.count; i++)
	{
		sphereTraceOctTreeInsertCollider(&pSimulationSpace->octTree, pild->value, ST_TRUE);
		pild = pild->pNext;
	}

	pild = pSimulationSpace->uniformTerrainColliders.pFirst;
	for (int i = 0; i < pSimulationSpace->uniformTerrainColliders.count; i++)
	{
		sphereTraceOctTreeInsertCollider(&pSimulationSpace->octTree, pild->value, ST_TRUE);
		pild = pild->pNext;
	}
}

void sphereTraceSimulationUpdateSphereColliderBucketIndices(ST_SimulationSpace* const pSimulationSpace, ST_SphereCollider* const pSphereCollider)
{
	ST_IndexList newBucketIndices = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pSphereCollider->collider.aabb);
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
	pRigidBody->velocity = sphereTraceVector3Scale(pRigidBody->linearMomentum, 1.0f/pRigidBody->mass);


	sphereTraceRigidBodyApplyTorques(pRigidBody, dt);
	sphereTraceRigidBodyApplyDeltaAngularMomentums(pRigidBody);
	//set angular velocity
	pRigidBody->angularVelocity = sphereTraceVector3Scale(pRigidBody->angularMomentum, 1.0f/pRigidBody->inertia);
}

void sphereTraceSimulationStepQuantity(const ST_SimulationSpace* const pSimulationSpace, ST_RigidBody* const pRigidBody, float dt)
{
	sphereTraceSimulationApplyForcesAndTorques(pSimulationSpace, pRigidBody, dt, 1);

	pRigidBody->rotation = sphereTraceQuaternionAdd(pRigidBody->rotation, sphereTraceQuaternionMultiply(sphereTraceQuaternionConstruct(0.0f, 0.5f * dt * pRigidBody->angularVelocity.x, 0.5f * dt * pRigidBody->angularVelocity.y, 0.5f * dt * pRigidBody->angularVelocity.z), pRigidBody->rotation));
	pRigidBody->rotation = sphereTraceQuaternionNormalize(pRigidBody->rotation);
	pRigidBody->rotationMatrix = sphereTraceMatrixFromQuaternion(pRigidBody->rotation);

	pRigidBody->prevPosition = pRigidBody->position;
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
	ST_Vector3 imposedVelocity = sphereTraceVector3Scale(imposedLinearMomentum, 1.0f/pRigidBody->mass);

	//return the imposed next position
	return sphereTraceVector3Add(pRigidBody->position, sphereTraceVector3Scale(imposedVelocity, dt));
}



void sphereTraceSimulationStepQuantities(const ST_SimulationSpace* const pSimulationSpace, float dt)
{
	ST_IndexListData* pild = pSimulationSpace->sphereColliders.pFirst;
	ST_SphereCollider* pSphereCollider;
	for (int i = 0; i < pSimulationSpace->sphereColliders.count; i++)
	{
		pSphereCollider = pild->value;
		sphereTraceSimulationStepQuantity(pSimulationSpace, &pSphereCollider->rigidBody, dt);
		pild = pild->pNext;
	}
}

void sphereTraceSimulationSpherePlaneResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_SphereContact* const pContactInfo, float dt)
{
	ST_SphereCollider* pSphereCollider = sphereTraceColliderSphereGetFromContact(pContactInfo);
	ST_PlaneCollider* pPlaneCollider = sphereTraceColliderPlaneGetFromContact(pContactInfo);
	pSphereCollider->rigidBody.position = sphereTraceVector3Add(pSphereCollider->rigidBody.position, sphereTraceVector3Scale(pContactInfo->normal.v, pContactInfo->penetrationDistance));
	ST_Vector3 r = sphereTraceVector3Subtract(pContactInfo->point, pSphereCollider->rigidBody.position);
	float vnMag = sphereTraceVector3Dot(pContactInfo->normal.v, pSphereCollider->rigidBody.velocity);
	ST_Vector3 vn = sphereTraceVector3Scale(pContactInfo->normal.v, vnMag);
	ST_Vector3 vt = sphereTraceVector3Subtract(pSphereCollider->rigidBody.velocity, vn);
	float j = -(1.0f + pSimulationSpace->defaultMaterial.restitution) * vnMag * pSphereCollider->rigidBody.mass;
	ST_Vector3 dp = sphereTraceVector3Scale(pContactInfo->normal.v, j);
	float slope = sphereTraceVector3Dot(pContactInfo->normal.v, gVector3Up);

	if (sphereTraceAbs(vnMag) > ST_VELOCITY_THRESHOLD || slope < 0.0f)
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
		sphereTraceRigidBodyAddForce(&pSphereCollider->rigidBody, sphereTraceVector3Scale(sphereTraceVector3Normalize(sphereTraceVector3Scale(pSphereCollider->rigidBody.velocity, 
			sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->normal.v))), pSimulationSpace->defaultMaterial.staticFriction));
	}
}

void sphereTraceSimulationSphereTriangleResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_SphereContact* const pContactInfo, float dt)
{
	ST_SphereCollider* pSphereCollider = sphereTraceColliderSphereGetFromContact(pContactInfo);;
	ST_TriangleCollider* pTriangleCollider = sphereTraceColliderTriangleGetFromContact(pContactInfo);;
	pSphereCollider->rigidBody.position = sphereTraceVector3Add(pSphereCollider->rigidBody.position, sphereTraceVector3Scale(pContactInfo->normal.v, pContactInfo->penetrationDistance));
	ST_Vector3 r = sphereTraceVector3Subtract(pContactInfo->point, pSphereCollider->rigidBody.position);
	float vnMag = sphereTraceVector3Dot(pContactInfo->normal.v, pSphereCollider->rigidBody.velocity);
	ST_Vector3 vn = sphereTraceVector3Scale(pContactInfo->normal.v, vnMag);
	ST_Vector3 vt = sphereTraceVector3Subtract(pSphereCollider->rigidBody.velocity, vn);
	float j = -(1.0f + pSimulationSpace->defaultMaterial.restitution) * vnMag * pSphereCollider->rigidBody.mass;
	ST_Vector3 dp = sphereTraceVector3Scale(pContactInfo->normal.v, j);
	float slope = sphereTraceVector3Dot(pContactInfo->normal.v, gVector3Up);

	if (sphereTraceAbs(vnMag) > ST_VELOCITY_THRESHOLD || slope < 0.0f)
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
		ST_Vector3 f = sphereTraceVector3Scale(sphereTraceVector3Normalize(sphereTraceVector3Scale(pSphereCollider->rigidBody.velocity, 
			sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->normal.v))), pSimulationSpace->defaultMaterial.staticFriction);
		if (!sphereTraceVector3Nan(f))
			sphereTraceRigidBodyAddForce(&pSphereCollider->rigidBody, f);
	}
}

b32 sphereTraceSimulationSphereContactIsRestingCheck(const ST_SphereContact* const pContactInfo)
{
	ST_SphereCollider* pSphereCollider = sphereTraceColliderSphereGetFromContact(pContactInfo);
	float vnMag = sphereTraceVector3Dot(pContactInfo->normal.v, pSphereCollider->rigidBody.velocity);
	return sphereTraceAbs(vnMag) < ST_VELOCITY_THRESHOLD;
}

void sphereTraceSimulationSphereContactResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_SphereContact* const pContactInfo, float dt, ST_Index* pNumImpulses)
{
	//ST_SphereCollider* pSphereCollider = sphereTraceColliderSphereGetFromContact(pContactInfo);
	ST_SphereCollider* pSphereCollider = pContactInfo->pSphereCollider;
	pSphereCollider->rigidBody.position = sphereTraceVector3Add(pSphereCollider->rigidBody.position, sphereTraceVector3Scale(pContactInfo->normal.v, pContactInfo->penetrationDistance));
	ST_Vector3 r = sphereTraceVector3Subtract(pContactInfo->point, pSphereCollider->rigidBody.position);
	float vnMag = sphereTraceVector3Dot(pContactInfo->normal.v, pSphereCollider->rigidBody.velocity);
	ST_Vector3 vn = sphereTraceVector3Scale(pContactInfo->normal.v, vnMag);
	ST_Vector3 vt = sphereTraceVector3Subtract(pSphereCollider->rigidBody.velocity, vn);
	float j = -(1.0f + pSimulationSpace->defaultMaterial.restitution) * vnMag * pSphereCollider->rigidBody.mass;
	//float slope = sphereTraceVector3Dot(pContactInfo->normal.v, gVector3Up);
	b32 restingContactCondition = (sphereTraceAbs(vnMag) < ST_VELOCITY_THRESHOLD);
	float accelNormal = sphereTraceAbs(vnMag);
	//printf("accel normal: %f\n", accelNormal);
	//sphereTraceVector3Print(pContactInfo->normal.v);
	if (pContactInfo->collisionType == ST_COLLISION_INWARD_CIRCULAR)
	{
		ST_Vector3 circularTangent = sphereTraceVector3Cross(pContactInfo->normal.v, pContactInfo->bitangent.v);
		float centripitalAccel = sphereTraceVector3Dot(pSphereCollider->rigidBody.velocity, circularTangent);
		if (sphereTraceAbs(centripitalAccel) / sphereTraceAbs(vnMag) > ST_VELOCITY_THRESHOLD)
		{
			centripitalAccel = centripitalAccel * centripitalAccel / pContactInfo->radiusOfCurvature - sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->normal.v);
			if (centripitalAccel < 0.0f)
				restingContactCondition = 0;
			else
			{
				restingContactCondition = 1;
				accelNormal += centripitalAccel;
			}
		}
	}
	else if (pContactInfo->collisionType == ST_COLLISION_INWARD_SPHEREICAL)
	{
		float centripitalAccel = sphereTraceVector3Length(vt);
		if (centripitalAccel / sphereTraceAbs(vnMag) > ST_VELOCITY_THRESHOLD)
		{
			centripitalAccel = centripitalAccel * centripitalAccel / pContactInfo->radiusOfCurvature - sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->normal.v);
			if (centripitalAccel < 0.0f)
				restingContactCondition = 0;
			else
			{
				restingContactCondition = 1;
				accelNormal += centripitalAccel;
			}
		}
	}
	if (!restingContactCondition)
	{
		pSphereCollider->restingContact = ST_FALSE;
		ST_Vector3 dp = sphereTraceVector3Scale(pContactInfo->normal.v, j);
		if (vnMag < 0.0f)
		{
			sphereTraceRigidBodyAddDeltaMomentum(&pSphereCollider->rigidBody, dp);
			*pNumImpulses = *pNumImpulses + 1;
		}
		ST_Vector3 ft = sphereTraceVector3Scale(sphereTraceVector3Normalize(vt), 
			j*sphereTraceVector3Length(vt)/sphereTraceVector3Length(pSphereCollider->rigidBody.velocity));
		ST_Vector3 dl = sphereTraceVector3Cross(r, sphereTraceVector3Scale(ft, -pSimulationSpace->defaultMaterial.kineticFriction*pSphereCollider->radius));
		if (!sphereTraceVector3NanAny(dl))
			sphereTraceRigidBodyAddDeltaAngularMomentum(&pSphereCollider->rigidBody, dl);
	}
	else
	{
		//if ((1.0f - slope) < ST_SPHERE_RESTING_SLOPE)
		pSphereCollider->restingContact = ST_TRUE;
		pSphereCollider->rigidBody.linearMomentum = sphereTraceVector3Scale(vt, pSphereCollider->rigidBody.mass);
		ST_Vector3 rollingWithoutSlipAngularVelocity = sphereTraceVector3Cross(pContactInfo->normal.v, sphereTraceVector3Scale(pSphereCollider->rigidBody.velocity, 1.0f / pSphereCollider->radius));
		float dav = sphereTraceVector3Distance(pSphereCollider->rigidBody.angularVelocity, rollingWithoutSlipAngularVelocity);

		//if (dav < 0.)
		{
			sphereTraceRigidBodyAddDeltaAngularMomentum(&pSphereCollider->rigidBody, sphereTraceVector3Scale(sphereTraceVector3Subtract(
				rollingWithoutSlipAngularVelocity, pSphereCollider->rigidBody.angularVelocity),
				(1.0f+accelNormal)*pSimulationSpace->defaultMaterial.kineticFriction*ST_KINETIC_FRICTION_MODIFIER*dt));
			//sphereTraceRigidBodySetAngularVelocity(&pSphereCollider->rigidBody, rollingWithoutSlipAngularVelocity);
		}
		//else
		{
			//sphereTraceRigidBodySetAngularVelocity(&pSphereCollider->rigidBody, rollingWithoutSlipAngularVelocity);
		}
		//ST_Vector3 f = sphereTraceVector3Scale(sphereTraceVector3Scale(sphereTraceVector3Normalize(pSphereCollider->rigidBody.velocity), 
		//	sphereTraceVector3Dot(pSimulationSpace->gravitationalAcceleration, pContactInfo->normal.v)), 
		//	(pSphereCollider->rigidBody.mass-j)*pSimulationSpace->defaultMaterial.staticFriction *ST_FRICTION_MODIFIER);
		ST_Vector3 f = sphereTraceVector3Scale(sphereTraceVector3Normalize(pSphereCollider->rigidBody.velocity),
			(-accelNormal+pSimulationSpace->gravitationalAcceleration.y) * pSphereCollider->rigidBody.mass * pSimulationSpace->defaultMaterial.staticFriction * ST_FRICTION_MODIFIER);
		if (!sphereTraceVector3NanAny(f))
			sphereTraceRigidBodyAddForce(&pSphereCollider->rigidBody, f);
	}
}

void sphereTraceSimulationSphereTerrainTriangleResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_SphereContact* const pContactInfo, float dt)
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

	//if (sphereTraceAbs(vnMag) > ST_VELOCITY_THRESHOLD || slope < 0.0f)
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

void sphereTraceSimulationSphereSphereResponse(const ST_SimulationSpace* const pSimulationSpace, const ST_SphereContact* const pContactInfo, float dt)
{
	ST_SphereCollider* pA = pContactInfo->pSphereCollider;
	ST_SphereCollider* pB = pContactInfo->pOtherCollider;
	pA->rigidBody.position = sphereTraceVector3AddAndScale(pA->rigidBody.position, pContactInfo->normal.v, -pContactInfo->penetrationDistance * 0.5f);
	pB->rigidBody.position = sphereTraceVector3AddAndScale(pB->rigidBody.position, pContactInfo->normal.v, pContactInfo->penetrationDistance * 0.5f);
	ST_Vector3 relativeVelocity = sphereTraceVector3Subtract(pB->rigidBody.velocity, pA->rigidBody.velocity);
	float sRel = sphereTraceVector3Dot(relativeVelocity, pContactInfo->normal.v);
	if (sphereTraceAbs(sRel) > ST_VELOCITY_THRESHOLD)
	{
		ST_Vector3 jdt = sphereTraceVector3Scale(pContactInfo->normal.v, -(1.0f + pSimulationSpace->defaultMaterial.restitution) * sRel/ ((1.0f / pA->rigidBody.mass) + (1.0f / pB->rigidBody.mass)));
		sphereTraceRigidBodyAddDeltaMomentum(&pB->rigidBody, jdt);
		sphereTraceRigidBodyAddDeltaMomentum(&pA->rigidBody, sphereTraceVector3Negative(jdt));
	}
}

void sphereTraceSimulationAddCurFrameContactEntry(ST_SphereContact* const pContact)
{
	ST_Collider* pColliderA = pContact->pSphereCollider;
	ST_Collider* pColliderB = pContact->pOtherCollider;

	if (pColliderA->subscriberList.hasSubscriber)
	{
		ST_SphereContactEntry* pContactEntryA = sphereTraceAllocatorAllocateContactEntry();
		pContactEntryA->pOtherCollider = pColliderB;
		pContactEntryA->contact = *pContact;
		sphereTraceIndexListAddUnique(&pColliderA->subscriberList.curFrameContactEntries, pContactEntryA);
		//sphereTraceIndexListAddUnique(pCollidersThatHaveSubscribers, pColliderA);
	}

	if (pColliderB->subscriberList.hasSubscriber)
	{
		ST_SphereContactEntry* pContactEntryB = sphereTraceAllocatorAllocateContactEntry();
		pContactEntryB->pOtherCollider = pColliderA;
		pContactEntryB->contact = *pContact;
		sphereTraceIndexListAddUnique(&pColliderB->subscriberList.curFrameContactEntries, pContactEntryB);
		//sphereTraceIndexListAddUnique(pCollidersThatHaveSubscribers, pColliderB);
	}
}

void sphereTraceSimulationExecuteCallbacksOnCollider(ST_Collider* const pCollider)
{
	ST_IndexListData* pildCur;
	ST_IndexListData* pildPrev;
	ST_IndexListData* pNext;
	ST_IndexListData* pCallbackListData;
	ST_SphereContactEntry* pCurContactEntry;
	ST_SphereContactEntry* pPrevContactEntry;
	

	pildCur = pCollider->subscriberList.curFrameContactEntries.pFirst;
	for (int i = 0; i < pCollider->subscriberList.curFrameContactEntries.count; i++)
	{
		pCurContactEntry = pildCur->value;
		pildPrev = pCollider->subscriberList.contactEntries.pFirst;
		b32 foundEntry = 0;
		for (int j = 0; j < pCollider->subscriberList.contactEntries.count; j++)
		{
			pPrevContactEntry = pildPrev->value;
			if (pCurContactEntry->pOtherCollider == pPrevContactEntry->pOtherCollider)
			{
				foundEntry = 1;
				break;
			}
			pildPrev = pildPrev->pNext;
		}

		//if the entry isnt found, then execute the on trigger enters
		if (!foundEntry)
		{
			ST_CallbackFunction* pCallback;
			pCallbackListData = pCollider->subscriberList.onCollisionEnterCallbacks.pFirst;
			for (int j = 0; j < pCollider->subscriberList.onCollisionEnterCallbacks.count; j++)
			{
				pCallback = pCallbackListData->value;
				pCallback->callback(&pCurContactEntry->contact, pCurContactEntry->pOtherCollider, pCollider->subscriberList.pSubscriberContext);
				pCallbackListData = pCallbackListData->pNext;
			}

			//we can also call the collision stay for this collider as needed
			pCallbackListData = pCollider->subscriberList.onCollisionStayCallbacks.pFirst;
			for (int j = 0; j < pCollider->subscriberList.onCollisionStayCallbacks.count; j++)
			{
				pCallback = pCallbackListData->value;
				pCallback->callback(&pCurContactEntry->contact, pCurContactEntry->pOtherCollider, pCollider->subscriberList.pSubscriberContext);
				pCallbackListData = pCallbackListData->pNext;
			}
		}
		else
		{
			//the entry was found again, call collision stay callback
			ST_CallbackFunction* pCallback;
			pCallbackListData = pCollider->subscriberList.onCollisionStayCallbacks.pFirst;
			for (int j = 0; j < pCollider->subscriberList.onCollisionStayCallbacks.count; j++)
			{
				pCallback = pCallbackListData->value;
				pCallback->callback(&pCurContactEntry->contact, pCurContactEntry->pOtherCollider, pCollider->subscriberList.pSubscriberContext);
				pCallbackListData = pCallbackListData->pNext;
			}
		}
		pildCur = pildCur->pNext;
	}

	//now to find all collisions that have left
	pildPrev = pCollider->subscriberList.contactEntries.pFirst;
	for (int i = 0; i < pCollider->subscriberList.contactEntries.count; i++)
	{
		pPrevContactEntry = pildPrev->value;
		pildCur = pCollider->subscriberList.curFrameContactEntries.pFirst;
		b32 foundEntry = 0;
		for (int j = 0; j < pCollider->subscriberList.curFrameContactEntries.count; j++)
		{
			pCurContactEntry = pildCur->value;
			if (pCurContactEntry->pOtherCollider == pPrevContactEntry->pOtherCollider)
			{
				foundEntry = 1;
				break;
			}
			pildCur = pildCur->pNext;
		}

		//if the entry on the previous frame is no longer found, then call on collision exit
		if (!foundEntry)
		{
			ST_CallbackFunction* pCallback;
			pCallbackListData = pCollider->subscriberList.onCollisionExitCallbacks.pFirst;
			for (int j = 0; j < pCollider->subscriberList.onCollisionExitCallbacks.count; j++)
			{
				pCallback = pCallbackListData->value;
				pCallback->callback(&pPrevContactEntry->contact, pPrevContactEntry->pOtherCollider, pCollider->subscriberList.pSubscriberContext);
				pCallbackListData = pCallbackListData->pNext;
			}
		}
		pildPrev = pildPrev->pNext;
	}

	pildPrev = pCollider->subscriberList.contactEntries.pFirst;
	for (int i = 0; i < pCollider->subscriberList.contactEntries.count; i++)
	{
		pNext = pildPrev->pNext;
		sphereTraceAllocatorFreeContactEntry(pildPrev->value);
		sphereTraceAllocatorFreeIndexListData(pildPrev);
		pildPrev = pNext;
	}

	pCollider->subscriberList.contactEntries = pCollider->subscriberList.curFrameContactEntries;
	pCollider->subscriberList.curFrameContactEntries.count = 0;
	pCollider->subscriberList.curFrameContactEntries.pFirst = NULL;
}


void sphereTraceSimulationGlobalSolveDiscreteFirstComeFirstServe(ST_SimulationSpace* const pSimulationSpace, float dt, ST_Index iterations)
{
	dt = sphereTraceMin(dt, pSimulationSpace->minDeltaTime);
	if (dt < 0.0f)
		dt = pSimulationSpace->minDeltaTime;
	//clear cur frame colliders
	//sphereTraceSimulationClearAllCurFrameContacts(pSimulationSpace);

	//update all sphere aabb's 
	ST_IndexListData* pSphereIndexData;
	ST_IndexListData* pOtherIndexData;

	

	float substepDt = dt / (float)iterations;
	for (ST_Index iters = 0; iters < iterations; iters++)
	{
		pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
		for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
		{
			ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
			sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
			if(iters == 0)
				sphereTraceSimulationApplyForcesAndTorques(pSimulationSpace, &pSphereCollider->rigidBody, dt, 0);
			pSphereIndexData = pSphereIndexData->pNext;
		}

		sphereTraceSimulationStepQuantities(pSimulationSpace, substepDt);

		//ST_IndexList collidersThatHaveSubscribers = sphereTraceIndexListConstruct();

		//check for all sphere plane collisions
		pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
		for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
		{
			ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
			ST_Index numImpulses = 0;
			pOtherIndexData = pSimulationSpace->planeColliders.pFirst;
			for (int planeColliderIndex = 0; planeColliderIndex < pSimulationSpace->planeColliders.count; planeColliderIndex++)
			{
				ST_PlaneCollider* pPlaneCollider = (ST_PlaneCollider*)pOtherIndexData->value;
				ST_SphereContact contactInfo;
				if (sphereTraceColliderPlaneSphereCollisionTest(pPlaneCollider, pSphereCollider, &contactInfo))
				{
					sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
					sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, substepDt, &numImpulses);
				}
				pOtherIndexData = pOtherIndexData->pNext;
			}

			pOtherIndexData = pSimulationSpace->triangleColliders.pFirst;
			for (int triangleColliderIndex = 0; triangleColliderIndex < pSimulationSpace->triangleColliders.count; triangleColliderIndex++)
			{
				ST_TriangleCollider* pTriangleCollider = (ST_TriangleCollider*)pOtherIndexData->value;
				ST_SphereContact contactInfo;
				if (sphereTraceColliderTriangleSphereCollisionTest(pTriangleCollider, pSphereCollider, &contactInfo))
				{
					sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
					sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, substepDt, &numImpulses);
				}
				pOtherIndexData = pOtherIndexData->pNext;
			}

			pOtherIndexData = pSimulationSpace->uniformTerrainColliders.pFirst;
			for (int terrainColliderIndex = 0; terrainColliderIndex < pSimulationSpace->uniformTerrainColliders.count; terrainColliderIndex++)
			{
				ST_UniformTerrainCollider* pTerrainCollider = (ST_UniformTerrainCollider*)pOtherIndexData->value;
				ST_SphereContact contactInfo;
				if (sphereTraceColliderUniformTerrainSphereFindMaxPenetratingTriangle(pTerrainCollider, pSphereCollider, &contactInfo))
				{
					sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
					sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, substepDt, &numImpulses);
				}
				pOtherIndexData = pOtherIndexData->pNext;
			}

			//pOtherIndexData = pSimulationSpace->bowlColliders.pFirst;
			//for (int bowlColliderIndex = 0; bowlColliderIndex < pSimulationSpace->bowlColliders.count; bowlColliderIndex++)
			//{
			//	ST_BowlCollider* pBowlCollider = (ST_BowlCollider*)pOtherIndexData->value;
			//	ST_SphereContact contactInfo;
			//	if (sphereTraceColliderBowlSphereCollisionTest(pBowlCollider, pSphereCollider, &contactInfo))
			//	{
			//		sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
			//		sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, substepDt, &numImpulses);
			//	}
			//	pOtherIndexData = pOtherIndexData->pNext;
			//}

			//pOtherIndexData = pSimulationSpace->pipeColliders.pFirst;
			//for (int pipeColliderIndex = 0; pipeColliderIndex < pSimulationSpace->pipeColliders.count; pipeColliderIndex++)
			//{
			//	ST_PipeCollider* pPipeCollider = (ST_PipeCollider*)pOtherIndexData->value;
			//	ST_SphereContact contactInfo;
			//	if (sphereTraceColliderPipeSphereCollisionTest(pPipeCollider, pSphereCollider, &contactInfo))
			//	{
			//		sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
			//		sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, substepDt, &numImpulses);
			//	}
			//	pOtherIndexData = pOtherIndexData->pNext;
			//}

			//check for all sphere sphere collisions
			pOtherIndexData = pSimulationSpace->sphereColliders.pFirst->pNext;
			for (int sphereColliderBIndex = sphereColliderIndex + 1; sphereColliderBIndex < pSimulationSpace->sphereColliders.count; sphereColliderBIndex++)
			{
				ST_SphereCollider* pSphereColliderB = (ST_SphereCollider*)pOtherIndexData->value;
				ST_SphereContact contactInfo;
				if (sphereTraceColliderSphereSphereCollisionTest(pSphereCollider, pSphereColliderB, &contactInfo))
				{
					sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
					sphereTraceSimulationSphereSphereResponse(pSimulationSpace, &contactInfo, substepDt);
				}
				pOtherIndexData = pOtherIndexData->pNext;
			}

			pOtherIndexData = pSphereCollider->rigidBody.appliedDeltaMomentums.pFirst;
			float s = 1.0f / (float)numImpulses;
			for (int i = 0; i < pSphereCollider->rigidBody.appliedDeltaMomentums.count; i++)
			{
				sphereTraceVector3ScaleByRef(&pOtherIndexData->value, s);
			}

			pSphereIndexData = pSphereIndexData->pNext;
		}

		//execute all the callback
		pOtherIndexData = pSimulationSpace->callbackColliders.pFirst;
		for (int i = 0; i < pSimulationSpace->callbackColliders.count; i++)
		{
			sphereTraceSimulationExecuteCallbacksOnCollider(pOtherIndexData->value);
			pOtherIndexData = pOtherIndexData->pNext;
		}
	}
}

b32 sphereTraceColliderSphereTraceOut(ST_Vector3 spherePos, float sphereRadius, ST_Direction clipoutDir, ST_Collider* pCollider, ST_SphereTraceData* const pSphereCastData)
{
	switch (pCollider->colliderType)
	{
	case COLLIDER_SPHERE:;
		ST_SphereCollider* pSphere = (ST_SphereCollider*)pCollider;
		return sphereTraceColliderSphereSphereTraceOut(spherePos, sphereRadius, clipoutDir, pSphere->rigidBody.position, pSphere->radius, pSphereCastData);
		break;
	case COLLIDER_PLANE:
		return sphereTraceColliderPlaneSphereTraceOut(spherePos, sphereRadius, clipoutDir, pCollider, pSphereCastData);
		break;
	case COLLIDER_TRIANGLE:
		return sphereTraceColliderTriangleSphereTraceOut(spherePos, sphereRadius, clipoutDir, pCollider, pSphereCastData);
		break;
	//case COLLIDER_BOWL:

	}
}

void sphereTraceSimulationResolvePenetration(ST_SphereCollider* pSphereCollider, ST_SphereContact* pContact, PenetrationRestriction* pPenetrationRestriction)
{
	switch (pPenetrationRestriction->penetrationRestrictionType)
	{
	case ST_PENETRATION_RESTRICTION_NONE:
	{
		pSphereCollider->rigidBody.position = sphereTraceVector3Add(pSphereCollider->rigidBody.position, sphereTraceVector3Scale(pContact->normal.v, pContact->penetrationDistance));
		pPenetrationRestriction->positionOnNormal = pSphereCollider->rigidBody.position;
		pPenetrationRestriction->planeNormal = pContact->normal;
		pPenetrationRestriction->penetrationRestrictionType = ST_PENETRATION_RESTRICTION_PLANE;
	}
	break;
	case ST_PENETRATION_RESTRICTION_PLANE:
	{
		pPenetrationRestriction->tangent = sphereTraceDirectionProjectDirectionOntoPlane(pContact->normal, pPenetrationRestriction->planeNormal);
		ST_SphereTraceData std;
		sphereTraceColliderSphereTraceOut(pSphereCollider->rigidBody.position, pSphereCollider->radius,
			pPenetrationRestriction->tangent, pContact->pOtherCollider, &std);
		pSphereCollider->rigidBody.position = std.sphereCenter;
		pPenetrationRestriction->penetrationRestrictionType = ST_PENETRATION_RESTRICTION_DIRECTION;
		pContact->normal = std.rayTraceData.contact.normal;
		pContact->penetrationDistance = 0.0f;
		pContact->point = std.rayTraceData.contact.point;
	}
	break;
	case ST_PENETRATION_RESTRICTION_DIRECTION:
	{
		if (!pPenetrationRestriction->restrictionDirectionSet)
		{
			ST_Vector3 cross = sphereTraceVector3Cross(pPenetrationRestriction->tangent.v, pPenetrationRestriction->planeNormal.v);
			float d = sphereTraceVector3Dot(cross, pContact->normal.v);
			float mul = 1.0f;
			if (d < 0.0f)
				mul = -1.0f;
			pPenetrationRestriction->restrictedDir = sphereTraceDirectionConstruct(sphereTraceVector3Scale(cross, mul), 1);
			pPenetrationRestriction->restrictionDirectionSet = 1;
		}
		ST_SphereTraceData std;
		sphereTraceColliderSphereTraceOut(pSphereCollider->rigidBody.position, pSphereCollider->radius,
			pPenetrationRestriction->restrictedDir, pContact->pOtherCollider, &std);
		pSphereCollider->rigidBody.position = std.sphereCenter;
		pContact->normal = std.rayTraceData.contact.normal;
		pContact->penetrationDistance = 0.0f;
		pContact->point = std.rayTraceData.contact.point;
	}
	break;
	}
}




void sphereTraceSimulationSphereMultipleContactResponse(const ST_SimulationSpace* const pSimulationSpace, ST_SphereCollider* pSphereCollider, float dt)
{
	ST_SphereContact* pRestingContactWithMaxForce = NULL;
	float maxAccel = 0.0f;
	ST_Vector3List impulses = sphereTraceVector3ListConstruct();
	ST_Vector3List restingContactNormals = sphereTraceVector3ListConstruct();
	ST_Vector3List restingContactPoints = sphereTraceVector3ListConstruct();
	ST_Index numContacts = sphereTraceLinearAllocatorGetSphereContactCount();
	for (ST_Index contactIndex = 0; contactIndex < numContacts; contactIndex++)
	{
		ST_SphereContact* pContactInfo = sphereTraceLinearAllocatorGetSphereContactByIndex(contactIndex);
		ST_Vector3 r = sphereTraceVector3Subtract(pContactInfo->point, pSphereCollider->rigidBody.position);
		float vnMag = sphereTraceVector3Dot(pContactInfo->normal.v, pSphereCollider->rigidBody.velocity);
		ST_Vector3 vn = sphereTraceVector3Scale(pContactInfo->normal.v, vnMag);
		ST_Vector3 vt = sphereTraceVector3Subtract(pSphereCollider->rigidBody.velocity, vn);
		float j = -(1.0f + pSimulationSpace->defaultMaterial.restitution) * vnMag * (pSphereCollider->rigidBody.mass / (float)numContacts);
		b32 restingContactCondition = (sphereTraceAbs(vnMag) < ST_VELOCITY_THRESHOLD && pContactInfo->normal.v.y>0.0f);
		float accelNormal = sphereTraceAbs(vnMag);
		//printf("accel normal: %f\n", accelNormal);
		if (!restingContactCondition)
		{
			if (vnMag < 0.0f)
			{
				ST_Vector3 dp = sphereTraceVector3Scale(pContactInfo->normal.v, j);
				sphereTraceVector3ListAddFirst(&impulses, dp);
			}
			ST_Vector3 dl = sphereTraceVector3Cross(sphereTraceVector3Normalize(pSphereCollider->rigidBody.velocity), sphereTraceVector3Scale(pContactInfo->normal.v, -pSimulationSpace->defaultMaterial.kineticFriction * j));
			if (!sphereTraceVector3NanAny(dl))
				sphereTraceRigidBodyAddDeltaAngularMomentum(&pSphereCollider->rigidBody, dl);
			
		}
		else
		{
			if (accelNormal > maxAccel)
			{
				pRestingContactWithMaxForce = pContactInfo;
				maxAccel = accelNormal;
			}
			sphereTraceVector3ListAddFirst(&restingContactNormals, pContactInfo->normal.v);
			sphereTraceVector3ListAddFirst(&restingContactPoints, pContactInfo->point);


		}
	}

	if (pRestingContactWithMaxForce)
	{
		ST_Vector3 r = sphereTraceVector3Subtract(pRestingContactWithMaxForce->point, pSphereCollider->rigidBody.position);
		float vnMag = sphereTraceVector3Dot(pRestingContactWithMaxForce->normal.v, pSphereCollider->rigidBody.velocity);
		ST_Vector3 vn = sphereTraceVector3Scale(pRestingContactWithMaxForce->normal.v, vnMag);
		ST_Vector3 vt = sphereTraceVector3Subtract(pSphereCollider->rigidBody.velocity, vn);
		float j = -(1.0f + pSimulationSpace->defaultMaterial.restitution) * vnMag * pSphereCollider->rigidBody.mass;
		float accelNormal = sphereTraceAbs(vnMag);
		//pSphereCollider->restingContact = ST_TRUE;
		pSphereCollider->rigidBody.linearMomentum = sphereTraceVector3Scale(vt, pSphereCollider->rigidBody.mass);
		ST_Vector3 actualVelocity = sphereTraceVector3Subtract(pSphereCollider->rigidBody.position, pSphereCollider->rigidBody.prevPosition);
		sphereTraceVector3ScaleByRef(&actualVelocity, 1.0f/dt);
		//float actualvnMag = sphereTraceVector3Dot(pRestingContactWithMaxForce->normal.v, actualVelocity);
		//ST_Vector3 actualvn = sphereTraceVector3Scale(pRestingContactWithMaxForce->normal.v, actualvnMag);
		//ST_Vector3 actualvt = sphereTraceVector3Subtract(actualVelocity, actualvn);
		ST_Vector3 rollingWithoutSlipAngularVelocity = sphereTraceVector3Cross(pRestingContactWithMaxForce->normal.v, sphereTraceVector3Scale(vt, 1.0f / pSphereCollider->radius));
		if(pRestingContactWithMaxForce->collisionType==ST_COLLISION_FACE)
			sphereTraceRigidBodySetAngularVelocity(&pSphereCollider->rigidBody, rollingWithoutSlipAngularVelocity);
		//float dav = sphereTraceVector3Distance(pSphereCollider->rigidBody.angularVelocity, rollingWithoutSlipAngularVelocity);
		//sphereTraceRigidBodyAddDeltaAngularMomentum(&pSphereCollider->rigidBody, sphereTraceVector3Scale(sphereTraceVector3Subtract(
		//	rollingWithoutSlipAngularVelocity, pSphereCollider->rigidBody.angularVelocity),
		//	(1.0f + accelNormal) * pSimulationSpace->defaultMaterial.kineticFriction * ST_KINETIC_FRICTION_MODIFIER * dt));
		ST_Vector3 f = sphereTraceVector3Scale(sphereTraceVector3Normalize(pSphereCollider->rigidBody.velocity),
			(-accelNormal + pSimulationSpace->gravitationalAcceleration.y) * pSphereCollider->rigidBody.mass * pSimulationSpace->defaultMaterial.staticFriction * ST_FRICTION_MODIFIER);
		if (!sphereTraceVector3NanAny(f))
			sphereTraceRigidBodyAddForce(&pSphereCollider->rigidBody, f);
	}

	
	ST_Vector3ListData* pvld = impulses.pFirst;
	ST_Vector3 forceSum = gVector3Zero;
	for (int i = 0; i < impulses.count; i++)
	{

		ST_Vector3 modifiedForce = pvld->value;
		ST_Vector3ListData* pRestingContactNormalsData = restingContactNormals.pFirst;
		for (int j = 0; j < restingContactNormals.count; j++)
		{
			float fnm = sphereTraceVector3Dot(modifiedForce, pRestingContactNormalsData->value);
			if (fnm < 0.0f)
			{
				ST_Vector3 fn = sphereTraceVector3Scale(pRestingContactNormalsData->value, fnm);
				modifiedForce = sphereTraceVector3Subtract(modifiedForce, fn);
			}
			pRestingContactNormalsData = pRestingContactNormalsData->pNext;
		}

		//sphereTraceRigidBodyAddDeltaMomentum(&pSphereCollider->rigidBody, modifiedForce);
		sphereTraceVector3AddByRef(&forceSum, modifiedForce);
		pvld = pvld->pNext;
	}
	sphereTraceRigidBodyAddDeltaMomentum(&pSphereCollider->rigidBody, forceSum);
	sphereTraceVector3ListFree(&impulses);
	sphereTraceVector3ListFree(&restingContactNormals);
	sphereTraceVector3ListFree(&restingContactPoints);

}

void sphereTraceSimulationGlobalSolveDiscrete(ST_SimulationSpace* const pSimulationSpace, float dt)
{
	dt = sphereTraceMin(dt, pSimulationSpace->minDeltaTime);
	if (dt < 0.0f)
		dt = pSimulationSpace->minDeltaTime;

	//step all quantities
	sphereTraceSimulationStepQuantities(pSimulationSpace, dt);

	//update all sphere aabb's 
	ST_IndexListData* pSphereIndexData;
	ST_IndexListData* pOtherIndexData;
	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
		pSphereIndexData = pSphereIndexData->pNext;
	}

	PenetrationRestriction penetrationRestriction;
	
	//ST_SphereContact contacts[ST_CONTACT_MAX];
	ST_Index contactsCount;

	//first do all sphere-sphere collisions
	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;

		//check for all sphere sphere collisions
		pOtherIndexData = pSimulationSpace->sphereColliders.pFirst;
		for (int sphereColliderBIndex = 0; sphereColliderBIndex < pSimulationSpace->sphereColliders.count; sphereColliderBIndex++)
		{
			ST_SphereCollider* pSphereColliderB = (ST_SphereCollider*)pOtherIndexData->value;

			if (pSphereCollider != pSphereColliderB)
			{
				ST_SphereContact contactInfo;
				if (sphereTraceColliderSphereSphereCollisionTest(pSphereCollider, pSphereColliderB, &contactInfo))
				{
					sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
					sphereTraceSimulationSphereSphereResponse(pSimulationSpace, &contactInfo, dt);
				}
			}
			pOtherIndexData = pOtherIndexData->pNext;
		}
		pSphereIndexData = pSphereIndexData->pNext;
	}

	//check for all sphere plane collisions
	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		ST_SphereContact contactInfo;
		penetrationRestriction.penetrationRestrictionType = ST_PENETRATION_RESTRICTION_NONE;
		penetrationRestriction.restrictionDirectionSet = 0;
		contactsCount = 0;
		sphereTraceLinearAllocatorResetSphereContacts();

		pOtherIndexData = pSimulationSpace->planeColliders.pFirst;
		for (int planeColliderIndex = 0; planeColliderIndex < pSimulationSpace->planeColliders.count; planeColliderIndex++)
		{
			ST_PlaneCollider* pPlaneCollider = (ST_PlaneCollider*)pOtherIndexData->value;
			if (sphereTraceColliderPlaneSphereCollisionTest(pPlaneCollider, pSphereCollider, &contactInfo))
			{
				if (!sphereTraceIndexListContains(&pSphereCollider->prevFrameContacts, pPlaneCollider))
				{
					sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
					sphereTraceSimulationResolvePenetration(pSphereCollider, &contactInfo, &penetrationRestriction);
					ST_SphereContact* pAllocatedContact = sphereTraceLinearAllocatorAllocateSphereContact();
					*pAllocatedContact = contactInfo;
				}
			}
			pOtherIndexData = pOtherIndexData->pNext;
		}



		pOtherIndexData = pSimulationSpace->triangleColliders.pFirst;
		for (int triangleColliderIndex = 0; triangleColliderIndex < pSimulationSpace->triangleColliders.count; triangleColliderIndex++)
		{
			ST_TriangleCollider* pTriangleCollider = (ST_TriangleCollider*)pOtherIndexData->value;
			if (sphereTraceColliderTriangleSphereCollisionTest(pTriangleCollider, pSphereCollider, &contactInfo))
			{
				if (!sphereTraceIndexListContains(&pSphereCollider->prevFrameContacts, pTriangleCollider))
				{
					sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
					sphereTraceSimulationResolvePenetration(pSphereCollider, &contactInfo, &penetrationRestriction);
					ST_SphereContact* pAllocatedContact = sphereTraceLinearAllocatorAllocateSphereContact();
					*pAllocatedContact = contactInfo;
				}
			}
			pOtherIndexData = pOtherIndexData->pNext;
		}

		pOtherIndexData = pSimulationSpace->uniformTerrainColliders.pFirst;
		for (int terrainColliderIndex = 0; terrainColliderIndex < pSimulationSpace->uniformTerrainColliders.count; terrainColliderIndex++)
		{
			ST_UniformTerrainCollider* pTerrainCollider = (ST_UniformTerrainCollider*)pOtherIndexData->value;
			if (sphereTraceColliderUniformTerrainSphereFindMaxPenetratingTriangle(pTerrainCollider, pSphereCollider, &contactInfo))
			{
				if (!sphereTraceIndexListContains(&pSphereCollider->prevFrameContacts, pTerrainCollider))
				{
					sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
					sphereTraceSimulationResolvePenetration(pSphereCollider, &contactInfo, &penetrationRestriction);
					ST_SphereContact* pAllocatedContact = sphereTraceLinearAllocatorAllocateSphereContact();
					*pAllocatedContact = contactInfo;
				}
			}
			pOtherIndexData = pOtherIndexData->pNext;
		}

		//pOtherIndexData = pSimulationSpace->bowlColliders.pFirst;
		//for (int bowlColliderIndex = 0; bowlColliderIndex < pSimulationSpace->bowlColliders.count; bowlColliderIndex++)
		//{
		//	ST_BowlCollider* pBowlCollider = (ST_BowlCollider*)pOtherIndexData->value;
		//	if (sphereTraceColliderBowlSphereCollisionTest(pBowlCollider, pSphereCollider, &contactInfo))
		//	{
		//		if (!sphereTraceIndexListContains(&pSphereCollider->prevFrameContacts, pBowlCollider))
		//		{
		//			sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
		//			sphereTraceSimulationResolvePenetration(pSphereCollider, &contactInfo, &penetrationRestriction);
		//			ST_SphereContact* pAllocatedContact = sphereTraceLinearAllocatorAllocateSphereContact();
		//			*pAllocatedContact = contactInfo;
		//		}
		//	}
		//	pOtherIndexData = pOtherIndexData->pNext;
		//}

		//pOtherIndexData = pSimulationSpace->pipeColliders.pFirst;
		//for (int pipeColliderIndex = 0; pipeColliderIndex < pSimulationSpace->pipeColliders.count; pipeColliderIndex++)
		//{
		//	ST_PipeCollider* pPipeCollider = (ST_PipeCollider*)pOtherIndexData->value;
		//	if (sphereTraceColliderPipeSphereCollisionTest(pPipeCollider, pSphereCollider, &contactInfo))
		//	{
		//		if (!sphereTraceIndexListContains(&pSphereCollider->prevFrameContacts, pPipeCollider))
		//		{
		//			sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
		//			sphereTraceSimulationResolvePenetration(pSphereCollider, &contactInfo, &penetrationRestriction);
		//			ST_SphereContact* pAllocatedContact = sphereTraceLinearAllocatorAllocateSphereContact();
		//			*pAllocatedContact = contactInfo;
		//		}
		//	}
		//	pOtherIndexData = pOtherIndexData->pNext;
		//}

		//check for all sphere sphere collisions
		//pOtherIndexData = pSimulationSpace->sphereColliders.pFirst;
		//for (int sphereColliderBIndex = 0; sphereColliderBIndex < pSimulationSpace->sphereColliders.count; sphereColliderBIndex++)
		//{
		//	ST_SphereCollider* pSphereColliderB = (ST_SphereCollider*)pOtherIndexData->value;
		//	if (pSphereCollider != pSphereColliderB)
		//	{
		//		ST_SphereContact contactInfo;
		//		if (sphereTraceColliderSphereSphereCollisionTest(pSphereCollider, pSphereColliderB, &contactInfo))
		//		{
		//			//sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
		//			sphereTraceSimulationResolvePenetration(pSphereCollider, &contactInfo, &penetrationRestriction);
		//		}
		//	}
		//	pOtherIndexData = pOtherIndexData->pNext;
		//}

		ST_Vector3 dp = sphereTraceVector3Subtract(pSphereCollider->rigidBody.position, pSphereCollider->rigidBody.prevPosition);
		float actualSpeed = sqrtf(sphereTraceVector3Dot(dp, dp)) / dt;
		float rbSpeed = sphereTraceRigidBodyGetSpeed(&pSphereCollider->rigidBody);
		//if the actual position speed calculation is greater than the rigidbody speed
		//some sort of teleportation is taking place due to the penetration constraints
		//and we need to cap the teleportation
		if (actualSpeed > rbSpeed)
		{
			//printf("speed: %f, actual speed: %f\n", actualSpeed, rbSpeed);
			ST_Vector3 dir = sphereTraceVector3Normalize(dp);
			sphereTraceVector3AddAndScaleByRef(&pSphereCollider->rigidBody.prevPosition, dir, rbSpeed* dt);
		}
		//printf("contacts count: %i\n", contactsCount);
		sphereTraceSimulationSphereMultipleContactResponse(pSimulationSpace, pSphereCollider, dt);

		pSphereIndexData = pSphereIndexData->pNext;
	}

	//execute all the callback
	pOtherIndexData = pSimulationSpace->callbackColliders.pFirst;
	for (int i = 0; i < pSimulationSpace->callbackColliders.count; i++)
	{
		sphereTraceSimulationExecuteCallbacksOnCollider(pOtherIndexData->value);
		pOtherIndexData = pOtherIndexData->pNext;
	}
}

void sphereTraceSimulationOctTreeSolveDiscrete(ST_SimulationSpace* const pSimulationSpace, float dt)
{
	//dt = sphereTraceMin(dt, pSimulationSpace->minDeltaTime);
	//if (dt < 0.0f)
	//	dt = pSimulationSpace->minDeltaTime;

	//step all quantities
	sphereTraceSimulationStepQuantities(pSimulationSpace, dt);


	//update all sphere aabb's 
	ST_IndexListData* pSphereIndexData;
	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	for (int i = 0; i < pSimulationSpace->sphereColliders.count; i++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
		sphereTraceOctTreeReInsertCollider(&pSimulationSpace->octTree, &pSphereCollider->collider, ST_FALSE);
		pSphereIndexData = pSphereIndexData->pNext;
	}


	ST_IndexListData* pOtherIndexData;

	PenetrationRestriction penetrationRestriction;

	//ST_SphereContact contacts[ST_CONTACT_MAX];
	ST_Index contactsCount;

	////first do all sphere-sphere collisions
	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		ST_SphereContact contactInfo;
		ST_IndexList handledCollisionList = sphereTraceIndexListConstruct();
		ST_IndexList sampledColliders = sphereTraceIndexListConstruct();
		sphereTraceOctTreeReSampleIntersectionLeafsAndColliders(&pSimulationSpace->octTree, &pSphereCollider->collider.aabb, NULL, &sampledColliders, ST_TRUE, ST_FALSE);
		sphereTraceSortedIndexListRemove(&sampledColliders, &pSphereCollider->collider);
		pOtherIndexData = sampledColliders.pFirst;
		ST_Collider* pOtherCollider;
		for (ST_Index i = 0; i < sampledColliders.count; i++)
		{
			pOtherCollider = pOtherIndexData->value;
			if (!sphereTraceSortedIndexListContains(&handledCollisionList, pOtherCollider))
			{
				if (pOtherCollider->colliderType == COLLIDER_SPHERE)
				{
					ST_SphereCollider* pSphereColliderB = (ST_PlaneCollider*)pOtherCollider;
					if (sphereTraceColliderSphereSphereCollisionTest(pSphereCollider, pSphereColliderB, &contactInfo))
					{
						sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
						sphereTraceSimulationSphereSphereResponse(pSimulationSpace, &contactInfo, dt);
						sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
						sphereTraceColliderSphereAABBSetTransformedVertices(pSphereColliderB);
						sphereTraceOctTreeReInsertCollider(&pSimulationSpace->octTree, &pSphereCollider->collider, ST_FALSE);
						sphereTraceOctTreeReInsertCollider(&pSimulationSpace->octTree, &pSphereColliderB->collider, ST_FALSE);
						sphereTraceIndexListFree(&sampledColliders);
						sphereTraceOctTreeReSampleIntersectionLeafsAndColliders(&pSimulationSpace->octTree, &pSphereCollider->collider.aabb, NULL, &sampledColliders, ST_TRUE, ST_FALSE);
						sphereTraceSortedIndexListRemove(&sampledColliders, &pSphereCollider->collider);
						sphereTraceSortedIndexListAddUnique(&handledCollisionList, pOtherCollider);
						i = 0;
						pOtherIndexData = sampledColliders.pFirst;
					}
				}
			}
			if (sampledColliders.count == 0)
				break;
			pOtherIndexData = pOtherIndexData->pNext;
		}
		sphereTraceIndexListFree(&sampledColliders);
		sphereTraceIndexListFree(&handledCollisionList);

		pSphereIndexData = pSphereIndexData->pNext;
	}

	//check for all sphere plane collisions
	pSphereIndexData = pSimulationSpace->sphereColliders.pFirst;
	for (int sphereColliderIndex = 0; sphereColliderIndex < pSimulationSpace->sphereColliders.count; sphereColliderIndex++)
	{
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		ST_SphereContact contactInfo;
		penetrationRestriction.penetrationRestrictionType = ST_PENETRATION_RESTRICTION_NONE;
		penetrationRestriction.restrictionDirectionSet = 0;
		contactsCount = 0;
		sphereTraceLinearAllocatorResetSphereContacts();

		ST_IndexList handledCollisionList = sphereTraceIndexListConstruct();
		ST_IndexList sampledColliders = sphereTraceIndexListConstruct();
		sphereTraceOctTreeReSampleIntersectionLeafsAndColliders(&pSimulationSpace->octTree, &pSphereCollider->collider.aabb, NULL, &sampledColliders, ST_FALSE, ST_TRUE);
		sphereTraceSortedIndexListRemove(&sampledColliders, &pSphereCollider->collider);
		pOtherIndexData = sampledColliders.pFirst;
		ST_Collider* pOtherCollider;
		for (ST_Index i = 0; i < sampledColliders.count; i++)
		{
			pOtherCollider = pOtherIndexData->value;
			if (!sphereTraceSortedIndexListContains(&handledCollisionList, pOtherCollider))
			{
				switch (pOtherCollider->colliderType)
				{
				case COLLIDER_PLANE:
				{
					ST_PlaneCollider* pPlaneCollider = (ST_PlaneCollider*)pOtherCollider;
					if (sphereTraceColliderPlaneSphereCollisionTest(pPlaneCollider, pSphereCollider, &contactInfo))
					{
						sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
						sphereTraceSimulationResolvePenetration(pSphereCollider, &contactInfo, &penetrationRestriction);
						ST_SphereContact* pAllocatedContact = sphereTraceLinearAllocatorAllocateSphereContact();
						*pAllocatedContact = contactInfo;
						sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
						sphereTraceOctTreeReInsertCollider(&pSimulationSpace->octTree, &pSphereCollider->collider, ST_FALSE);
						sphereTraceIndexListFree(&sampledColliders);
						sphereTraceOctTreeReSampleIntersectionLeafsAndColliders(&pSimulationSpace->octTree, &pSphereCollider->collider.aabb, NULL, &sampledColliders, ST_FALSE, ST_TRUE);
						sphereTraceSortedIndexListRemove(&sampledColliders, &pSphereCollider->collider);
						sphereTraceSortedIndexListAddUnique(&handledCollisionList, pOtherCollider);
						i = 0;
						pOtherIndexData = sampledColliders.pFirst;
					}
				}
				break;
				case COLLIDER_SPHERE:
				{
					//ST_SphereCollider* pSphereColliderB = (ST_PlaneCollider*)pOtherCollider;
					//if (sphereTraceColliderSphereSphereCollisionTest(pSphereCollider, pSphereColliderB, &contactInfo))
					//{
					//	sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
					//	sphereTraceSimulationSphereSphereResponse(pSimulationSpace, &contactInfo, dt);
					//	sphereTraceColliderSphereAABBSetTransformedVertices(pSphereCollider);
					//	sphereTraceColliderSphereAABBSetTransformedVertices(pSphereColliderB);
					//	sphereTraceOctTreeReInsertCollider(&pSimulationSpace->octTree, &pSphereCollider->collider, ST_FALSE);
					//	sphereTraceOctTreeReInsertCollider(&pSimulationSpace->octTree, &pSphereColliderB->collider, ST_FALSE);
					//	sphereTraceIndexListFree(&sampledColliders);
					//	sphereTraceOctTreeReSampleIntersectionLeafsAndColliders(&pSimulationSpace->octTree, &pSphereCollider->collider.aabb, NULL, &sampledColliders);
					//	sphereTraceSortedIndexListRemove(&sampledColliders, &pSphereCollider->collider);
					//	sphereTraceSortedIndexListAddUnique(&handledCollisionList, pOtherCollider);
					//	i = 0;
					//	pOtherIndexData = sampledColliders.pFirst;
					//}
				}
				break;
				case COLLIDER_TRIANGLE:
				{

				}
				break;
				case COLLIDER_TERRAIN:
				{

				}
				break;
				}
			}
			if (sampledColliders.count == 0)
				break;
			pOtherIndexData = pOtherIndexData->pNext;
		}
		sphereTraceIndexListFree(&sampledColliders);
		sphereTraceIndexListFree(&handledCollisionList);


		ST_Vector3 dp = sphereTraceVector3Subtract(pSphereCollider->rigidBody.position, pSphereCollider->rigidBody.prevPosition);
		float wannabeSpeed = sqrtf(sphereTraceVector3Dot(dp, dp)) / dt;
		float rbSpeed = sphereTraceRigidBodyGetSpeed(&pSphereCollider->rigidBody);
		//if the actual position speed calculation is greater than the rigidbody speed
		//some sort of teleportation is taking place due to the penetration constraints
		//and we need to cap the teleportation
		if (wannabeSpeed > rbSpeed)
		{
			//printf("speed: %f, actual speed: %f\n", actualSpeed, rbSpeed);
			ST_Vector3 dir = sphereTraceVector3Normalize(dp);
			sphereTraceVector3AddAndScaleByRef(&pSphereCollider->rigidBody.prevPosition, dir, rbSpeed * dt);
		}

		//printf("contacts count: %i\n", contactsCount);
		sphereTraceSimulationSphereMultipleContactResponse(pSimulationSpace, pSphereCollider, dt);

		rbSpeed = sphereTraceRigidBodyGetSpeed(&pSphereCollider->rigidBody);
		if (rbSpeed < ST_RESTING_SPEED_SQUARED)
		{
			sphereTraceVector3ScaleByRef(&pSphereCollider->rigidBody.linearMomentum, 0.0f);
		}
		if (wannabeSpeed < ST_RESTING_SPEED_SQUARED && rbSpeed>0.0f)
		{
			sphereTraceVector3ScaleByRef(&pSphereCollider->rigidBody.linearMomentum, wannabeSpeed / rbSpeed);
		}

		pSphereIndexData = pSphereIndexData->pNext;
	}

	//execute all the callback
	pOtherIndexData = pSimulationSpace->callbackColliders.pFirst;
	for (int i = 0; i < pSimulationSpace->callbackColliders.count; i++)
	{
		sphereTraceSimulationExecuteCallbacksOnCollider(pOtherIndexData->value);
		pOtherIndexData = pOtherIndexData->pNext;
	}
}

void sphereTraceSimulationSolveDiscreteFirstComeFirstServe(ST_SimulationSpace* const pSimulationSpace, float dt)
{
	dt = sphereTraceMin(dt, pSimulationSpace->minDeltaTime);
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

	sphereTraceSimulationStepQuantities(pSimulationSpace, dt);


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
		ST_SphereCollider* pSphereCollider = (ST_SphereCollider*)pSphereIndexData->value;
		ST_Index numImpulses = 0;
		ST_IndexList bucketsToCheck = sphereTraceSpacialPartitionStaticGetBucketIndicesFromAABB(&pSimulationSpace->spacialPartitionContainer, &pSphereCollider->collider.aabb);
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
						ST_SphereContact contactInfo;
						if (sphereTraceColliderSphereSphereCollisionTest(pSphereCollider, pSphereColliderB, &contactInfo))
						{
							sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
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
						ST_SphereContact contactInfo;
						if (sphereTraceColliderPlaneSphereCollisionTest(pCollider, pSphereCollider, &contactInfo))
						{
							sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
							sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, dt, &numImpulses);
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
						ST_SphereContact contactInfo;
						if (sphereTraceColliderUniformTerrainSphereFindMaxPenetratingTriangle(pCollider, pSphereCollider, &contactInfo))
						{
							sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
							sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, dt, &numImpulses);
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
						ST_SphereContact contactInfo;
						if (sphereTraceColliderPipeSphereCollisionTest(pCollider, pSphereCollider, &contactInfo))
						{
							sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
							sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, dt, &numImpulses);
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
						ST_SphereContact contactInfo;
						if (sphereTraceColliderBowlSphereCollisionTest(pCollider, pSphereCollider, &contactInfo))
						{
							sphereTraceSimulationAddCurFrameContactEntry(&contactInfo);
							sphereTraceSimulationSphereContactResponse(pSimulationSpace, &contactInfo, dt, &numImpulses);
						}
					}
					pOtherIndexData = pOtherIndexData->pNext;
				}
			}
		}

		pOtherIndexData = pSphereCollider->rigidBody.appliedDeltaMomentums.pFirst;
		float s = 1.0f / (float)numImpulses;
		for (int i = 0; i < pSphereCollider->rigidBody.appliedDeltaMomentums.count; i++)
		{
			sphereTraceVector3ScaleByRef(&pOtherIndexData->value, s);
		}

		sphereTraceIndexListFree(&bucketsToCheck);
		sphereTraceIndexListFree(&checkedBowlColliders);
		sphereTraceIndexListFree(&checkedPipeColliders);
		sphereTraceIndexListFree(&checkedTerrainColliders);
		sphereTraceIndexListFree(&checkedPlaneColliders);
		sphereTraceIndexListFree(&checkedSphereColliders);
		pSphereIndexData = pSphereIndexData->pNext;
	}

	//execute all the callback
	pOtherIndexData = pSimulationSpace->callbackColliders.pFirst;
	for (int i = 0; i < pSimulationSpace->callbackColliders.count; i++)
	{
		sphereTraceSimulationExecuteCallbacksOnCollider(pOtherIndexData->value);
		pOtherIndexData = pOtherIndexData->pNext;
	}
}

void sphereTraceSimulationGlobalSolveImposedPosition(ST_SimulationSpace* const pSimulationSpace, float dt)
{
	dt = sphereTraceMin(dt, pSimulationSpace->minDeltaTime);
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
					float adjustedDt = sphereTraceMin((dt - accumulatedDt) * sphereCastDistance / imposedNextLength + ST_AUTO_DT_FACTOR * dt, dt - accumulatedDt);

					//step the simulation
					sphereTraceSimulationStepQuantity(pSimulationSpace, &pSphereCollider->rigidBody, adjustedDt);
					//sphereTraceVector3Print(pSphereCollider->pRigidBody->linearMomentum);

					//resolve the collisions if any
					ST_SphereContact contactInfo;
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
					float adjustedDt = sphereTraceMin((dt - accumulatedDt) * sphereCastDistance / imposedNextLength + ST_AUTO_DT_FACTOR * dt, dt - accumulatedDt);

					//step the simulation
					sphereTraceSimulationStepQuantity(pSimulationSpace, &pSphereCollider->rigidBody, adjustedDt);

					ST_SphereContact contactInfo;
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
					float adjustedDt = sphereTraceMin((dt - accumulatedDt) * sphereCastDistance / imposedNextLength + ST_AUTO_DT_FACTOR * dt, dt - accumulatedDt);

					//step the simulation
					sphereTraceSimulationStepQuantity(pSimulationSpace, &pSphereCollider->rigidBody, adjustedDt);

					ST_SphereContact contactInfo;
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
	//dt = sphereTraceMin(dt, pSimulationSpace->minDeltaTime);
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
									if (sphereTraceColliderAABBIntersectAABB(&imposedSpherePathAABB, &pSphereColliderOther->collider.aabb))
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
					float adjustedDt = sphereTraceMin((dt - accumulatedDt) * sphereCastDistance / imposedNextLength + ST_AUTO_DT_FACTOR * dt, dt - accumulatedDt);
					if (closestPlaneSame)
					{
						adjustedDt = dt - accumulatedDt;
					}
					//step the simulation
					sphereTraceSimulationStepQuantity(pSimulationSpace, &pSphereCollider->rigidBody, adjustedDt);

					//resolve the collisions if any
					ST_SphereContact contactInfo;
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