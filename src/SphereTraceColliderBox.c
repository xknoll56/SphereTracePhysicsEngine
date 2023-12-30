#include "SphereTraceColliderBox.h"
#include "SphereTraceGlobals.h"

void sphereTraceColliderBoxSetLocalAxis(ST_BoxCollider* const pBoxCollider)
{
	pBoxCollider->localRight = sphereTraceDirectionGetLocalXAxisFromRotationMatrix(pBoxCollider->rigidBody.rotationMatrix);
	pBoxCollider->localUp = sphereTraceDirectionGetLocalYAxisFromRotationMatrix(pBoxCollider->rigidBody.rotationMatrix);
	pBoxCollider->localForward = sphereTraceDirectionGetLocalZAxisFromRotationMatrix(pBoxCollider->rigidBody.rotationMatrix);
}

void sphereTraceColliderBoxUpdateTransformedVertices(ST_BoxCollider* const pBoxCollider)
{
	pBoxCollider->transformedVertices[ST_LEFT_DOWN_BACK] = sphereTraceVector3AddAndScale3(pBoxCollider->rigidBody.position, pBoxCollider->localRight.v, -pBoxCollider->halfExtents.x,
		pBoxCollider->localUp.v, -pBoxCollider->halfExtents.y, pBoxCollider->localForward.v, -pBoxCollider->halfExtents.z);
	pBoxCollider->transformedVertices[ST_RIGHT_DOWN_BACK] = sphereTraceVector3AddAndScale3(pBoxCollider->rigidBody.position, pBoxCollider->localRight.v, pBoxCollider->halfExtents.x,
		pBoxCollider->localUp.v, -pBoxCollider->halfExtents.y, pBoxCollider->localForward.v, -pBoxCollider->halfExtents.z);
	pBoxCollider->transformedVertices[ST_LEFT_DOWN_FORWARD] = sphereTraceVector3AddAndScale3(pBoxCollider->rigidBody.position, pBoxCollider->localRight.v, -pBoxCollider->halfExtents.x,
		pBoxCollider->localUp.v, -pBoxCollider->halfExtents.y, pBoxCollider->localForward.v, pBoxCollider->halfExtents.z);
	pBoxCollider->transformedVertices[ST_RIGHT_DOWN_FORWARD] = sphereTraceVector3AddAndScale3(pBoxCollider->rigidBody.position, pBoxCollider->localRight.v, pBoxCollider->halfExtents.x,
		pBoxCollider->localUp.v, -pBoxCollider->halfExtents.y, pBoxCollider->localForward.v, pBoxCollider->halfExtents.z);
	pBoxCollider->transformedVertices[ST_LEFT_UP_BACK] = sphereTraceVector3AddAndScale3(pBoxCollider->rigidBody.position, pBoxCollider->localRight.v, -pBoxCollider->halfExtents.x,
		pBoxCollider->localUp.v, pBoxCollider->halfExtents.y, pBoxCollider->localForward.v, -pBoxCollider->halfExtents.z);
	pBoxCollider->transformedVertices[ST_RIGHT_UP_BACK] = sphereTraceVector3AddAndScale3(pBoxCollider->rigidBody.position, pBoxCollider->localRight.v, pBoxCollider->halfExtents.x,
		pBoxCollider->localUp.v, pBoxCollider->halfExtents.y, pBoxCollider->localForward.v, -pBoxCollider->halfExtents.z);
	pBoxCollider->transformedVertices[ST_LEFT_UP_FORWARD] = sphereTraceVector3AddAndScale3(pBoxCollider->rigidBody.position, pBoxCollider->localRight.v, -pBoxCollider->halfExtents.x,
		pBoxCollider->localUp.v, pBoxCollider->halfExtents.y, pBoxCollider->localForward.v, pBoxCollider->halfExtents.z);
	pBoxCollider->transformedVertices[ST_RIGHT_UP_FORWARD] = sphereTraceVector3AddAndScale3(pBoxCollider->rigidBody.position, pBoxCollider->localRight.v, pBoxCollider->halfExtents.x,
		pBoxCollider->localUp.v, pBoxCollider->halfExtents.y, pBoxCollider->localForward.v, pBoxCollider->halfExtents.z);
}

void sphereTraceColliderBoxUpdateTransformedEdges(ST_BoxCollider* const pBoxCollider)
{
	
}

ST_BoxCollider sphereTraceColliderBoxConstruct(ST_Vector3 halfExtents)
{
	ST_BoxCollider bc;
	bc.collider = sphereTraceColliderConstruct(COLLIDER_BOX, sphereTraceVector3Length(halfExtents));
	bc.rigidBody = sphereTraceRigidBodyConstruct(1.0f, 1.0f);
	bc.halfExtents = halfExtents;
	bc.ignoreCollisions = ST_FALSE;
	bc.restingContact = ST_FALSE;
	sphereTraceColliderBoxSetLocalAxis(&bc);
	return bc;
}



ST_BoxFace sphereTraceColliderBoxGetFaceClosestToDirection(const ST_BoxCollider* const pBoxCollider, const ST_Direction dir)
{
	float dotRight = sphereTraceVector3Dot(dir.v, pBoxCollider->localRight.v);
	float dotUp = sphereTraceVector3Dot(dir.v, pBoxCollider->localUp.v);
	float dotForward = sphereTraceVector3Dot(dir.v, pBoxCollider->localForward.v);
	if (sphereTraceAbs(dotRight) > sphereTraceAbs(dotUp))
	{
		//right/left closer that up
		if (sphereTraceAbs(dotRight) > sphereTraceAbs(dotForward))
		{
			//right/left closer than forward
			if (dotRight > 0.0f)
			{
				return gFaceRight;
			}
			else
			{
				return gFaceLeft;
			}
		}
		else
		{
			if (dotForward > 0.0f)
			{
				return gFaceForward;
			}
			else
			{
				return gFaceBack;
			}
		}
	}
	else
	{
		if (sphereTraceAbs(dotUp) > sphereTraceAbs(dotForward))
		{
			if (dotUp > 0.0f)
			{
				return gFaceUp;
			}
			else
			{
				return gFaceDown;
			}
		}
		else
		{
			if (dotForward > 0.0f)
			{
				return gFaceForward;
			}
			else
			{
				return gFaceBack;
			}
		}
	}
}

void sphereTraceColliderBoxSetAABB(ST_BoxCollider* const pBoxCollider)
{
	ST_BoxFace rightFace = sphereTraceColliderBoxGetFaceClosestToDirection(pBoxCollider, gDirectionRight);
	ST_BoxFace upFace = sphereTraceColliderBoxGetFaceClosestToDirection(pBoxCollider, gDirectionUp);
	ST_BoxFace forwardFace = sphereTraceColliderBoxGetFaceClosestToDirection(pBoxCollider, gDirectionForward);
	//pBoxCollider->
}