#include "SphereTraceColliderBowl.h"
#include "SphereTraceGlobals.h"

ST_BowlCollider sphereTraceColliderBowlConstruct(ST_Vector3 position, float radius, ST_Direction normal)
{
	ST_BowlCollider bowlCollider;
	bowlCollider.position = position;
	bowlCollider.radius = radius;
	bowlCollider.normal = normal;
	bowlCollider.collider = sphereTraceColliderConstruct(COLLIDER_BOWL, radius);
	//bowlCollider.collider.colliderType = COLLIDER_BOWL;
	//bowlCollider.collider.bucketIndices = sphereTraceIndexListConstruct();
	sphereTraceColliderBowlsSetAABB(&bowlCollider);
	return bowlCollider;
}

b32 sphereTraceColliderBowlImposedSphereCollisionTest(ST_BowlCollider* const pBowlCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereContact* pContact)
{
	ST_Vector3 dp = sphereTraceVector3Subtract(imposedPosition, pBowlCollider->position);
	float dist = sphereTraceVector3Length(dp);
	float normDist = sphereTraceVector3Dot(pBowlCollider->normal.v, dp);
	if (normDist <= 0.0f)
	{
		if (dist > pBowlCollider->radius && dist <= (pBowlCollider->radius + imposedRadius))
		{
			//outside of bowl
			pContact->collisionType = ST_COLLISION_OUTWARD_SPHEREICAL;
			pContact->pOtherCollider = pBowlCollider;
			pContact->otherColliderType = COLLIDER_BOWL;
			pContact->normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(dp, 1.0f / dist), 1);
			pContact->point = sphereTraceVector3AddAndScale(pBowlCollider->position, pContact->normal.v, pBowlCollider->radius);
			pContact->penetrationDistance = pBowlCollider->radius + imposedRadius - dist;
			pContact->radiusOfCurvature = pBowlCollider->radius;
			return 1;
		}
		else if (dist < pBowlCollider->radius && dist >= (pBowlCollider->radius - imposedRadius))
		{
			//inside of bowl
			pContact->collisionType = ST_COLLISION_INWARD_SPHEREICAL;
			pContact->pOtherCollider = pBowlCollider;
			pContact->otherColliderType = COLLIDER_BOWL;
			pContact->normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(dp, -1.0f / dist), 1);
			pContact->point = sphereTraceVector3AddAndScale(pBowlCollider->position, sphereTraceVector3Negative(pContact->normal.v), pBowlCollider->radius);
			pContact->penetrationDistance = -pBowlCollider->radius + imposedRadius + dist;
			pContact->radiusOfCurvature = pBowlCollider->radius;
			return 1;
		}
	}
	else if (normDist <= imposedRadius)
	{
		ST_Vector3 bowlRight = sphereTraceVector3Normalize(sphereTraceVector3Cross(dp, pBowlCollider->normal.v));
		ST_Vector3 bowlFwd = sphereTraceVector3Cross(pBowlCollider->normal.v, bowlRight);
		ST_Vector3 bowlCirclePlanePos = sphereTraceVector3AddAndScale2(gVector3Zero, bowlRight, sphereTraceVector3Dot(bowlRight, dp),
			bowlFwd, sphereTraceVector3Dot(bowlFwd, dp));
		ST_Vector3 bowlCirclePlaneDir = sphereTraceVector3Normalize(bowlCirclePlanePos);
		ST_Vector3 closestPointOnCircle = sphereTraceVector3AddAndScale(pBowlCollider->position, bowlCirclePlaneDir, pBowlCollider->radius);
		ST_Vector3 dEdge = sphereTraceVector3Subtract(imposedPosition, closestPointOnCircle);
		float dist = sphereTraceVector3Length(dEdge);
		if (dist <= imposedRadius)
		{
			pContact->collisionType = ST_COLLISION_EDGE;
			pContact->pOtherCollider = pBowlCollider;
			pContact->otherColliderType = COLLIDER_BOWL;
			pContact->normal = sphereTraceDirectionConstruct(sphereTraceVector3Scale(dEdge, 1.0f / dist), 1);
			pContact->point = closestPointOnCircle;
			pContact->penetrationDistance = imposedRadius - dist;
			return 1;
		}
	}

	return 0;
}


b32 sphereTraceColliderBowlSphereCollisionTest(ST_BowlCollider* const pBowlCollider, ST_SphereCollider* const pSphere, ST_SphereContact* pContact)
{
	if (sphereTraceColliderBowlImposedSphereCollisionTest(pBowlCollider, pSphere->rigidBody.position, pSphere->radius, pContact))
	{
		pContact->pSphereCollider = pSphere;
		return 1;
	}
	return 0;
}


void sphereTraceColliderBowlsSetAABB(ST_BowlCollider* const pBowlCollider)
{
	ST_Vector3 centroid = pBowlCollider->position;
	pBowlCollider->aabb.halfExtents = sphereTraceVector3UniformSize(pBowlCollider->radius);
	pBowlCollider->aabb.rightTopForwardsTransformedVertex = sphereTraceVector3Add(centroid, pBowlCollider->aabb.halfExtents);
	pBowlCollider->aabb.leftDownBackTransformedVertex = sphereTraceVector3Subtract(centroid, pBowlCollider->aabb.halfExtents);
}