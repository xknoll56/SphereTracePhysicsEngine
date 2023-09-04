#include "SphereTraceColliderTerrain.h"
#include "SphereTraceColliderPlane.h"
#include "SphereTraceColliderSphere.h"
#include "SphereTraceColliderTriangle.h"
#include "SphereTraceGlobals.h"

ST_UniformTerrainCollider sphereTraceColliderUniformTerrainConstruct(int xCells, int zCells, float cellSize)
{
	ST_UniformTerrainCollider terrain;
	terrain.xCells = xCells;
	terrain.zCells = zCells;
	terrain.cellSize = cellSize;
	terrain.xSize = xCells * cellSize;
	terrain.zSize = zCells * cellSize;
	terrain.triangles = (ST_TriangleCollider*)malloc(2 * xCells * 2 * zCells * sizeof(ST_TriangleCollider));
	terrain.aabb.halfExtents = sphereTraceVector3Construct(xCells * cellSize * 0.5f, 0.0f, zCells * cellSize * 0.5f);
	terrain.aabb.leftDownBackTransformedVertex = sphereTraceVector3Construct(0.0f, 0.0f, 0.0f);
	terrain.aabb.rightTopForwardsTransformedVertex = sphereTraceVector3Construct(xCells * cellSize, 0.0f, zCells * cellSize);
	ST_Vector3 midpoint = sphereTraceVector3Average(terrain.aabb.leftDownBackTransformedVertex, terrain.aabb.rightTopForwardsTransformedVertex);
	terrain.rightPlane = sphereTraceColliderPlaneConstruct(gVector3Right, 0.0f, 0.0f, terrain.aabb.halfExtents.z, sphereTraceVector3Construct(0.0f, midpoint.y, midpoint.z));
	terrain.topPlane = sphereTraceColliderPlaneConstruct(gVector3Up, 0.0f, terrain.aabb.halfExtents.x, terrain.aabb.halfExtents.z, sphereTraceVector3Construct(midpoint.x, midpoint.y, midpoint.z));
	terrain.forwardPlane = sphereTraceColliderPlaneConstruct(gVector3Forward, 0.0f, terrain.aabb.halfExtents.x, 0.0f, sphereTraceVector3Construct(midpoint.x, midpoint.y, 0.0f));
	terrain.angle = 0.0f;
	terrain.right = gVector3Right;
	terrain.forward = gVector3Forward;
	terrain.position = gVector3Zero;
	terrain.rotation = gQuaternionIdentity;
	return terrain;
}

void sphereTraceColliderUniformTerrainSetTransform(ST_UniformTerrainCollider* const pTerrainCollider, float angle, ST_Vector3 position)
{

	pTerrainCollider->angle = angle;
	pTerrainCollider->right = sphereTraceVector3Construct(cosf(angle), 0.0f, -sinf(angle));
	pTerrainCollider->forward = sphereTraceVector3Construct(sinf(angle), 0.0f, cosf(angle));
	pTerrainCollider->rotation = sphereTraceQuaternionFromAngleAxis(gVector3Up, angle);
	pTerrainCollider->aabb.halfExtents = sphereTraceVector3Construct(0.0f, 0.0f, 0.0f);
	pTerrainCollider->aabb.leftDownBackTransformedVertex = position;
	pTerrainCollider->aabb.rightTopForwardsTransformedVertex = position;
	for (int z = 0; z < pTerrainCollider->zCells; z++)
	{
		for (int x = 0; x < pTerrainCollider->xCells; x++)
		{
			int triangleIndex = 2 * x + 2 * z * pTerrainCollider->xCells;
			ST_TriangleCollider* pTC = &pTerrainCollider->triangles[triangleIndex];
			ST_Vector3 v1 = sphereTraceVector3Subtract(pTC->transformedVertices[0], pTerrainCollider->position);
			ST_Vector3 v2 = sphereTraceVector3Subtract(pTC->transformedVertices[1], pTerrainCollider->position);
			ST_Vector3 v3 = sphereTraceVector3Subtract(pTC->transformedVertices[2], pTerrainCollider->position);
			v1 = sphereTraceVector3Construct(cosf(angle) * v1.x + sinf(angle) * v1.z, v1.y, -sinf(angle) * v1.x + cosf(angle) * v1.z);
			v2 = sphereTraceVector3Construct(cosf(angle) * v2.x + sinf(angle) * v2.z, v2.y, -sinf(angle) * v2.x + cosf(angle) * v2.z);
			v3 = sphereTraceVector3Construct(cosf(angle) * v3.x + sinf(angle) * v3.z, v3.y, -sinf(angle) * v3.x + cosf(angle) * v3.z);
			sphereTraceColliderTriangleSetVertexAndEdgeData(pTC, sphereTraceVector3Add(v1, position), sphereTraceVector3Add(v2, position), sphereTraceVector3Add(v3, position));
			//sphereTraceColliderTriangleSetAABB(pTC);
			sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&pTerrainCollider->aabb, &pTerrainCollider->triangles[triangleIndex].aabb);

			triangleIndex++;
			pTC = &pTerrainCollider->triangles[triangleIndex];
			v1 = sphereTraceVector3Subtract(pTC->transformedVertices[0], pTerrainCollider->position);
			v2 = sphereTraceVector3Subtract(pTC->transformedVertices[1], pTerrainCollider->position);
			v3 = sphereTraceVector3Subtract(pTC->transformedVertices[2], pTerrainCollider->position);
			v1 = sphereTraceVector3Construct(cosf(angle) * v1.x + sinf(angle) * v1.z, v1.y, -sinf(angle) * v1.x + cosf(angle) * v1.z);
			v2 = sphereTraceVector3Construct(cosf(angle) * v2.x + sinf(angle) * v2.z, v2.y, -sinf(angle) * v2.x + cosf(angle) * v2.z);
			v3 = sphereTraceVector3Construct(cosf(angle) * v3.x + sinf(angle) * v3.z, v3.y, -sinf(angle) * v3.x + cosf(angle) * v3.z);
			sphereTraceColliderTriangleSetVertexAndEdgeData(pTC, sphereTraceVector3Add(v1, position), sphereTraceVector3Add(v2, position), sphereTraceVector3Add(v3, position)); sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&pTerrainCollider->aabb, &pTerrainCollider->triangles[triangleIndex].aabb);
			//sphereTraceColliderTriangleSetAABB(pTC);
			sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&pTerrainCollider->aabb, &pTerrainCollider->triangles[triangleIndex].aabb);
		}
	}
	pTerrainCollider->position = position;
	ST_Vector3 midpoint = sphereTraceVector3Construct(pTerrainCollider->xCells * pTerrainCollider->cellSize * 0.5f, 0.5f * (pTerrainCollider->aabb.leftDownBackTransformedVertex.y + pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y), pTerrainCollider->zCells * pTerrainCollider->cellSize * 0.5f);
	midpoint = sphereTraceVector3Add(position, sphereTraceVector3Construct(cosf(angle) * midpoint.x + sinf(angle) * midpoint.z, midpoint.y, -sinf(angle) * midpoint.x + cosf(angle) * midpoint.z));
	ST_Vector3 endpoint = sphereTraceVector3Construct(pTerrainCollider->xCells * pTerrainCollider->cellSize, pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y, pTerrainCollider->zCells * pTerrainCollider->cellSize);
	endpoint = sphereTraceVector3Add(position, sphereTraceVector3Construct(cosf(angle) * endpoint.x + sinf(angle) * endpoint.z, endpoint.y, -sinf(angle) * endpoint.x + cosf(angle) * endpoint.z));
	pTerrainCollider->leftPlane = sphereTraceColliderPlaneConstruct(sphereTraceVector3Negative(pTerrainCollider->right), 0.0f, pTerrainCollider->leftPlane.xHalfExtent, pTerrainCollider->leftPlane.zHalfExtent, sphereTraceVector3Add(position, sphereTraceVector3Construct(cosf(angle) * pTerrainCollider->leftPlane.position.x + sinf(angle) * pTerrainCollider->leftPlane.position.z, pTerrainCollider->leftPlane.position.y, -sinf(angle) * pTerrainCollider->leftPlane.position.x + cosf(angle) * pTerrainCollider->leftPlane.position.z)));
	pTerrainCollider->topPlane = sphereTraceColliderPlaneConstruct(gVector3Up, angle, pTerrainCollider->topPlane.xHalfExtent, pTerrainCollider->topPlane.zHalfExtent, sphereTraceVector3Construct(midpoint.x, pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y, midpoint.z));
	pTerrainCollider->backPlane = sphereTraceColliderPlaneConstruct(sphereTraceVector3Negative(pTerrainCollider->forward), 0.0f, pTerrainCollider->backPlane.zHalfExtent, pTerrainCollider->backPlane.xHalfExtent, sphereTraceVector3Add(position, sphereTraceVector3Construct(cosf(angle) * pTerrainCollider->backPlane.position.x + sinf(angle) * pTerrainCollider->backPlane.position.z, pTerrainCollider->backPlane.position.y, -sinf(angle) * pTerrainCollider->backPlane.position.x + cosf(angle) * pTerrainCollider->backPlane.position.z)));
	pTerrainCollider->rightPlane = sphereTraceColliderPlaneConstruct(pTerrainCollider->right, 0.0f, pTerrainCollider->rightPlane.xHalfExtent, pTerrainCollider->rightPlane.zHalfExtent, sphereTraceVector3Add(position, sphereTraceVector3Construct(cosf(angle) * pTerrainCollider->rightPlane.position.x + sinf(angle) * pTerrainCollider->rightPlane.position.z, pTerrainCollider->rightPlane.position.y, -sinf(angle) * pTerrainCollider->rightPlane.position.x + cosf(angle) * pTerrainCollider->rightPlane.position.z)));
	pTerrainCollider->bottomPlane = sphereTraceColliderPlaneConstruct(gVector3Down, angle, pTerrainCollider->bottomPlane.xHalfExtent, pTerrainCollider->bottomPlane.zHalfExtent, sphereTraceVector3Construct(midpoint.x, pTerrainCollider->aabb.leftDownBackTransformedVertex.y, midpoint.z));
	pTerrainCollider->forwardPlane = sphereTraceColliderPlaneConstruct(pTerrainCollider->forward, 0.0f, pTerrainCollider->forwardPlane.zHalfExtent, pTerrainCollider->forwardPlane.xHalfExtent, sphereTraceVector3Add(position, sphereTraceVector3Construct(cosf(angle) * pTerrainCollider->forwardPlane.position.x + sinf(angle) * pTerrainCollider->forwardPlane.position.z, pTerrainCollider->forwardPlane.position.y, -sinf(angle) * pTerrainCollider->forwardPlane.position.x + cosf(angle) * pTerrainCollider->forwardPlane.position.z)));


}

void sphereTraceColliderUniformTerrainFillTrianglesWithFunction(ST_UniformTerrainCollider* const terrainCollider, float (*fxz)(float, float))
{
	for (int z = 0; z < terrainCollider->zCells; z++)
	{
		for (int x = 0; x < terrainCollider->xCells; x++)
		{
			int triangleIndex = 2 * x + 2 * z * terrainCollider->xCells;
			//(0,0), (1,0), (0,1)
			float x1 = x * terrainCollider->cellSize;
			float x2 = (x + 1) * terrainCollider->cellSize;
			float z1 = z * terrainCollider->cellSize;
			float z2 = (z + 1) * terrainCollider->cellSize;
			terrainCollider->triangles[triangleIndex] = sphereTraceColliderTriangleConstruct(sphereTraceVector3Construct(x1, fxz(x1, z1), z1),
				sphereTraceVector3Construct(
					x1, fxz(x1, z2), z2
				),
				sphereTraceVector3Construct(
					x2, fxz(x2, z1), z1
				));
			sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&terrainCollider->aabb, &terrainCollider->triangles[triangleIndex].aabb);
			triangleIndex++;
			//(1,0), (1,1), (0,1)
			terrainCollider->triangles[triangleIndex] = sphereTraceColliderTriangleConstruct(sphereTraceVector3Construct(x2, fxz(x2, z1), z1),
				sphereTraceVector3Construct(
					x1, fxz(x1, z2), z2
				),
				sphereTraceVector3Construct(
					x2, fxz(x2, z2), z2
				));
			sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&terrainCollider->aabb, &terrainCollider->triangles[triangleIndex].aabb);
		}
	}

	ST_Vector3 midpoint = sphereTraceVector3Average(terrainCollider->aabb.leftDownBackTransformedVertex, terrainCollider->aabb.rightTopForwardsTransformedVertex);
	terrainCollider->leftPlane = sphereTraceColliderPlaneConstruct(sphereTraceVector3Negative(gVector3Right), 0.0f, terrainCollider->aabb.halfExtents.y, terrainCollider->aabb.halfExtents.z, sphereTraceVector3Construct(0.0f, midpoint.y, midpoint.z));
	terrainCollider->topPlane = sphereTraceColliderPlaneConstruct(gVector3Up, 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.z, sphereTraceVector3Construct(midpoint.x, midpoint.y, midpoint.z));
	terrainCollider->backPlane = sphereTraceColliderPlaneConstruct(sphereTraceVector3Negative(gVector3Forward), 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.y, sphereTraceVector3Construct(midpoint.x, midpoint.y, 0.0f));
	terrainCollider->rightPlane = sphereTraceColliderPlaneConstruct(gVector3Right, 0.0f, terrainCollider->aabb.halfExtents.y, terrainCollider->aabb.halfExtents.z, sphereTraceVector3Construct(terrainCollider->aabb.rightTopForwardsTransformedVertex.x, midpoint.y, midpoint.z));
	terrainCollider->bottomPlane = sphereTraceColliderPlaneConstruct(gVector3Down, 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.z, sphereTraceVector3Construct(midpoint.x, 0.0f, midpoint.z));
	terrainCollider->forwardPlane = sphereTraceColliderPlaneConstruct(gVector3Forward, 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.y, sphereTraceVector3Construct(midpoint.x, midpoint.y, terrainCollider->aabb.rightTopForwardsTransformedVertex.z));

}

void sphereTraceColliderUniformTerrainFillTrianglesWithFunctionAndConditionalFunction(ST_UniformTerrainCollider* const terrainCollider, float (*fxz)(float, float), b32(*conditionalFunc)(float (*fxz)(float, float), float, float))
{
	for (int z = 0; z < terrainCollider->zCells; z++)
	{
		for (int x = 0; x < terrainCollider->xCells; x++)
		{
			int triangleIndex = 2 * x + 2 * z * terrainCollider->xCells;
			//(0,0), (1,0), (0,1)
			float x1 = x * terrainCollider->cellSize;
			float x2 = (x + 1) * terrainCollider->cellSize;
			float z1 = z * terrainCollider->cellSize;
			float z2 = (z + 1) * terrainCollider->cellSize;
			if (conditionalFunc(fxz, x1, z1))
			{
				terrainCollider->triangles[triangleIndex] = sphereTraceColliderTriangleConstruct(sphereTraceVector3Construct(x1, fxz(x1, z1), z1),
					sphereTraceVector3Construct(
						x1, fxz(x1, z2), z2
					),
					sphereTraceVector3Construct(
						x2, fxz(x2, z1), z1
					));
				sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&terrainCollider->aabb, &terrainCollider->triangles[triangleIndex].aabb);
				triangleIndex++;
				//(1,0), (1,1), (0,1)
				terrainCollider->triangles[triangleIndex] = sphereTraceColliderTriangleConstruct(sphereTraceVector3Construct(x2, fxz(x2, z1), z1),
					sphereTraceVector3Construct(
						x1, fxz(x1, z2), z2
					),
					sphereTraceVector3Construct(
						x2, fxz(x2, z2), z2
					));
				sphereTraceColliderAABBResizeAABBToContainAnotherAABB(&terrainCollider->aabb, &terrainCollider->triangles[triangleIndex].aabb);
			}
			else
			{
				terrainCollider->triangles[triangleIndex] = sphereTraceColliderTriangleConstruct(terrainCollider->position, terrainCollider->position, terrainCollider->position);
				terrainCollider->triangles[triangleIndex++].ignoreCollisions = 1;
				terrainCollider->triangles[triangleIndex] = sphereTraceColliderTriangleConstruct(terrainCollider->position, terrainCollider->position, terrainCollider->position);
				terrainCollider->triangles[triangleIndex].ignoreCollisions = 1;
			}
		}
	}

	ST_Vector3 midpoint = sphereTraceVector3Average(terrainCollider->aabb.leftDownBackTransformedVertex, terrainCollider->aabb.rightTopForwardsTransformedVertex);
	terrainCollider->leftPlane = sphereTraceColliderPlaneConstruct(sphereTraceVector3Negative(gVector3Right), 0.0f, terrainCollider->aabb.halfExtents.y, terrainCollider->aabb.halfExtents.z, sphereTraceVector3Construct(0.0f, midpoint.y, midpoint.z));
	terrainCollider->topPlane = sphereTraceColliderPlaneConstruct(gVector3Up, 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.z, sphereTraceVector3Construct(midpoint.x, midpoint.y, midpoint.z));
	terrainCollider->backPlane = sphereTraceColliderPlaneConstruct(sphereTraceVector3Negative(gVector3Forward), 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.y, sphereTraceVector3Construct(midpoint.x, midpoint.y, 0.0f));
	terrainCollider->rightPlane = sphereTraceColliderPlaneConstruct(gVector3Right, 0.0f, terrainCollider->aabb.halfExtents.y, terrainCollider->aabb.halfExtents.z, sphereTraceVector3Construct(terrainCollider->aabb.rightTopForwardsTransformedVertex.x, midpoint.y, midpoint.z));
	terrainCollider->bottomPlane = sphereTraceColliderPlaneConstruct(gVector3Down, 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.z, sphereTraceVector3Construct(midpoint.x, 0.0f, midpoint.z));
	terrainCollider->forwardPlane = sphereTraceColliderPlaneConstruct(gVector3Forward, 0.0f, terrainCollider->aabb.halfExtents.x, terrainCollider->aabb.halfExtents.y, sphereTraceVector3Construct(midpoint.x, midpoint.y, terrainCollider->aabb.rightTopForwardsTransformedVertex.z));
}

int sphereTraceColliderUniformTerrainSampleFirstTriangleIndex(const ST_UniformTerrainCollider* const terrainCollider, ST_Vector3 samplePosition)
{
	//if (sphereTraceColliderAABBIsPointInside(&terrainCollider->aabb, (ST_Vector3) { samplePosition.x, terrainCollider->aabb.leftDownBackTransformedVertex.y, samplePosition.z }))
	//{
	float dpx = samplePosition.x - terrainCollider->position.x;
	float dpz = samplePosition.z - terrainCollider->position.z;
	float xDist = (dpx * terrainCollider->right.x + dpz * terrainCollider->right.z);
	float zDist = (dpx * terrainCollider->forward.x + dpz * terrainCollider->forward.z);
	if (xDist <= terrainCollider->xSize && zDist <= terrainCollider->zSize && xDist >= 0.0f && zDist >= 0.0f)
	{
		int x = (int)(xDist / terrainCollider->cellSize);
		int z = (int)(zDist / terrainCollider->cellSize);
		return 2 * x + 2 * z * terrainCollider->xCells;
	}
	else
		return -1;
	//}
	//else
	//{
	//	return -1;
	//}
}

ST_IntList sphereTraceColliderUniformTerrainSampleTriangleIndicesForSphere(const ST_UniformTerrainCollider* const terrainCollider, ST_Vector3 spherePosition, float radius)
{
	float maxCheckDistRight = radius * 2.0f + terrainCollider->cellSize;
	float maxCheckDistFwd = radius * 2.0f + terrainCollider->cellSize;
	//ST_Vector2 maxCheckPos = { paabb->rightTopForwardsTransformedVertex.x + terrainCollider->cellSize, paabb->rightTopForwardsTransformedVertex.z + terrainCollider->cellSize };
	ST_IntList indices = sphereTraceIntListConstruct();
	float distRight = 0.0f;
	float distFwd = 0.0f;
	ST_Vector3 posStart = sphereTraceVector3AddAndScale(sphereTraceVector3AddAndScale(spherePosition, terrainCollider->leftPlane.normal, radius), terrainCollider->backPlane.normal, radius);
	for (float distFwd = 0.0f; distFwd < maxCheckDistFwd; distFwd += terrainCollider->cellSize)
	{
		for (float distRight = 0.0f; distRight < maxCheckDistRight; distRight += terrainCollider->cellSize)
		{
			ST_Vector3 pos = sphereTraceVector3AddAndScale(sphereTraceVector3AddAndScale(posStart, terrainCollider->rightPlane.normal, distRight), terrainCollider->forwardPlane.normal, distFwd);
			int index = sphereTraceColliderUniformTerrainSampleFirstTriangleIndex(terrainCollider, sphereTraceVector3Construct(pos.x, terrainCollider->aabb.leftDownBackTransformedVertex.y, pos.z));
			if (index != -1)
			{
				sphereTraceIntListAddFirst(&indices, index);
				sphereTraceIntListAddFirst(&indices, index + 1);
			}

		}
	}
	return indices;
}


b32 sphereTraceColliderUniformTerrainImposedSphereFindMaxPenetratingTriangle(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 imposedPosition, float imposedRadius, ST_SphereTerrainTriangleContactInfo* const pContactInfo)
{
	ST_IntListData* pIl;
	ST_AABB imposedAABB;
	imposedAABB.halfExtents = sphereTraceVector3Construct(imposedRadius, imposedRadius, imposedRadius);
	imposedAABB.rightTopForwardsTransformedVertex = sphereTraceVector3Add(imposedPosition, imposedAABB.halfExtents);
	imposedAABB.leftDownBackTransformedVertex = sphereTraceVector3Subtract(imposedPosition, imposedAABB.halfExtents);
	ST_IntList il = sphereTraceColliderUniformTerrainSampleTriangleIndicesForSphere(pTerrainCollider, imposedPosition, imposedRadius);
	pIl = il.pFirst;
	//float maxPen = -FLT_MAX;
	ST_IntList penetratingIndices = sphereTraceIntListConstruct();
	for (int i = 0; i < il.count; i++)
	{

		int index = pIl->value;
		if (!pTerrainCollider->triangles[index].ignoreCollisions)
		{
			if (sphereTraceColliderAABBIntersectAABBVertically(&imposedAABB, &pTerrainCollider->triangles[index].aabb))
			{
				if (sphereTraceColliderAABBIntersectImposedSphere(&pTerrainCollider->triangles[index].aabb, imposedPosition, imposedRadius))
				{
					ST_SphereTriangleContactInfo ciTest;
					if (sphereTraceColliderTriangleImposedSphereCollisionTest(&pTerrainCollider->triangles[index], imposedPosition, imposedRadius, &ciTest))
					{
						sphereTraceIntListAddFirst(&penetratingIndices, index);
						//if (ciTest.penetrationDistance > maxPen)
						//{
						//	maxPen = ciTest.penetrationDistance;
						//	*pContactInfo = ciTest;
						//}
					}
				}
			}
		}
		pIl = pIl->pNext;
	}
	sphereTraceIntListFree(&il);
	if (penetratingIndices.count > 0)
	{
		//now find the point at which the triangle is contacted with the sphere on the y axis
		pIl = penetratingIndices.pFirst;
		ST_Vector3 castPoint = sphereTraceVector3Construct(imposedPosition.x, pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y + imposedRadius, imposedPosition.z);
		float minDist = FLT_MAX;
		int minIndex;
		ST_SphereTraceData std;
		for (int i = 0; i < penetratingIndices.count; i++)
		{
			if (sphereTraceColliderTriangleSphereTrace(castPoint, gVector3Down, imposedRadius, &pTerrainCollider->triangles[pIl->value], &std))
			{
				float height = castPoint.y - std.sphereCenter.y;
				if (height < minDist)
				{
					minDist = height;
					minIndex = pIl->value;
					pContactInfo->downSphereTraceData = std;
					pContactInfo->downSphereTraceData.traceDistance = 0.0f;
				}
			}

			pIl = pIl->pNext;
		}
		//now recollect the contact info for the sphere-triangle
		sphereTraceColliderTriangleImposedSphereCollisionTest(&pTerrainCollider->triangles[minIndex], imposedPosition, imposedRadius, &pContactInfo->sphereTriangleContactInfo);
		sphereTraceIntListFree(&penetratingIndices);
		return 1;
	}
	else
	{
		return 0;
	}
}

b32 sphereTraceColliderUniformTerrainSphereFindMaxPenetratingTriangle(const ST_UniformTerrainCollider* const pTerrainCollider, ST_SphereCollider* const pSphereCollider, ST_SphereTerrainTriangleContactInfo* const pContactInfo)
{
	b32 res = sphereTraceColliderUniformTerrainImposedSphereFindMaxPenetratingTriangle(pTerrainCollider, pSphereCollider->pRigidBody->position, pSphereCollider->radius, pContactInfo);
	if (res)
	{
		pContactInfo->sphereTriangleContactInfo.pSphereCollider = pSphereCollider;
	}
	return res;
}





b32 sphereTraceColliderUniformTerrainRayTrace(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 from, ST_Vector3 dir, ST_RayTraceData* const pRayTraceData)
{
	dir = sphereTraceVector3Normalize(dir);
	if (dir.x == 0.0f && dir.z == 0.0f)
	{
		int triangleInd = sphereTraceColliderUniformTerrainSampleFirstTriangleIndex(pTerrainCollider, from);
		if (triangleInd != -1)
		{
			if (sphereTraceColliderTriangleRayTrace(from, dir, &pTerrainCollider->triangles[triangleInd], pRayTraceData))
			{
				return 1;
			}
			else if (sphereTraceColliderTriangleRayTrace(from, dir, &pTerrainCollider->triangles[triangleInd + 1], pRayTraceData))
			{
				return 1;
			}
		}
		return 0;
	}
	ST_RayTraceData rtcPlane;
	b32 doesIntersectWithTerrainBoundingBox = 0;
	b32 xIncrementing = 0;
	b32 yIncrementing = 0;
	b32 zIncrementing = 0;
	float magX = sphereTraceVector3Dot(dir, pTerrainCollider->rightPlane.normal);
	float magY = sphereTraceVector3Dot(dir, pTerrainCollider->topPlane.normal);
	float magZ = sphereTraceVector3Dot(dir, pTerrainCollider->forwardPlane.normal);
	ST_Vector3 intersectingDirection;
	ST_Vector3 intersection = from;
	ST_Vector3 dp = sphereTraceVector3Subtract(from, pTerrainCollider->position);
	float dpx = intersection.x - pTerrainCollider->position.x;
	float dpz = intersection.z - pTerrainCollider->position.z;
	float xDist = (dpx * pTerrainCollider->right.x + dpz * pTerrainCollider->right.z);
	float zDist = (dpx * pTerrainCollider->forward.x + dpz * pTerrainCollider->forward.z);
	float height = intersection.y;
	if (xDist <= pTerrainCollider->xSize && zDist <= pTerrainCollider->zSize && height <= pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y && xDist >= 0.0f && zDist >= 0.0f && height >= pTerrainCollider->aabb.leftDownBackTransformedVertex.y)
	{
		doesIntersectWithTerrainBoundingBox = 1;
		if (magX > 0.0f)
			xIncrementing = 1;
		if (magY > 0.0f)
			yIncrementing = 1;
		if (magZ > 0.0f)
			zIncrementing = 1;
	}
	else
	{
		if (magX <= 0.0f)
		{
			if (sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->rightPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		else
		{
			xIncrementing = 1;
			if (sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->leftPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		if (magY <= 0.0f)
		{
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->topPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		else
		{
			yIncrementing = 1;
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->bottomPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		if (magZ < 0.0f)
		{
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->forwardPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		else
		{
			zIncrementing = 1;
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->backPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
	}
	if (doesIntersectWithTerrainBoundingBox)
	{
		dpx = intersection.x - pTerrainCollider->position.x;
		dpz = intersection.z - pTerrainCollider->position.z;
		xDist = (dpx * pTerrainCollider->right.x + dpz * pTerrainCollider->right.z);
		zDist = (dpx * pTerrainCollider->forward.x + dpz * pTerrainCollider->forward.z);
		height = intersection.y;
		float tMin = FLT_MAX;
		b32 xChanged = 0;
		b32 zChanged = 0;
		ST_AABB heightBox;
		int xInd = (int)(xDist / pTerrainCollider->cellSize);
		int zInd = (int)(zDist / pTerrainCollider->cellSize);
		while (xDist <= pTerrainCollider->xSize && zDist <= pTerrainCollider->zSize && height <= pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y && xDist >= 0.0f && zDist >= 0.0f && height >= pTerrainCollider->aabb.leftDownBackTransformedVertex.y)
		{


			b32 incomingFromX = 1;
			if (xIncrementing)
			{
				if (xChanged)
					xInd++;
				else
					xInd = (int)(xDist / pTerrainCollider->cellSize) + 1;
				float tentativeX = (xInd * pTerrainCollider->cellSize - xDist) / magX;
				if (tentativeX < tMin && tentativeX > 0.0f)
				{
					tMin = tentativeX;
					intersectingDirection = pTerrainCollider->rightPlane.normal;
				}
			}
			else
			{
				if (xChanged)
					xInd--;
				else
					xInd = (int)(xDist / pTerrainCollider->cellSize);
				float tentativeX = (xInd * pTerrainCollider->cellSize - xDist) / magX;
				if (tentativeX < tMin && tentativeX > 0.0f)
				{

					tMin = tentativeX;
					intersectingDirection = pTerrainCollider->leftPlane.normal;
				}
			}

			if (zIncrementing)
			{
				if (zChanged)
					zInd++;
				else
					zInd = (int)(zDist / pTerrainCollider->cellSize) + 1;
				float tentativeT = (zInd * pTerrainCollider->cellSize - zDist) / magZ;
				if (tentativeT < tMin && tentativeT > 0.0f)
				{

					tMin = tentativeT;
					intersectingDirection = pTerrainCollider->forwardPlane.normal;
					incomingFromX = 0;
				}
			}
			else
			{
				if (zChanged)
					zInd--;
				else
					zInd = (int)(zDist / pTerrainCollider->cellSize);
				float tentativeT = (zInd * pTerrainCollider->cellSize - zDist) / magZ;
				if (tentativeT < tMin && tentativeT > 0.0f)
				{

					tMin = tentativeT;
					intersectingDirection = pTerrainCollider->backPlane.normal;
					incomingFromX = 0;
				}
			}
			ST_Vector3 nextIntersection = sphereTraceVector3AddAndScale(intersection, dir, tMin);
			//need to adjust the intersection so we dont get stuck with floating point error
			if (incomingFromX)
			{
				xChanged = 1;
				xDist = xInd * pTerrainCollider->cellSize;
				dpx = nextIntersection.x - pTerrainCollider->position.x;
				dpz = nextIntersection.z - pTerrainCollider->position.z;
				zDist = (dpx * pTerrainCollider->forward.x + dpz * pTerrainCollider->forward.z);
			}
			else
			{
				zChanged = 1;
				dpx = nextIntersection.x - pTerrainCollider->position.x;
				dpz = nextIntersection.z - pTerrainCollider->position.z;
				zDist = zInd * pTerrainCollider->cellSize;
				xDist = (dpx * pTerrainCollider->right.x + dpz * pTerrainCollider->right.z);
			}
			float nextHeight = nextIntersection.y;
			if (yIncrementing)
			{
				heightBox.rightTopForwardsTransformedVertex.y = nextHeight;
				heightBox.leftDownBackTransformedVertex.y = height;
			}
			else
			{
				heightBox.rightTopForwardsTransformedVertex.y = height;
				heightBox.leftDownBackTransformedVertex.y = nextHeight;
			}
			ST_Vector3 samplePos = sphereTraceVector3Average(intersection, nextIntersection);
			int triIndex = sphereTraceColliderUniformTerrainSampleFirstTriangleIndex(pTerrainCollider, samplePos);
			if (triIndex != -1)
			{
				ST_TriangleCollider* pt1 = &pTerrainCollider->triangles[triIndex];
				ST_TriangleCollider* pt2 = &pTerrainCollider->triangles[triIndex + 1];

				if (sphereTraceColliderAABBIntersectAABBVertically(&pt1->aabb, &heightBox))
				{
					if (sphereTraceColliderTriangleRayTrace(from, dir, pt1, pRayTraceData))
					{
						return 1;
					}
				}
				if (sphereTraceColliderAABBIntersectAABBVertically(&pt2->aabb, &heightBox))
				{
					if (sphereTraceColliderTriangleRayTrace(from, dir, pt2, pRayTraceData))
					{
						return 1;
					}
				}
			}

			intersection = nextIntersection;
			height = intersection.y;
			tMin = FLT_MAX;


		}
	}
	return 0;
}

b32 sphereTraceColliderUniformTerrainSphereTraceDown(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 from, float radius, ST_SphereTraceData* const pSphereTraceData)
{
	//ST_AABB testAABB;
	//testAABB.halfExtents = (ST_Vector3){ radius, radius, radius };
	//testAABB.rightTopForwardsTransformedVertex = sphereTraceVector3Add(from, testAABB.halfExtents);
	//testAABB.leftDownBackTransformedVertex = sphereTraceVector3Subtract(from, testAABB.halfExtents);
	ST_IntList triangleIndices = sphereTraceColliderUniformTerrainSampleTriangleIndicesForSphere(pTerrainCollider, from, radius);
	ST_IntListData* pild = triangleIndices.pFirst;
	float maxHeight = -FLT_MAX;
	ST_SphereTraceData dataTest;
	int triangleMin = 0;
	for (int i = 0; i < triangleIndices.count; i++)
	{
		if (sphereTraceColliderTriangleSphereTrace(from, gVector3Down, radius, &pTerrainCollider->triangles[pild->value], &dataTest))
		{
			//rendererDrawSphere(dataTest.rayTraceData.hitPoint, (ST_Vector3) { 0.1f, 0.1f, 0.1f }, gQuaternionIdentity, gVector4ColorRed);
			//sceneDrawTriangleOutline(&pTerrainCollider->triangles[pild->value], gVector4ColorGreen);
			if (dataTest.sphereCenter.y > maxHeight)
			{
				maxHeight = dataTest.sphereCenter.y;
				*pSphereTraceData = dataTest;
				triangleMin = pild->value;
			}
		}
		//if (sphereTraceColliderTriangleSphereTrace(from, gVector3Down, radius, &pTerrainCollider->triangles[pild->value+1], &dataTest))
		//{
		//	rendererDrawSphere(dataTest.rayTraceData.hitPoint, (ST_Vector3) { 0.1f, 0.1f, 0.1f }, gQuaternionIdentity, gVector4ColorRed);
		//	sceneDrawTriangleOutline(&pTerrainCollider->triangles[pild->value+1], gVector4ColorGreen);
		//	float sphereDist = sphereTraceVector3Length(sphereTraceVector3Subtract(dataTest.sphereCenter, from));
		//	if (dataTest.sphereCenter.y > maxHeight)
		//	{
		//		maxHeight = dataTest.sphereCenter.y;
		//		*pSphereTraceData = dataTest;
		//		triangleMin = pild->value+1;
		//	}
		//}
		pild = pild->pNext;
	}
	if (maxHeight > -FLT_MAX)
	{
		sphereTraceColliderTriangleSphereTrace(from, gVector3Down, radius, &pTerrainCollider->triangles[triangleMin], &dataTest);
		return 1;
	}

	return 0;
}

ST_IntList sphereTraceColliderUniformTerrainSampleTrianglesIndicesForSphereTrace(const ST_UniformTerrainCollider* const terrainCollider, ST_SphereTraceData* const pSphereTraceData)
{
	float minRight, minForward, dx, dz;
	float maxCheckDistRight;
	float maxCheckDistFwd;
	float rightDot1 = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereTraceData->rayTraceData.startPoint, terrainCollider->position), terrainCollider->right);
	float rightDot2 = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereTraceData->sphereCenter, terrainCollider->position), terrainCollider->right);
	float fwdDot1 = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereTraceData->rayTraceData.startPoint, terrainCollider->position), terrainCollider->forward);
	float fwdDot2 = sphereTraceVector3Dot(sphereTraceVector3Subtract(pSphereTraceData->sphereCenter, terrainCollider->position), terrainCollider->forward);
	ST_Vector3 posStart;
	ST_Vector3 dir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pSphereTraceData->sphereCenter, pSphereTraceData->rayTraceData.startPoint));
	ST_Vector3 sphereRight = sphereTraceVector3Normalize(sphereTraceVector3Cross(gVector3Up, dir));
	ST_Vector3 rightRadius = sphereTraceVector3Scale(sphereRight, pSphereTraceData->radius);
	ST_Vector3 sphereForward = sphereTraceVector3Normalize(sphereTraceVector3Construct(dir.x, 0.0f, dir.z));
	ST_Vector3 forwardRadius = sphereTraceVector3Scale(sphereForward, pSphereTraceData->radius);
	float rightRight = fabsf(sphereTraceVector3Dot(rightRadius, terrainCollider->rightPlane.normal));
	float rightForward = fabsf(sphereTraceVector3Dot(rightRadius, terrainCollider->forwardPlane.normal));
	float forwardRight = fabsf(sphereTraceVector3Dot(forwardRadius, terrainCollider->rightPlane.normal));
	float forwardForward = fabsf(sphereTraceVector3Dot(forwardRadius, terrainCollider->forwardPlane.normal));
	float signX = 1.0f;
	float signZ = 1.0f;
	if (rightDot1 <= rightDot2)
	{
		minRight = rightDot1;
		minRight -= rightRight;
		dx = rightDot2 - minRight;
		maxCheckDistRight = dx + rightRight + forwardRight + terrainCollider->cellSize;
	}
	else
	{
		minRight = rightDot2;
		minRight -= rightRight;
		dx = rightDot1 - rightDot2 + rightRight;
		maxCheckDistRight = dx + rightRight + forwardRight + terrainCollider->cellSize;
		//signX = -1.0f;
	}
	if (fwdDot1 <= fwdDot2)
	{
		minForward = fwdDot1;
		minForward -= rightForward;
		dz = fwdDot2 - minForward;
		maxCheckDistFwd = dz + rightForward + forwardForward + terrainCollider->cellSize;
	}
	else
	{
		minForward = fwdDot2;
		minForward -= rightForward;
		dz = fwdDot1 - fwdDot2 + rightForward;
		maxCheckDistFwd = dz + rightForward + forwardForward + terrainCollider->cellSize;
		//signZ = -1.0f;
	}
	float r2 = pSphereTraceData->radius * pSphereTraceData->radius;


	//ST_Vector2 maxCheckPos = { paabb->rightTopForwardsTransformedVertex.x + terrainCollider->cellSize, paabb->rightTopForwardsTransformedVertex.z + terrainCollider->cellSize };
	ST_IntList indices = sphereTraceIntListConstruct();
	float distRight = 0.0f;
	float distFwd = 0.0f;
	//float theta = acosf();
	float cTheta = sphereTraceVector3Dot(dir, sphereForward);
	posStart = sphereTraceVector3AddAndScale(sphereTraceVector3AddAndScale(terrainCollider->position, terrainCollider->right, minRight), terrainCollider->forward, minForward);

	//rendererDrawSphere(posStart, sphereTraceVector3Scale(gVector3One, 0.1f), gQuaternionIdentity, gVector4ColorGreen);
	//ST_Vector3 pos1 = sphereTraceVector3AddAndScale(posStart, terrainCollider->forwardPlane.normal, signZ * maxCheckDistFwd);
	//ST_Vector3 pos3 = sphereTraceVector3AddAndScale(posStart, terrainCollider->rightPlane.normal, signX * maxCheckDistRight);
	//ST_Vector3 pos2 = sphereTraceVector3AddAndScale(sphereTraceVector3AddAndScale(posStart, terrainCollider->rightPlane.normal, signX * maxCheckDistRight), terrainCollider->forwardPlane.normal, signZ * maxCheckDistFwd);
	//rendererDrawSphere(pos1, sphereTraceVector3Scale(gVector3One, 0.1f), gQuaternionIdentity, gVector4ColorRed);
	//rendererDrawSphere(pos2, sphereTraceVector3Scale(gVector3One, 0.1f), gQuaternionIdentity, gVector4ColorBlue);
	//rendererDrawSphere(pos3, sphereTraceVector3Scale(gVector3One, 0.1f), gQuaternionIdentity, gVector4Zero);
	//rendererDrawLineFromTo(posStart, pos1, gVector4ColorRed);
	//rendererDrawLineFromTo(pos1, pos2, gVector4ColorRed);
	//rendererDrawLineFromTo(pos2, pos3, gVector4ColorRed);
	//rendererDrawLineFromTo(pos3, posStart, gVector4ColorRed);
	for (float distFwd = 0.0f; distFwd < maxCheckDistFwd; distFwd += terrainCollider->cellSize)
	{
		for (float distRight = 0.0f; distRight < maxCheckDistRight; distRight += terrainCollider->cellSize)
		{
			ST_Vector3 pos = sphereTraceVector3AddAndScale(sphereTraceVector3AddAndScale(posStart, terrainCollider->rightPlane.normal, signX * distRight), terrainCollider->forwardPlane.normal, signZ * distFwd);
			float dirDist = sphereTraceVector3Dot(dir, sphereTraceVector3Subtract(pos, pSphereTraceData->rayTraceData.startPoint));
			ST_Vector3 centerPos = sphereTraceVector3AddAndScale(pSphereTraceData->rayTraceData.startPoint, dir, fabsf(dirDist));
			float distFromCenterLine = sphereTraceVector3Dot(sphereTraceVector3Subtract(centerPos, pos), sphereRight);
			float height = centerPos.y - pSphereTraceData->radius / cTheta;
			if (fabsf(distFromCenterLine) <= pSphereTraceData->radius + terrainCollider->cellSize)
			{
				int index = sphereTraceColliderUniformTerrainSampleFirstTriangleIndex(terrainCollider, sphereTraceVector3Construct(pos.x, terrainCollider->aabb.leftDownBackTransformedVertex.y, pos.z));
				if (index != -1)
				{
					if (terrainCollider->triangles[index].aabb.rightTopForwardsTransformedVertex.y >= height)
						sphereTraceIntListAddFirst(&indices, index);
					if (terrainCollider->triangles[index + 1].aabb.rightTopForwardsTransformedVertex.y >= height)
						sphereTraceIntListAddFirst(&indices, index + 1);
				}
			}

		}
	}
	return indices;
}

b32 sphereTraceColliderUniformTerrainSphereTraceByStartEndPoint(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 startPoint, ST_Vector3 endPoint, float radius, ST_SphereTraceData* const pSphereTraceData)
{
	float minDist = FLT_MAX;
	ST_SphereTraceData stdDummy;
	stdDummy.rayTraceData.startPoint = startPoint;
	stdDummy.sphereCenter = endPoint;
	stdDummy.radius = radius;
	ST_Vector3 dir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(endPoint, startPoint));
	ST_SphereTerrainTriangleContactInfo sttci;
	if (sphereTraceColliderUniformTerrainImposedSphereFindMaxPenetratingTriangle(pTerrainCollider, startPoint, radius, &sttci))
	{
		*pSphereTraceData = sttci.downSphereTraceData;
		return 1;
	}
	//if (sphereTraceVector3Equal(dir, gVector3Down))
	//{
	//	return sphereTraceColliderUniformTerrainSphereTraceDown(pTerrainCollider, stdDummy.rayTraceData.startPoint, stdDummy.radius, pSphereTraceData);
	//}
	ST_IntList il = sphereTraceColliderUniformTerrainSampleTrianglesIndicesForSphereTrace(pTerrainCollider, &stdDummy);
	if (il.count == 0)
		return 0;
	ST_IntListData* pild = il.pFirst;

	for (int i = 0; i < il.count; i++)
	{
		if (sphereTraceColliderTriangleSphereTrace(startPoint, dir, radius, &pTerrainCollider->triangles[pild->value], &stdDummy))
		{
			float sphereLength = sphereTraceVector3Distance(stdDummy.sphereCenter, startPoint);
			if (sphereLength < minDist)
			{
				minDist = sphereLength;
				*pSphereTraceData = stdDummy;
			}
		}
		pild = pild->pNext;
	}
	sphereTraceIntListFree(&il);
	if (minDist != FLT_MAX)
		return 1;
	else
		return 0;
}


b32 sphereTraceColliderUniformTerrainSphereTrace(const ST_UniformTerrainCollider* const pTerrainCollider, ST_Vector3 from, ST_Vector3 dir, float radius, ST_SphereTraceData* const pSphereTraceData)
{
	dir = sphereTraceVector3Normalize(dir);
	if (dir.x == 0.0f && dir.z == 0.0f)
	{
		return sphereTraceColliderUniformTerrainSphereTraceDown(pTerrainCollider, from, radius, pSphereTraceData);
	}
	ST_RayTraceData rtcPlane;
	b32 doesIntersectWithTerrainBoundingBox = 0;
	b32 xIncrementing = 0;
	b32 yIncrementing = 0;
	b32 zIncrementing = 0;
	float magX = sphereTraceVector3Dot(dir, pTerrainCollider->rightPlane.normal);
	float magY = sphereTraceVector3Dot(dir, pTerrainCollider->topPlane.normal);
	float magZ = sphereTraceVector3Dot(dir, pTerrainCollider->forwardPlane.normal);
	ST_Vector3 intersectingDirection;
	ST_Vector3 intersection = from;
	ST_Vector3 dp = sphereTraceVector3Subtract(from, pTerrainCollider->position);
	float dpx = intersection.x - pTerrainCollider->position.x;
	float dpz = intersection.z - pTerrainCollider->position.z;
	float xDist = (dpx * pTerrainCollider->right.x + dpz * pTerrainCollider->right.z);
	float zDist = (dpx * pTerrainCollider->forward.x + dpz * pTerrainCollider->forward.z);
	float height = intersection.y;
	if (xDist <= pTerrainCollider->xSize && zDist <= pTerrainCollider->zSize && height <= pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y && xDist >= 0.0f && zDist >= 0.0f && height >= pTerrainCollider->aabb.leftDownBackTransformedVertex.y)
	{
		doesIntersectWithTerrainBoundingBox = 1;
		if (magX > 0.0f)
			xIncrementing = 1;
		if (magY > 0.0f)
			yIncrementing = 1;
		if (magZ > 0.0f)
			zIncrementing = 1;
	}
	else
	{
		if (magX <= 0.0f)
		{
			if (sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->rightPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		else
		{
			xIncrementing = 1;
			if (sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->leftPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		if (magY <= 0.0f)
		{
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->topPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		else
		{
			yIncrementing = 1;
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->bottomPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		if (magZ < 0.0f)
		{
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->forwardPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
		else
		{
			zIncrementing = 1;
			if (!doesIntersectWithTerrainBoundingBox && sphereTraceColliderPlaneRayTrace(from, dir, &pTerrainCollider->backPlane, &rtcPlane))
			{
				doesIntersectWithTerrainBoundingBox = 1;
				intersection = rtcPlane.hitPoint;
			}
		}
	}
	if (doesIntersectWithTerrainBoundingBox)
	{
		dpx = intersection.x - pTerrainCollider->position.x;
		dpz = intersection.z - pTerrainCollider->position.z;
		xDist = (dpx * pTerrainCollider->right.x + dpz * pTerrainCollider->right.z);
		zDist = (dpx * pTerrainCollider->forward.x + dpz * pTerrainCollider->forward.z);
		height = intersection.y;
		float tMin = FLT_MAX;
		b32 xChanged = 0;
		b32 zChanged = 0;
		ST_AABB heightBox;
		int xInd = (int)(xDist / pTerrainCollider->cellSize);
		int zInd = (int)(zDist / pTerrainCollider->cellSize);
		while (xDist <= pTerrainCollider->xSize && zDist <= pTerrainCollider->zSize && height <= pTerrainCollider->aabb.rightTopForwardsTransformedVertex.y && xDist >= 0.0f && zDist >= 0.0f && height >= pTerrainCollider->aabb.leftDownBackTransformedVertex.y)
		{


			b32 incomingFromX = 1;
			if (xIncrementing)
			{
				if (xChanged)
					xInd++;
				else
					xInd = (int)(xDist / pTerrainCollider->cellSize) + 1;
				float tentativeX = (xInd * pTerrainCollider->cellSize - xDist) / magX;
				if (tentativeX < tMin && tentativeX > 0.0f)
				{
					tMin = tentativeX;
					intersectingDirection = pTerrainCollider->rightPlane.normal;
				}
			}
			else
			{
				if (xChanged)
					xInd--;
				else
					xInd = (int)(xDist / pTerrainCollider->cellSize);
				float tentativeX = (xInd * pTerrainCollider->cellSize - xDist) / magX;
				if (tentativeX < tMin && tentativeX > 0.0f)
				{

					tMin = tentativeX;
					intersectingDirection = pTerrainCollider->leftPlane.normal;
				}
			}

			if (zIncrementing)
			{
				if (zChanged)
					zInd++;
				else
					zInd = (int)(zDist / pTerrainCollider->cellSize) + 1;
				float tentativeT = (zInd * pTerrainCollider->cellSize - zDist) / magZ;
				if (tentativeT < tMin && tentativeT > 0.0f)
				{

					tMin = tentativeT;
					intersectingDirection = pTerrainCollider->forwardPlane.normal;
					incomingFromX = 0;
				}
			}
			else
			{
				if (zChanged)
					zInd--;
				else
					zInd = (int)(zDist / pTerrainCollider->cellSize);
				float tentativeT = (zInd * pTerrainCollider->cellSize - zDist) / magZ;
				if (tentativeT < tMin && tentativeT > 0.0f)
				{

					tMin = tentativeT;
					intersectingDirection = pTerrainCollider->backPlane.normal;
					incomingFromX = 0;
				}
			}
			ST_Vector3 nextIntersection = sphereTraceVector3AddAndScale(intersection, dir, tMin);
			//need to adjust the intersection so we dont get stuck with floating point error
			if (incomingFromX)
			{
				xChanged = 1;
				xDist = xInd * pTerrainCollider->cellSize;
				dpx = nextIntersection.x - pTerrainCollider->position.x;
				dpz = nextIntersection.z - pTerrainCollider->position.z;
				zDist = (dpx * pTerrainCollider->forward.x + dpz * pTerrainCollider->forward.z);
			}
			else
			{
				zChanged = 1;
				dpx = nextIntersection.x - pTerrainCollider->position.x;
				dpz = nextIntersection.z - pTerrainCollider->position.z;
				zDist = zInd * pTerrainCollider->cellSize;
				xDist = (dpx * pTerrainCollider->right.x + dpz * pTerrainCollider->right.z);
			}
			float nextHeight = nextIntersection.y;
			if (yIncrementing)
			{
				heightBox.rightTopForwardsTransformedVertex.y = nextHeight;
				heightBox.leftDownBackTransformedVertex.y = height;
			}
			else
			{
				heightBox.rightTopForwardsTransformedVertex.y = height;
				heightBox.leftDownBackTransformedVertex.y = nextHeight;
			}
			//if (sphereTraceColliderUniformTerrainSphereTraceByStartEndPoint(pTerrainCollider, intersection, nextIntersection, radius, pSphereTraceData))
			{
				float minDist = FLT_MAX;
				ST_SphereTraceData stdDummy;
				stdDummy.rayTraceData.startPoint = intersection;
				stdDummy.sphereCenter = nextIntersection;
				stdDummy.radius = radius;
				//ST_Vector3 dir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(endPoint, startPoint));
				ST_IntList il = sphereTraceColliderUniformTerrainSampleTrianglesIndicesForSphereTrace(pTerrainCollider, &stdDummy);
				//if (il.count == 0)
				//	return 0;
				ST_IntListData* pild = il.pFirst;

				for (int i = 0; i < il.count; i++)
				{
					//sceneDrawTriangleOutline(&pTerrainCollider->triangles[pild->value], gVector4ColorGreen);
					//sceneDrawAABB(&pTerrainCollider->triangles[pild->value].aabb, gVector4ColorRed);
					if (sphereTraceColliderTriangleSphereTrace(from, dir, radius, &pTerrainCollider->triangles[pild->value], &stdDummy))
					{
						float sphereLength = sphereTraceVector3Distance(stdDummy.sphereCenter, from);
						if (sphereLength < minDist)
						{
							minDist = sphereLength;
							*pSphereTraceData = stdDummy;
						}
					}
					pild = pild->pNext;
				}
				sphereTraceIntListFree(&il);
				if (minDist != FLT_MAX)
				{
					int triIndex;
					ST_TriangleCollider* ptc;
					stdDummy.rayTraceData.startPoint = pSphereTraceData->sphereCenter;
					stdDummy.sphereCenter = sphereTraceVector3AddAndScale(stdDummy.rayTraceData.startPoint, dir, radius);
					il = sphereTraceColliderUniformTerrainSampleTrianglesIndicesForSphereTrace(pTerrainCollider, &stdDummy);
					ST_IntListData* pild = il.pFirst;
					for (int i = 0; i < il.count; i++)
					{
						//sceneDrawTriangleOutline(&pTerrainCollider->triangles[pild->value], gVector4ColorGreen);
						//sceneDrawAABB(&pTerrainCollider->triangles[pild->value].aabb, gVector4ColorRed);
						if (sphereTraceColliderTriangleSphereTrace(from, dir, radius, &pTerrainCollider->triangles[pild->value], &stdDummy))
						{
							float sphereLength = sphereTraceVector3Distance(stdDummy.sphereCenter, from);
							if (sphereLength < minDist)
							{
								minDist = sphereLength;
								*pSphereTraceData = stdDummy;
								triIndex = pild->value;
								ptc = &pTerrainCollider->triangles[triIndex];
							}
						}
						pild = pild->pNext;
					}
					//sphereTraceColliderTriangleSphereTrace(from, dir, radius, ptc, &stdDummy);
					ST_Vector3 adjustedDir = sphereTraceVector3Normalize(sphereTraceVector3Subtract(pSphereTraceData->sphereCenter, pSphereTraceData->rayTraceData.startPoint));
					printf("%i\n", sphereTraceVector3Equal(dir, adjustedDir));
					sphereTraceIntListFree(&il);
					return 1;
				}
			}
			//ST_Vector3 samplePos = sphereTraceVector3Average(intersection, nextIntersection);
			//int triIndex = sphereTraceColliderUniformTerrainSampleFirstTriangleIndex(pTerrainCollider, samplePos);
			//if (triIndex != -1)
			//{
			//	ST_TriangleCollider* pt1 = &pTerrainCollider->triangles[triIndex];
			//	ST_TriangleCollider* pt2 = &pTerrainCollider->triangles[triIndex + 1];

			//	if (sphereTraceColliderAABBIntersectAABBVertically(&pt1->aabb, &heightBox))
			//	{
			//		if (sphereTraceColliderTriangleRayTrace(from, dir, pt1, pSphereTraceData))
			//		{
			//			return 1;
			//		}
			//	}
			//	if (sphereTraceColliderAABBIntersectAABBVertically(&pt2->aabb, &heightBox))
			//	{
			//		if (sphereTraceColliderTriangleRayTrace(from, dir, pt2, pSphereTraceData))
			//		{
			//			return 1;
			//		}
			//	}
			//}

			intersection = nextIntersection;
			height = intersection.y;
			tMin = FLT_MAX;


		}
	}
	return 0;
}
