#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <stdint.h>

typedef int b32;

typedef struct ST_Vector2
{
	float x;
	float y;
} ST_Vector2;

typedef struct ST_Vector2Integer
{
	int x;
	int y;
} ST_Vector2Integer;

typedef struct ST_Vector3
{
	float x;
	float y;
	float z;
} ST_Vector3;

typedef struct ST_Vector4
{
	float x;
	float y;
	float z;
	float w;
} ST_Vector4;


typedef struct ST_Matrix4
{
	float m00, m01, m02, m03;
	float m10, m11, m12, m13;
	float m20, m21, m22, m23;
	float m30, m31, m32, m33;
} ST_Matrix4;

typedef struct ST_Quaternion
{
	float w;
	float x;
	float y;
	float z;
} ST_Quaternion;

typedef struct ST_Direction
{
	ST_Vector3 v;
	b32 normalized;
}ST_Direction;



ST_Vector2 sphereTraceVector2Construct(float x, float y);

ST_Vector2Integer sphereTraceVector2IntegerConstruct(int x, int y);

ST_Vector3 sphereTraceVector3Construct(float x, float y, float z);

ST_Vector4 sphereTraceVector4Construct(float x, float y, float z, float w);

ST_Matrix4 sphereTraceMatrixConstruct(float m00, float m01, float m02, float m03,
	float m10, float m11, float m12, float m13,
	float m20, float m21, float m22, float m23,
	float m30, float m31, float m32, float m33);


ST_Vector4 sphereTraceMatrixRow(ST_Matrix4 const matrix, int i);

ST_Vector4 sphereTraceMatrixCol(ST_Matrix4 const matrix, int i);

ST_Vector2 sphereTraceVector2Add(ST_Vector2 v1, ST_Vector2 v2);

ST_Vector2 sphereTraceVector2Subtract(ST_Vector2 v1, ST_Vector2 v2);

ST_Vector2Integer sphereTraceVector2IntegerAdd(ST_Vector2Integer v1, ST_Vector2Integer v2);

ST_Vector2Integer sphereTraceVector2IntegerSubtract(ST_Vector2Integer v1, ST_Vector2Integer v2);


float sphereTraceDegreesToRadians(float degs);

b32 sphereTraceVector3Equal(ST_Vector3 v1, ST_Vector3 v2);

float sphereTraceVector3Length(ST_Vector3 v);

float sphereTraceVector3Dot(ST_Vector3 v1, ST_Vector3 v2);

float sphereTraceVector4Dot(ST_Vector4 v1, ST_Vector4 v2);

ST_Vector3 sphereTraceVector3Scale(ST_Vector3 v, float f);

void sphereTraceVector3ScaleByRef(ST_Vector3* const pRef, float f);

ST_Vector3 sphereTraceVector3Normalize(ST_Vector3 v);

void sphereTraceVector3NormalizeByRef(ST_Vector3* const pRef);

ST_Vector3 sphereTraceVector3Cross(ST_Vector3 v1, ST_Vector3 v2);

ST_Vector3 sphereTraceVector3Add(ST_Vector3 v1, ST_Vector3 v2);

void sphereTraceVector3AddByRef(ST_Vector3* const pRef, ST_Vector3 v);

ST_Vector3 sphereTraceVector3Subtract(ST_Vector3 v1, ST_Vector3 v2);

void sphereTraceVector3SubtractByRef(ST_Vector3* const pRef, ST_Vector3 v);

ST_Vector3 sphereTraceVector3Negative(ST_Vector3 v);

void sphereTraceVector3NegativeByRef(ST_Vector3* pRef);

void sphereTraceVector3Print(ST_Vector3 v);

b32 sphereTraceVector3EpsilonEquals(ST_Vector3 v1, ST_Vector3 v2, float epsilon);

b32 sphereTraceVector4EpsilonEquals(ST_Vector4 v1, ST_Vector4 v2, float epsilon);

ST_Vector3 sphereTraceVector3CopySign(ST_Vector3 v, float f);

float sphereTraceVector3Length2(ST_Vector3 vec);

ST_Vector3 sphereTraceVector3AddAndScale(ST_Vector3 toAdd, ST_Vector3 toScale, float scale);

void sphereTraceVector3AddAndScaleByRef(ST_Vector3* const pRef, ST_Vector3 toScale, float scale);

ST_Vector3 sphereTraceVector3Average(ST_Vector3 v1, ST_Vector3 v2);

b32 sphereTraceVector3Nan(ST_Vector3 vec);

b32 sphereTraceVector3NanAny(ST_Vector3 vec);

ST_Vector3 sphereTraceClosestPointOnLineBetweenTwoLines(ST_Vector3 point, ST_Vector3 lineDir, ST_Vector3 otherPoint, ST_Vector3 otherLineDir);

b32 sphereTraceVector3ClosestPointOnLineBetweenTwoLinesIsGreaterThanZeroAndLessThanMaxDist(ST_Vector3 point, ST_Vector3 normalizedLineDir, ST_Vector3 otherPoint, ST_Vector3 otherNormalizedLineDir, float maxDist);

float sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistanceOnLine(ST_Vector3 point, ST_Vector3 normalizedLineDir, ST_Vector3 otherPoint, ST_Vector3 otherNormalizedLineDir);

void sphereTraceVector3ClosestPointOnLineBetweenTwoLinesDistancesOnLines(ST_Vector3 point, ST_Vector3 normalizedLineDir, ST_Vector3 otherPoint, ST_Vector3 otherNormalizedLineDir, float* dist1, float* dist2);

void sphereTraceVector3ClosestPointsOnLineBetweenTwoLines(ST_Vector3 point, ST_Vector3 lineDir, ST_Vector3 otherPoint, ST_Vector3 otherLineDir, ST_Vector3* result1, ST_Vector3* result2);

float sphereTraceVector3Distance(ST_Vector3 point1, ST_Vector3 point2);

ST_Vector3 sphereTraceVector3Lerp(ST_Vector3 point1, ST_Vector3 point2, float t);

ST_Vector3 sphereTraceNormalizeBetweenPoints(ST_Vector3 to, ST_Vector3 from);

ST_Matrix4 sphereTraceMatrixIdentity();

ST_Matrix4 sphereTraceMatrixRotateX(float rad);

ST_Matrix4 sphereTraceMatrixRotateY(float rad);

ST_Matrix4 sphereTraceMatrixRotateZ(float rad);

ST_Matrix4 sphereTraceMatrixTranslation(ST_Vector3 trans);

ST_Matrix4 sphereTraceMatrixScale(ST_Vector3 scale);

ST_Matrix4 sphereTraceMatrixRotate(ST_Vector3 eulerAngles);

ST_Matrix4 sphereTraceMatrixPerspective(float aspectRatio, float fovYRadians, float zNear, float zFar);

ST_Matrix4 sphereTraceMatrixLookAt(ST_Vector3 eye, ST_Vector3 at, ST_Vector3 up);

ST_Matrix4 sphereTraceMatrixMult(ST_Matrix4 mat1, ST_Matrix4 mat2);

ST_Vector3 sphereTraceVector3GetLocalXAxisFromRotationMatrix(ST_Matrix4 mat);

ST_Vector3 sphereTraceVector3GetLocalYAxisFromRotationMatrix(ST_Matrix4 mat);

ST_Vector3 sphereTraceVector3GetLocalZAxisFromRotationMatrix(ST_Matrix4 mat);

void sphereTraceMatrixSetLocalXAxisOfRotationMatrix(ST_Matrix4* const mat, ST_Vector3 xAxis);

void sphereTraceMatrixSetLocalYAxisOfRotationMatrix(ST_Matrix4* const mat, ST_Vector3 yAxis);

void sphereTraceMatrixSetLocalZAxisOfRotationMatrix(ST_Matrix4* const mat, ST_Vector3 zAxis);

ST_Quaternion sphereTraceQuaternionConstruct(float w, float x, float y, float z);


ST_Quaternion sphereTraceQuaternionConjugate(ST_Quaternion quat);

void sphereTraceQuaternionConjugateByRef(ST_Quaternion* const pRef);

ST_Matrix4 sphereTraceMatrixFromQuaternion(ST_Quaternion quat);

ST_Quaternion sphereTraceQuaternionFromAngleAxis(ST_Vector3 axis, float angle);

ST_Quaternion sphereTraceQuaternionFromEulerAngles(ST_Vector3 eulerAngles);

ST_Quaternion sphereTraceQuaternionNormalize(ST_Quaternion quat);

void sphereTraceQuaternionNormalizeByRef(ST_Quaternion* const pRef);

ST_Quaternion sphereTraceQuaternionMultiply(ST_Quaternion a, ST_Quaternion b);

ST_Quaternion sphereTraceQuaternionAdd(ST_Quaternion a, ST_Quaternion b);

ST_Quaternion sphereTraceQuaternionSubtract(ST_Quaternion a, ST_Quaternion b);

ST_Quaternion sphereTraceQuaternionScale(float f, ST_Quaternion a);

void sphereTraceQuaternionPrint(ST_Quaternion quat);

ST_Vector3 sphereTraceVector3RotatePoint(ST_Vector3 point, ST_Quaternion rotation);

ST_Matrix4 sphereTraceMatrixConstructFromRightForwardUp(ST_Vector3 right, ST_Vector3 up, ST_Vector3 forward);

ST_Quaternion sphereTraceMatrixQuaternionFromRotationMatrix(ST_Matrix4 mat);

ST_Direction sphereTraceDirectionConstruct(ST_Vector3 vec, b32 normalized);

ST_Direction sphereTraceDirectionConstructNormalized(ST_Vector3 vec);

void sphereTraceDirectionNormalizeIfNotNormalizedByRef(ST_Direction* const dir);

ST_Direction sphereTraceDirectionNormalizeIfNotNormalized(ST_Direction dir);

ST_Direction sphereTraceDirectionNegative(ST_Direction dir);