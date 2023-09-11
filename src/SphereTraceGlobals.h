#pragma once
#include "SphereTraceMath.h"
#include "SphereTraceLists.h"

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
extern const ST_Vector4 gVector4ColorWhite;
extern const ST_Vector4 gVector4ColorBlack;
extern const ST_Vector4 gVector4ColorYellow;
extern const ST_Vector4 gVector4ColorMagenta;
extern const ST_Vector4 gVector4ColorCyan;
extern const ST_Vector4 gVector4ColorRed;
extern const ST_Vector4 gVector4ColorGreen;
extern const ST_Vector4 gVector4ColorBlue;
extern const ST_Quaternion gQuaternionIdentity;
extern const ST_Matrix4 gMatrix4Identity;
extern const ST_Direction gDirectionRight;
extern const ST_Direction gDirectionLeft;
extern const ST_Direction gDirectionUp;
extern const ST_Direction gDirectionDown;
extern const ST_Direction gDirectionForward;
extern const ST_Direction gDirectionBack;

#define ST_VECTOR3(x, y, z) sphereTraceVector3Construct(x,y,z)
//#define ST_VECTOR3() gVector3Zero
#define ST_VECTOR4(x, y, z, w) sphereTraceVector4Construct(x, y, z, w);

#define ST_QUATERNION(v, alpha) sphereTraceQuaternionFromAngleAxis(v, alpha);

#define ST_FOR_EACH_START(pld, l) \
	pld = l.pFirst; \
	for(int i = 0; i<l.count; i++) { 

#define ST_FOR_EACH_END(pld) \
	pld = pld->pNext;} \

#define ST_VECTOR3LIST() sphereTraceVector3ListConstruct();
#define ST_INTLIST() sphereTraceIntListConstruct();