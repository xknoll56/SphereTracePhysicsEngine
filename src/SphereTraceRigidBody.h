#pragma once
#include "SphereTraceLists.h"

typedef struct ST_RigidBody
{
    //constants
    float mass;
    float inertia;
    float massInv;
    float inertiaInv;

    //state variables
    ST_Vector3 position;
    ST_Vector3 prevPosition;
    ST_Quaternion rotation;
    ST_Matrix4 rotationMatrix;
    ST_Vector3 linearMomentum;
    ST_Vector3 angularMomentum;

    //derived quantities
    ST_Vector3 velocity;
    ST_Vector3 angularVelocity;

    //applied force/torque will be applie for a single step
    ST_Vector3List appliedForces;
    ST_Vector3List appliedDeltaMomentums;
    ST_Vector3List appliedTorques;
    ST_Vector3List appliedDeltaAngularMomentums;

    b32 isAsleep;
} ST_RigidBody;

ST_RigidBody sphereTraceRigidBodyConstruct(float mass, float inertia);

void sphereTraceRigidBodyAddForce(ST_RigidBody* const pRigidBody, const ST_Vector3 force);

void sphereTraceRigidBodyApplyForces(ST_RigidBody* const pRigidBody, float dt);

void sphereTraceRigidBodyAddDeltaMomentum(ST_RigidBody* const pRigidBody, const ST_Vector3 dp);

void sphereTraceRigidBodyApplyDeltaMomentums(ST_RigidBody* const pRigidBody);

void sphereTraceRigidBodyAddTorque(ST_RigidBody* const pRigidBody, const ST_Vector3 torque);

void sphereTraceRigidBodyApplyTorques(ST_RigidBody* const pRigidBody, float dt);

void sphereTraceRigidBodyAddDeltaAngularMomentum(ST_RigidBody* const pRigidBody, const ST_Vector3 dl);

void sphereTraceRigidBodyApplyDeltaAngularMomentums(ST_RigidBody* const pRigidBody);


void sphereTraceRigidBodySetVelocity(ST_RigidBody* const pRigidBody, const ST_Vector3 velocity);

void sphereTraceRigidBodySetAngularVelocity(ST_RigidBody* const pRigidBody, const ST_Vector3 angularVelocity);

float sphereTraceRigidBodyGetSpeed(ST_RigidBody* const pRigidBody);

