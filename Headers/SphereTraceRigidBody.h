#pragma once

typedef struct ST_RigidBody
{
    //constants
    float mass;
    float inertia;
    float massInv;
    float inertiaInv;
    float elasticity;

    //state variables
    ST_Vector3 position;
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
} ST_RigidBody;

ST_RigidBody sphereTraceRigidBodyConstruct(float mass, float inertia)
{
    ST_RigidBody rigidBody;
    rigidBody.mass = mass;
    rigidBody.inertia = inertia;
    rigidBody.massInv = 1.0f / mass;
    rigidBody.inertiaInv = 1.0f / inertia;
    rigidBody.elasticity = 0.25f;
    rigidBody.position = (ST_Vector3){ 0.0f,0.0f,0.0f };
    rigidBody.rotation = sphereTraceQuaternionFromEulerAngles((ST_Vector3) { 0.0f, 0.0f, 0.0f });
    rigidBody.rotationMatrix = sphereTraceMatrixFromQuaternion(rigidBody.rotation);
    rigidBody.linearMomentum = (ST_Vector3){ 0.0f,0.0f,0.0f };
    rigidBody.velocity = (ST_Vector3){ 0.0f,0.0f,0.0f };
    rigidBody.angularMomentum = (ST_Vector3){ 0.0f,0.0f,0.0f };
    rigidBody.angularVelocity = (ST_Vector3){ 0.0f,0.0f,0.0f };
    rigidBody.appliedForces = sphereTraceVector3ListConstruct();
    rigidBody.appliedDeltaMomentums = sphereTraceVector3ListConstruct();
    rigidBody.appliedTorques = sphereTraceVector3ListConstruct();
    rigidBody.appliedDeltaAngularMomentums = sphereTraceVector3ListConstruct();
    return rigidBody;
}

void sphereTraceRigidBodyAddForce(ST_RigidBody* const pRigidBody, const ST_Vector3 force)
{
	sphereTraceVector3ListAddFirst(&pRigidBody->appliedForces, force);
}

void sphereTraceRigidBodyApplyForces(ST_RigidBody* const pRigidBody, float dt)
{
    //apply forces
    int count = pRigidBody->appliedForces.count;
    for(int i = 0; i<count; i++)
    {
        pRigidBody->linearMomentum = sphereTraceVector3AddAndScale(pRigidBody->linearMomentum, pRigidBody->appliedForces.pFirst->value, dt);
        sphereTraceVector3ListRemoveFirst(&pRigidBody->appliedForces);
    }
}

void sphereTraceRigidBodyAddDeltaMomentum(ST_RigidBody* const pRigidBody, const ST_Vector3 dp)
{
	sphereTraceVector3ListAddFirst(&pRigidBody->appliedDeltaMomentums, dp);
}

void sphereTraceRigidBodyApplyDeltaMomentums(ST_RigidBody* const pRigidBody)
{
    //apply dps
    int count = pRigidBody->appliedDeltaMomentums.count;
    for (int i = 0; i < count; i++)
    {
        pRigidBody->linearMomentum = sphereTraceVector3Add(pRigidBody->linearMomentum, pRigidBody->appliedDeltaMomentums.pFirst->value);
        sphereTraceVector3ListRemoveFirst(&pRigidBody->appliedDeltaMomentums);
    }
}

void sphereTraceRigidBodyAddTorque(ST_RigidBody* const pRigidBody, const ST_Vector3 torque)
{
	sphereTraceVector3ListAddFirst(&pRigidBody->appliedTorques, torque);
}
void sphereTraceRigidBodyApplyTorques(ST_RigidBody* const pRigidBody, float dt)
{
    int count = pRigidBody->appliedTorques.count;
    for (int i = 0; i < count; i++)
    {
        pRigidBody->angularMomentum = sphereTraceVector3Add(pRigidBody->angularMomentum, sphereTraceVector3Scale(pRigidBody->appliedTorques.pFirst->value, dt));
        sphereTraceVector3ListRemoveFirst(&pRigidBody->appliedTorques);
    }

}

void sphereTraceRigidBodyAddDeltaAngularMomentum(ST_RigidBody* const pRigidBody, const ST_Vector3 dl)
{
	sphereTraceVector3ListAddFirst(&pRigidBody->appliedDeltaAngularMomentums, dl);
}

void sphereTraceRigidBodyApplyDeltaAngularMomentums(ST_RigidBody* const pRigidBody)
{
    int count = pRigidBody->appliedDeltaAngularMomentums.count;
    for (int i = 0; i < count; i++)
    {
        pRigidBody->angularMomentum = sphereTraceVector3Add(pRigidBody->angularMomentum, pRigidBody->appliedDeltaAngularMomentums.pFirst->value);
        sphereTraceVector3ListRemoveFirst(&pRigidBody->appliedDeltaAngularMomentums);
    }
}


void sphereTraceRigidBodySetVelocity(ST_RigidBody* const pRigidBody, const ST_Vector3 velocity)
{
    pRigidBody->linearMomentum = sphereTraceVector3Scale(velocity, pRigidBody->mass);
    pRigidBody->velocity = velocity;
}

void sphereTraceRigidBodySetAngularVelocity(ST_RigidBody* const pRigidBody, const ST_Vector3 angularVelocity)
{
    pRigidBody->angularMomentum = sphereTraceVector3Scale(angularVelocity, pRigidBody->inertia);
    pRigidBody->angularVelocity = angularVelocity;
}

