#include "RigidBody/RigidBody.h"
#include <assert.h>
#include <cfloat>
#include <cmath>

using namespace cyclone;

real RigidBody::sleepEpsilon = 0.3f;

RigidBody::RigidBody(): inverseMass(0), linearDamping(0), angularDamping(0), motion(0), isAwake(false), canSleep(false)
{
}

void RigidBody::CalculateDerivedData()
{
    orientation.Normalize();

    // Calculate the transform matrix for the body.
    CalculateTransformMatrix(transformMatrix, position, orientation);

    // Calculate the inertiaTensor in world space.
    TransformInertiaTensor(inverseInertiaTensorWorld, inverseInertiaTensor, transformMatrix);
}

void RigidBody::Integrate(const real deltaTime)
{
    if (!isAwake)
    {
        return;
    }

    // Calculate linear acceleration from force inputs.
    lastFrameAcceleration = acceleration;

    lastFrameAcceleration += accumulatedForce * inverseMass;

    // Calculate angular acceleration from torque inputs.
    const auto angularAcceleration = inverseInertiaTensorWorld.TransformVector(accumulatedTorque);

    // Adjust velocities
    // Update linear velocity from both acceleration and impulse.
    velocity += lastFrameAcceleration * deltaTime;

    // Update angular velocity from both acceleration and impulse.
    rotation += angularAcceleration * deltaTime;

    // Impose drag.
    velocity *= real_pow(linearDamping, deltaTime);

    rotation *= real_pow(angularDamping, deltaTime);

    // Adjust positions
    // Update linear position.
    position += velocity * deltaTime;

    // Update angular position.
    orientation += rotation * deltaTime;

    // Normalize the orientation, and update the matrices with the new
    // position and orientation
    CalculateDerivedData();

    // Clear accumulators.
    ClearAccumulators();

    // Update the kinetic energy store, and possibly put the body to sleep.
    if (canSleep)
    {
        const auto currentMotion = (velocity | velocity) + (rotation | rotation);

        const auto bias = real_pow(0.5f, deltaTime);

        motion = bias * motion + (1 - bias) * currentMotion;

        if (motion < sleepEpsilon)
        {
            SetAwake(false);
        }
        else if (motion > 10 * sleepEpsilon)
        {
            motion = 10 * sleepEpsilon;
        }
    }
}

void RigidBody::SetMass(const real mass)
{
    assert(mass != 0);

    inverseMass = 1.f / mass;
}

real RigidBody::GetMass() const
{
    if (inverseMass == 0)
    {
        return REAL_MAX;
    }

    return 1.f / inverseMass;
}

void RigidBody::SetInverseMass(const real inverseMass)
{
    RigidBody::inverseMass = inverseMass;
}

real RigidBody::GetInverseMass() const
{
    return inverseMass;
}

bool RigidBody::HasFiniteMass() const
{
    return inverseMass >= 0.f;
}

void RigidBody::SetInertiaTensor(const Matrix& inertiaTensor)
{
    inverseInertiaTensor = inertiaTensor.Inverse();
}

void RigidBody::GetInertiaTensor(Matrix* inertiaTensor) const
{
    if (inertiaTensor != nullptr)
    {
        *inertiaTensor = inverseInertiaTensor.Inverse();
    }
}

Matrix RigidBody::GetInertiaTensor() const
{
    return inverseInertiaTensor.Inverse();
}

void RigidBody::GetInertiaTensorWorld(Matrix* inertiaTensor) const
{
    if (inertiaTensor != nullptr)
    {
        *inertiaTensor = inverseInertiaTensorWorld.Inverse();
    }
}

Matrix RigidBody::GetInertiaTensorWorld() const
{
    return inverseInertiaTensorWorld.Inverse();
}

void RigidBody::SetInverseInertiaTensor(const Matrix& inverseInertiaTensor)
{
    RigidBody::inverseInertiaTensor = inverseInertiaTensor;
}

void RigidBody::GetInverseInertiaTensor(Matrix* inverseInertiaTensor) const
{
    if (inverseInertiaTensor != nullptr)
    {
        *inverseInertiaTensor = RigidBody::inverseInertiaTensor;
    }
}

Matrix RigidBody::GetInverseInertiaTensor() const
{
    return inverseInertiaTensor;
}

void RigidBody::GetInverseInertiaTensorWorld(Matrix* inverseInertiaTensor) const
{
    if (inverseInertiaTensor != nullptr)
    {
        *inverseInertiaTensor = inverseInertiaTensorWorld;
    }
}

Matrix RigidBody::GetInverseInertiaTensorWorld() const
{
    return inverseInertiaTensorWorld;
}

void RigidBody::SetDamping(const real linearDamping, const real angularDamping)
{
    RigidBody::linearDamping = linearDamping;

    RigidBody::angularDamping = angularDamping;
}

void RigidBody::SetLinearDamping(const real linearDamping)
{
    RigidBody::linearDamping = linearDamping;
}

real RigidBody::GetLinearDamping() const
{
    return linearDamping;
}

void RigidBody::SetAngularDamping(const real angularDamping)
{
    RigidBody::angularDamping = angularDamping;
}

real RigidBody::GetAngularDamping() const
{
    return angularDamping;
}

void RigidBody::SetPosition(const Vector3& position)
{
    RigidBody::position = position;
}

void RigidBody::SetPosition(const real x, const real y, const real z)
{
    position.x = x;

    position.y = y;

    position.z = z;
}

void RigidBody::GetPosition(Vector3* position) const
{
    if (position != nullptr)
    {
        *position = RigidBody::position;
    }
}

Vector3 RigidBody::GetPosition() const
{
    return position;
}

void RigidBody::SetOrientation(const Quaternion& orientation)
{
    RigidBody::orientation = orientation;
}

void RigidBody::SetOrientation(const real i, const real j, const real k, const real a)
{
    orientation.i = i;

    orientation.j = j;

    orientation.k = k;

    orientation.a = a;

    orientation.Normalize();
}

void RigidBody::GetOrientation(Quaternion* orientation) const
{
    if (orientation != nullptr)
    {
        *orientation = RigidBody::orientation;
    }
}

Quaternion RigidBody::GetOrientation() const
{
    return orientation;
}

void RigidBody::GetOrientation(Matrix* matrix) const
{
    if (matrix != nullptr)
    {
        *matrix = transformMatrix;
    }
}

void RigidBody::GetTransform(Matrix* transform) const
{
    if (transform != nullptr)
    {
        *transform = transformMatrix;
    }
}

void RigidBody::GetGLTransform(float matrix[16]) const
{
    matrix[0] = transformMatrix.M[0][0];

    matrix[1] = transformMatrix.M[1][0];

    matrix[2] = transformMatrix.M[2][0];

    matrix[3] = 0;

    matrix[4] = transformMatrix.M[0][1];

    matrix[5] = transformMatrix.M[1][1];

    matrix[6] = transformMatrix.M[1][2];

    matrix[7] = 0;

    matrix[8] = transformMatrix.M[0][2];

    matrix[9] = transformMatrix.M[1][2];

    matrix[10] = transformMatrix.M[2][2];

    matrix[11] = 0;

    matrix[12] = transformMatrix.M[0][3];

    matrix[13] = transformMatrix.M[1][3];

    matrix[14] = transformMatrix.M[2][3];

    matrix[15] = 1;
}

Matrix RigidBody::GetTransform() const
{
    return transformMatrix;
}

Vector3 RigidBody::GetPointInLocalSpace(const Vector3& point) const
{
    return transformMatrix.InverseTransformPosition(point);
}

Vector3 RigidBody::GetPointInWorldSpace(const Vector3& point) const
{
    return transformMatrix.TransformPosition(point);
}

Vector3 RigidBody::GetDirectionInLocalSpace(const Vector3& direction) const
{
    return transformMatrix.InverseTransformVector(direction);
}

Vector3 RigidBody::GetDirectionInWorldSpace(const Vector3& direction) const
{
    return transformMatrix.TransformVector(direction);
}

void RigidBody::SetVelocity(const Vector3& velocity)
{
    RigidBody::velocity = velocity;
}

void RigidBody::SetVelocity(const real x, const real y, const real z)
{
    velocity.x = x;

    velocity.y = y;

    velocity.z = z;
}

void RigidBody::GetVelocity(Vector3* velocity) const
{
    if (velocity != nullptr)
    {
        *velocity = RigidBody::velocity;
    }
}

Vector3 RigidBody::GetVelocity() const
{
    return velocity;
}

void RigidBody::AddVelocity(const Vector3& deltaVelocity)
{
    velocity += deltaVelocity;
}

void RigidBody::SetRotation(const Vector3& rotation)
{
    RigidBody::rotation = rotation;
}

void RigidBody::SetRotation(const real x, const real y, const real z)
{
    rotation.x = x;

    rotation.y = y;

    rotation.z = z;
}

void RigidBody::GetRotation(Vector3* rotation) const
{
    if (rotation != nullptr)
    {
        *rotation = RigidBody::rotation;
    }
}

Vector3 RigidBody::GetRotation() const
{
    return rotation;
}

void RigidBody::AddRotation(const Vector3& deltaRotation)
{
    rotation += deltaRotation;
}

bool RigidBody::GetAwake() const
{
    return isAwake;
}

void RigidBody::SetAwake(const bool awake)
{
    if (awake)
    {
        isAwake = true;

        // Add a bit of motion to avoid it falling asleep immediately.
        motion = sleepEpsilon * 2.f;
    }
    else
    {
        isAwake = false;

        velocity.Reset();

        rotation.Reset();
    }
}

bool RigidBody::GetCanSleep() const
{
    return canSleep;
}

void RigidBody::SetCanSleep(const bool canSleep)
{
    RigidBody::canSleep = canSleep;

    if (!canSleep && !isAwake)
    {
        SetAwake();
    }
}

void RigidBody::GetLastFrameLinearAcceleration(Vector3* linearAcceleration) const
{
    if (linearAcceleration != nullptr)
    {
        *linearAcceleration = lastFrameAcceleration;
    }
}

Vector3 RigidBody::GetLastFrameLinearAcceleration() const
{
    return lastFrameAcceleration;
}

void RigidBody::ClearAccumulators()
{
    accumulatedForce.Reset();

    accumulatedTorque.Reset();
}

void RigidBody::AddForce(const Vector3& force)
{
    accumulatedForce += force;

    isAwake = true;
}

void RigidBody::AddForceAtPoint(const Vector3& force, const Vector3& point)
{
    // Convert to coordinates relative to center of mass.
    auto pt = point;

    pt -= position;

    accumulatedForce += force;

    accumulatedTorque += pt ^ force;

    isAwake = true;
}

void RigidBody::AddForceAtBodyPoint(const Vector3& force, const Vector3& point)
{
    // Convert to coordinates relative to center of mass.
    const auto pointInWorld = GetPointInWorldSpace(point);

    AddForceAtPoint(force, pointInWorld);
}

void RigidBody::AddTorque(const Vector3& torque)
{
    accumulatedTorque += torque;

    isAwake = true;
}

void RigidBody::SetAcceleration(const Vector3& acceleration)
{
    RigidBody::acceleration = acceleration;
}

void RigidBody::SetAcceleration(const real x, const real y, const real z)
{
    acceleration.x = x;

    acceleration.y = y;

    acceleration.z = z;
}

void RigidBody::GetAcceleration(Vector3* acceleration) const
{
    if (acceleration != nullptr)
    {
        *acceleration = RigidBody::acceleration;
    }
}

Vector3 RigidBody::GetAcceleration() const
{
    return acceleration;
}

void RigidBody::CalculateTransformMatrix(Matrix& transformMatrix, const Vector3& position,
                                         const Quaternion& orientation)
{
    transformMatrix.M[0][0] = 1 - 2 * orientation.j * orientation.j - 2 * orientation.k * orientation.k;

    transformMatrix.M[0][1] = 2 * orientation.i * orientation.j - 2 * orientation.a * orientation.k;

    transformMatrix.M[0][2] = 2 * orientation.i * orientation.k + 2 * orientation.a * orientation.j;

    transformMatrix.M[0][3] = position.x;

    transformMatrix.M[1][0] = 2 * orientation.i * orientation.j + 2 * orientation.a * orientation.k;

    transformMatrix.M[1][1] = 1 - 2 * orientation.i * orientation.i - 2 * orientation.k * orientation.k;

    transformMatrix.M[1][2] = 2 * orientation.j * orientation.k - 2 * orientation.a * orientation.i;

    transformMatrix.M[1][3] = position.y;

    transformMatrix.M[2][0] = 2 * orientation.i * orientation.k - 2 * orientation.a * orientation.j;

    transformMatrix.M[2][1] = 2 * orientation.j * orientation.k + 2 * orientation.a * orientation.i;

    transformMatrix.M[2][2] = 1 - 2 * orientation.i * orientation.i - 2 * orientation.j * orientation.j;

    transformMatrix.M[2][3] = position.z;

    transformMatrix.M[3][0] = transformMatrix.M[3][1] = transformMatrix.M[3][2] = 0;

    transformMatrix.M[3][3] = 1.f;
}

void RigidBody::TransformInertiaTensor(Matrix& inverseInertiaTensorWorld, const Matrix& inverseInertiaTensorLocal,
                                       const Matrix& transformMatrix)
{
    const auto t4 = transformMatrix.M[0][0] * inverseInertiaTensorLocal.M[0][0] + transformMatrix.M[0][1] *
        inverseInertiaTensorLocal.M[1][0]
        + transformMatrix.M[0][2] * inverseInertiaTensorLocal.M[2][0];

    const auto t9 = transformMatrix.M[0][0] * inverseInertiaTensorLocal.M[0][1] + transformMatrix.M[0][1] *
        inverseInertiaTensorLocal.M[1][1]
        + transformMatrix.M[0][2] * inverseInertiaTensorLocal.M[2][1];

    const auto t14 = transformMatrix.M[0][0] * inverseInertiaTensorLocal.M[0][2] + transformMatrix.M[0][1] *
        inverseInertiaTensorLocal.M[1][2]
        + transformMatrix.M[0][2] * inverseInertiaTensorLocal.M[2][2];

    const auto t28 = transformMatrix.M[1][0] * inverseInertiaTensorLocal.M[0][0] + transformMatrix.M[1][1] *
        inverseInertiaTensorLocal.M[1][0]
        + transformMatrix.M[1][2] * inverseInertiaTensorLocal.M[2][0];

    const auto t33 = transformMatrix.M[1][0] * inverseInertiaTensorLocal.M[0][1] + transformMatrix.M[1][1] *
        inverseInertiaTensorLocal.M[1][1]
        + transformMatrix.M[1][2] * inverseInertiaTensorLocal.M[2][1];

    const auto t38 = transformMatrix.M[1][0] * inverseInertiaTensorLocal.M[0][2] + transformMatrix.M[1][1] *
        inverseInertiaTensorLocal.M[1][2]
        + transformMatrix.M[1][2] * inverseInertiaTensorLocal.M[2][2];

    const auto t52 = transformMatrix.M[2][0] * inverseInertiaTensorLocal.M[0][0] + transformMatrix.M[2][1] *
        inverseInertiaTensorLocal.M[1][0]
        + transformMatrix.M[2][2] * inverseInertiaTensorLocal.M[2][0];

    const auto t57 = transformMatrix.M[2][0] * inverseInertiaTensorLocal.M[0][1] + transformMatrix.M[2][1] *
        inverseInertiaTensorLocal.M[1][1]
        + transformMatrix.M[2][2] * inverseInertiaTensorLocal.M[2][1];

    const auto t62 = transformMatrix.M[2][0] * inverseInertiaTensorLocal.M[0][2] + transformMatrix.M[2][1] *
        inverseInertiaTensorLocal.M[1][2]
        + transformMatrix.M[2][2] * inverseInertiaTensorLocal.M[2][2];

    inverseInertiaTensorWorld.M[0][0] = t4 * transformMatrix.M[0][0] + t9 * transformMatrix.M[0][1] + t14 *
        transformMatrix.M[0][2];

    inverseInertiaTensorWorld.M[0][1] = t4 * transformMatrix.M[1][0] + t9 * transformMatrix.M[1][1] + t14 *
        transformMatrix.M[1][2];

    inverseInertiaTensorWorld.M[0][2] = t4 * transformMatrix.M[2][0] + t9 * transformMatrix.M[2][1] + t14 *
        transformMatrix.M[2][2];

    inverseInertiaTensorWorld.M[1][0] = t28 * transformMatrix.M[0][0] + t33 * transformMatrix.M[0][1] + t38 *
        transformMatrix.M[0][2];

    inverseInertiaTensorWorld.M[1][1] = t28 * transformMatrix.M[1][0] + t33 * transformMatrix.M[1][1] + t38 *
        transformMatrix.M[1][2];

    inverseInertiaTensorWorld.M[1][2] = t28 * transformMatrix.M[2][0] + t33 * transformMatrix.M[2][1] + t38 *
        transformMatrix.M[2][2];

    inverseInertiaTensorWorld.M[2][0] = t52 * transformMatrix.M[0][0] + t57 * transformMatrix.M[0][1] + t62 *
        transformMatrix.M[0][2];

    inverseInertiaTensorWorld.M[2][1] = t52 * transformMatrix.M[1][0] + t57 * transformMatrix.M[1][1] + t62 *
        transformMatrix.M[1][2];

    inverseInertiaTensorWorld.M[2][2] = t52 * transformMatrix.M[2][0] + t57 * transformMatrix.M[2][1] + t62 *
        transformMatrix.M[2][2];
}
