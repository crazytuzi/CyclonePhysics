#include "Particle.h"
#include <cassert>
#include <cfloat>
#include <cmath>

using namespace cyclone;

Particle::Particle(): inverseMass(0.f), damping(1.f)
{
}

void Particle::Integrate(const real deltaTime)
{
    // We don't integrate things with zero mass.
    if (inverseMass <= 0.f)
    {
        return;
    }

    assert(deltaTime > 0.f);

    // Update linear position.
    position += velocity * deltaTime;

    // Work out the acceleration from the force
    auto resultingAcceleration = acceleration;

    resultingAcceleration += accumulatedForce * inverseMass;

    // Update linear velocity from the acceleration.
    velocity += resultingAcceleration * deltaTime;

    // Impose drag.
    velocity *= real_pow(damping, deltaTime);

    // Clear the forces.
    ClearAccumulator();
}

void Particle::SetMass(const real mass)
{
    assert(mass != 0.f);

    inverseMass = 1.f / mass;
}

real Particle::GetMass() const
{
    if (inverseMass == 0.f)
    {
        return REAL_MAX;
    }

    return 1.f / inverseMass;
}

void Particle::SetInverseMass(const real inverseMass)
{
    Particle::inverseMass = inverseMass;
}

real Particle::GetInverseMass() const
{
    return inverseMass;
}

bool Particle::HasFiniteMass() const
{
    return inverseMass >= 0.f;
}

void Particle::SetDamping(const real damping)
{
    Particle::damping = damping;
}

real Particle::GetDamping() const
{
    return damping;
}

void Particle::SetPosition(const Vector3& position)
{
    Particle::position = position;
}

void Particle::SetPosition(const real x, const real y, const real z)
{
    position.x = x;

    position.y = y;

    position.z = z;
}

void Particle::GetPosition(Vector3* position) const
{
    assert(position != nullptr);

    *position = Particle::position;
}

Vector3 Particle::GetPosition() const
{
    return position;
}

void Particle::SetVelocity(const Vector3& velocity)
{
    Particle::velocity = velocity;
}

void Particle::SetVelocity(const real x, const real y, const real z)
{
    velocity.x = x;

    velocity.y = y;

    velocity.z = z;
}

void Particle::GetVelocity(Vector3* velocity) const
{
    assert(velocity != nullptr);

    *velocity = Particle::velocity;
}

Vector3 Particle::GetVelocity() const
{
    return velocity;
}

void Particle::SetAcceleration(const Vector3& acceleration)
{
    Particle::acceleration = acceleration;
}

void Particle::SetAcceleration(const real x, const real y, const real z)
{
    acceleration.x = x;

    acceleration.y = y;

    acceleration.z = z;
}

void Particle::GetAcceleration(Vector3* acceleration) const
{
    assert(acceleration != nullptr);

    *acceleration = Particle::acceleration;
}

Vector3 Particle::GetAcceleration() const
{
    return acceleration;
}

void Particle::ClearAccumulator()
{
    accumulatedForce.Reset();
}

void Particle::AddForce(const Vector3& force)
{
    accumulatedForce += force;
}
