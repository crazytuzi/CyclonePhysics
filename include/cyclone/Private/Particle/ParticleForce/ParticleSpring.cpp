#include "Particle/ParticleForce/ParticleSpring.h"

using namespace cyclone;

ParticleSpring::ParticleSpring(Particle* other, const real springConstant, const real restLength): other(other),
    springConstant(springConstant), restLength(restLength)
{
}

void ParticleSpring::UpdateForce(Particle* particle, const real duration)
{
    if (particle != nullptr && other != nullptr)
    {
        // Calculate the vector of the spring
        Vector3 force;

        particle->GetPosition(&force);

        force -= other->GetPosition();

        // Calculate the magnitude of the force
        auto magnitude = force.Size();

        magnitude = (magnitude - restLength) * springConstant;

        // Calculate the final force and apply it
        force.Normalize();

        force *= -magnitude;

        particle->AddForce(force);
    }
}
