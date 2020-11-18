#include "ParticleForce/ParticleSpring.h"
#include <cmath>

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

        magnitude = real_abs(magnitude - restLength);

        magnitude *= springConstant;

        // Calculate the final force and apply it
        force.Normalize();

        force *= -magnitude;

        particle->AddForce(force);
    }
}
