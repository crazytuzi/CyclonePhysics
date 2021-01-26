#include "Particle/ParticleForce/ParticleBungee.h"

using namespace cyclone;

ParticleBungee::ParticleBungee(Particle* other, const real springConstant, const real restLength): other(other),
    springConstant(springConstant), restLength(restLength)
{
}

void ParticleBungee::UpdateForce(Particle* particle, const real deltaTime)
{
    if (particle != nullptr && other != nullptr)
    {
        // Calculate the vector of the spring
        Vector3 force;

        particle->GetPosition(&force);

        force -= other->GetPosition();

        // Check if the bungee is compressed
        auto magnitude = force.Size();

        if (magnitude <= restLength)
        {
            return;
        }

        // Calculate the magnitude of the force
        magnitude = (magnitude - restLength) * springConstant;

        // Calculate the final force and apply it
        force.Normalize();

        force *= -magnitude;

        particle->AddForce(force);
    }
}
