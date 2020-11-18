#include "ParticleForce/ParticleGravity.h"

using namespace cyclone;

ParticleGravity::ParticleGravity(const Vector3& gravity): gravity(gravity)
{
}

void ParticleGravity::UpdateForce(Particle* particle, real duration)
{
    if (particle != nullptr)
    {
        // Check that we do not have infinite mass
        if (!particle->HasFiniteMass())
        {
            return;
        }

        // Apply the mass-scaled force to the particle
        particle->AddForce(gravity * particle->GetMass());
    }
}
