#include "ParticleForceGenerator/ParticleGravity.h"

cyclone::ParticleGravity::ParticleGravity(const Vector3& gravity): gravity(gravity)
{
}

void cyclone::ParticleGravity::UpdateForce(Particle* particle, real duration)
{
    // Check that we do not have infinite mass
    if (!particle->HasFiniteMass())
    {
        return;
    }

    // Apply the mass-scaled force to the particle
    particle->AddForce(gravity * particle->GetMass());
}
