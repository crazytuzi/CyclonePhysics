#include "Particle/ParticleForce/ParticleBuoyancy.h"

using namespace cyclone;

ParticleBuoyancy::ParticleBuoyancy(const real maxDepth, const real volume, const real waterHeight,
                                   const real liquidDensity): maxDepth(maxDepth), volume(volume),
                                                              waterHeight(waterHeight), liquidDensity(liquidDensity)
{
}

void ParticleBuoyancy::UpdateForce(Particle* particle, real duration)
{
    if (particle != nullptr)
    {
        // Calculate the submersion depth
        const auto depth = particle->GetPosition().y;

        // Check if we're out of the water
        if (depth >= waterHeight + maxDepth)
        {
            return;
        }

        Vector3 force;

        // Check if we're at maximum depth
        if (depth <= waterHeight - maxDepth)
        {
            force.y = liquidDensity * volume;

            particle->AddForce(force);

            return;
        }

        // Otherwise we are partly submerged
        force.y = liquidDensity * volume * (depth - maxDepth - waterHeight) / (2.f * maxDepth);

        particle->AddForce(-force);
    }
}
