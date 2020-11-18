#include "ParticleForce/ParticleAnchoredBungee.h"
#include <cmath>

using namespace cyclone;

void ParticleAnchoredBungee::UpdateForce(Particle* particle, const real duration)
{
    if (particle != nullptr && anchor != nullptr)
    {
        // Calculate the vector of the spring
        Vector3 force;

        particle->GetPosition(&force);

        force -= *anchor;

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
