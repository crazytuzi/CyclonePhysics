#include "Particle/ParticleForce/ParticleAnchoredBungee.h"

using namespace cyclone;

void ParticleAnchoredBungee::UpdateForce(Particle* particle, const real deltaTime)
{
    if (particle != nullptr && anchor != nullptr)
    {
        // Calculate the vector of the spring
        Vector3 force;

        particle->GetPosition(&force);

        force -= *anchor;

        // Calculate the magnitude of the force
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
