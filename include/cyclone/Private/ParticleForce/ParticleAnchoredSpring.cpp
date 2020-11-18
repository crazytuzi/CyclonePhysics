#include "ParticleForce/ParticleAnchoredSpring.h"

cyclone::ParticleAnchoredSpring::ParticleAnchoredSpring(Vector3* anchor, const real springConstant,
                                                        const real restLength):
    anchor(anchor), springConstant(springConstant), restLength(restLength)
{
}

void cyclone::ParticleAnchoredSpring::UpdateForce(Particle* particle, const real duration)
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

        magnitude = magnitude - restLength;

        magnitude *= springConstant;

        // Calculate the final force and apply it
        force.Normalize();

        force *= -magnitude;

        particle->AddForce(force);
    }
}
