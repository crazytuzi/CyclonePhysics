#pragma once

#include "ParticleAnchoredSpring.h"

namespace cyclone
{
    class ParticleAnchoredBungee : public ParticleAnchoredSpring
    {
    public:
        /** Applies the spring force to the given particle. */
        void UpdateForce(Particle* particle, real duration) override;
    };
}
