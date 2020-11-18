#pragma once

#include "ParticleForceGenerator.h"

namespace cyclone
{
    /**
    * A force generator that applies a Spring force.
    */
    class ParticleSpring : public ParticleForceGenerator
    {
    public:
        /** Creates a new spring with the given parameters. */
        ParticleSpring(Particle* other,
                       real springConstant, real restLength);

        /** Applies the spring force to the given particle. */
        void UpdateForce(Particle* particle, real duration) override;

    protected:
        /** The particle at the other end of the spring. */
        Particle* other;

        /** Holds the sprint constant. */
        real springConstant;

        /** Holds the rest length of the spring. */
        real restLength;
    };
}
