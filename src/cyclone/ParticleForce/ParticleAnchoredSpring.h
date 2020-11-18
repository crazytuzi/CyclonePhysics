#pragma once

#include "ParticleForceGenerator.h"

namespace cyclone
{
    /**
    * A force generator that applies a Spring force, where
    * one end is attached to a fixed point in space.
    */
    class ParticleAnchoredSpring : public ParticleForceGenerator
    {
    public:
        /** Creates a new spring with the given parameters. */
        ParticleAnchoredSpring(Vector3* anchor, real springConstant, real restLength);

        /** Applies the spring force to the given particle. */
        void UpdateForce(Particle* particle, real duration) override;

    protected:
        /** The location of the anchored end of the spring. */
        Vector3* anchor;

        /** Holds the sprint constant. */
        real springConstant;

        /** Holds the rest length of the spring. */
        real restLength;
    };
}
