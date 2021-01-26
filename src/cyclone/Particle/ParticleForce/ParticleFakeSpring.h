#pragma once

#include "ParticleForceGenerator.h"

namespace cyclone
{
    /**
    * A force generator that fakes a stiff spring force, and where
    * one end is attached to a fixed point in space.
    */
    class ParticleFakeSpring : public ParticleForceGenerator
    {
    public:
        /** Creates a new spring with the given parameters. */
        ParticleFakeSpring(Vector3* anchor, real springConstant, real damping);

        /** Applies the spring force to the given particle. */
        void UpdateForce(Particle* particle, real deltaTime) override;

    protected:
        /** The location of the anchored end of the spring. */
        Vector3* anchor;

        /** Holds the sprint constant. */
        real springConstant;

        /** Holds the damping on the oscillation of the spring. */
        real damping;
    };
}
