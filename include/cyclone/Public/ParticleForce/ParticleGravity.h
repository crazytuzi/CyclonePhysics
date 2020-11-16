#pragma once

#include "ParticleForceGenerator.h"

namespace cyclone
{
    /**
    * A force generator that applies a gravitational force. One instance
    * can be used for multiple particles.
    */
    class ParticleGravity : public ParticleForceGenerator
    {
    public:
        /** Creates the generator with the given acceleration. */
        ParticleGravity(const Vector3& gravity);

        /** Applies the gravitational force to the given particle. */
        void UpdateForce(Particle* particle, real duration) override;
    private:
        /** Holds the acceleration due to gravity. */
        Vector3 gravity;
    };
}
