#pragma once

#include "ParticleForceGenerator.h"

namespace cyclone
{
    /**
    * A force generator that applies a drag force. One instance
    * can be used for multiple particles.
    */
    class ParticleDrag : public ParticleForceGenerator
    {
    public:
        /** Creates the generator with the given coefficients. */
        ParticleDrag(real k1, real k2);

        /** Applies the drag force to the given particle. */
        void UpdateForce(Particle* particle, real deltaTime) override;

    protected:
        /** Holds the velocity drag coefficient. */
        real k1;

        /** Holds the velocity squared drag coefficient. */
        real k2;
    };
}
