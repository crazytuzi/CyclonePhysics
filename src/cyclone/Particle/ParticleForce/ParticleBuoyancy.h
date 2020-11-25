#pragma once

#include "ParticleForceGenerator.h"

namespace cyclone
{
    /**
    * A force generator that applies a buoyancy force for a plane of
    * liquid parallel to XZ plane.
    */
    class ParticleBuoyancy : public ParticleForceGenerator
    {
    public:
        /** Creates a new buoyancy force with the given parameters. */
        ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity = 1000.0f);

        /** Applies the buoyancy force to the given particle. */
        void UpdateForce(Particle* particle, real duration) override;

    protected:
        /**
        * The maximum submersion depth of the object before
        * it generates its maximum buoyancy force.
        */
        real maxDepth;

        /**
        * The volume of the object.
        */
        real volume;

        /**
        * The height of the water plane above y=0. The plane will be
        * parallel to the XZ plane.
        */
        real waterHeight;

        /**
        * The density of the liquid. Pure water has a density of
        * 1000kg per cubic meter.
        */
        real liquidDensity;
    };
}
