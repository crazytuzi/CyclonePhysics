#pragma once

/**
* @file
*
* This file contains the interface particle force generators.
*/

#include "Particle/Particle.h"

namespace cyclone
{
    /**
    * A force generator can be asked to add a force to one or more
    * particles.
    */
    class ParticleForceGenerator
    {
    public:
        /**
        * Overload this in implementations of the interface to calculate
        * and update the force applied to the given particle.
        */
        virtual void UpdateForce(Particle* particle, real deltaTime) = 0;
    };
}
