#pragma once

#include "Particle/Particle.h"
#include "Particle/ParticleForce/ParticleForceGenerator.h"
#include <vector>

namespace cyclone
{
    /**
    * Holds all the force generators and the particles they apply to.
    */
    class ParticleForceRegistry
    {
    public:
        /**
        * Registers the given force generator to apply to the
        * given particle.
        */
        void Add(Particle* particle, ParticleForceGenerator* forceGenerator);
        /**
        * Removes the given registered pair from the registry.
        * If the pair is not registered, this method will have
        * no effect.
        */
        void Remove(Particle* particle, ParticleForceGenerator* forceGenerator);

        /**
        * Clears all registrations from the registry. This will
        * not delete the particles or the force generators
        * themselves, just the records of their connection.
        */
        void Clear();

        /**
        * Calls all the force generators to update the forces of
        * their corresponding particles.
        */
        void UpdateForces(real duration);

    protected:
        /**
        * Keeps track of one force generator and the particle it
        * applies to.
        */
        struct ParticleForceRegistration
        {
            Particle* particle;

            ParticleForceGenerator* forceGenerator;

            ParticleForceRegistration(Particle* particle, ParticleForceGenerator* forceGenerator): particle(particle),
                forceGenerator(forceGenerator)
            {
            }
        };

        /**
        * Holds the list of registrations.
        */
        typedef std::vector<ParticleForceRegistration> Registry;

        Registry registrations;
    };
}
