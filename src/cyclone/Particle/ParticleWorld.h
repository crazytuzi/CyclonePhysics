#pragma once

/**
* @file
*
* This file contains the definitions for a structure to hold any number o
* particle masses, and their connections.
*/

#include <vector>
#include "Particle.h"
#include "ParticleContact/ParticleContactGenerator.h"
#include "ParticleContact/ParticleContactResolver.h"
#include "ParticleForce/ParticleForceRegistry.h"

namespace cyclone
{
    /**
    * Keeps track of a set of particles, and provides the means to
    * update them all.
    */
    class ParticleWorld
    {
    public:
        typedef std::vector<Particle*> Particles;

        typedef std::vector<ParticleContactGenerator*> ContactGenerators;

    public:
        /**
        * Creates a new particle simulator that can handle up to the
        * given number of contacts per frame. You can also optionally
        * give a number of contact-resolution iterations to use. If you
        * don't give a number of iterations, then twice the number of
        * contacts will be used.
        */
        ParticleWorld(unsigned maxContacts, unsigned iterations = 0);

        /**
        * Deletes the simulator.
        */
        ~ParticleWorld();

        /**
        * Calls each of the registered contact generators to report
        * their contacts. Returns the number of generated contacts.
        */
        unsigned GenerateContacts();

        /**
        * Integrates all the particles in this world forward in time
        * by the given duration.
        */
        void Integrate(real duration);

        /**
        * Processes all the physics for the particle world.
        */
        void RunPhysics(real duration);

        /**
        * Initializes the world for a simulation frame. This clears
        * the force accumulators for particles in the world. After
        * calling this, the particles can have their forces for this
        * frame added.
        */
        void StartFrame();

        /**
        *  Returns the list of particles.
        */
        Particles& GetParticles();

        /**
        * Returns the list of contact generators.
        */
        ContactGenerators& GetContactGenerators();

        /**
        * Returns the force registry.
        */
        ParticleForceRegistry& GetForceRegistry();

    protected:
        /**
        * Holds the particles
        */
        Particles particles;

        /**
        * True if the world should calculate the number of iterations
        * to give the contact resolver at each frame.
        */
        bool bCalculateIterations;

        /**
        * Holds the force generators for the particles in this world.
        */
        ParticleForceRegistry registry;

        /**
        * Holds the resolver for contacts.
        */
        ParticleContactResolver resolver;

        /**
        * Contact generators.
        */
        ContactGenerators contactGenerators;

        /**
        * Holds the list of contacts.
        */
        ParticleContact* contacts;

        /**
        * Holds the maximum number of contacts allowed (i.e. the
        * size of the contacts array).
        */
        unsigned maxContacts;
    };
}
