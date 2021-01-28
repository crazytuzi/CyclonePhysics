#pragma once

#include "ParticleContact.h"

namespace cyclone
{
    /**
    * The contact resolution routine for particle contacts. One
    * resolver instance can be shared for the whole simulation.
    */
    class ParticleContactResolver
    {
    public:
        /**
        * Creates a new contact resolver.
        */
        ParticleContactResolver(unsigned iterations);

        /**
        * Sets the number of iterations that can be used.
        */
        void SetIterations(unsigned iterations);

        /**
        * Resolves a set of particle contacts for both penetration
        * and velocity.
        *
        * Contacts that cannot interact with each other should be
        * passed to separate calls to resolveContacts, as the
        * resolution algorithm takes much longer for lots of contacts
        * than it does for the same number of contacts in small sets.
        *
        * @param contactArray Pointer to an array of particle contact
        * objects.
        *
        * @param numContacts The number of contacts in the array to
        * resolve.
        *
        * @param deltaTime The deltaTime of the previous integration step.
        * This is used to compensate for forces applied.
        */
        void ResolveContacts(ParticleContact* contactArray, unsigned numContacts, real deltaTime);

    protected:
        /**
        * Holds the number of iterations allowed.
        */
        unsigned iterations;

        /**
        * This is a performance tracking value - we keep a record
        * of the actual number of iterations used.
        */
        unsigned iterationsUsed;
    };
}
