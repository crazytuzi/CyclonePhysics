#pragma once

#include "ParticleContactGenerator.h"

namespace cyclone
{
    /**
    * Links connect two particles together, generating a contact if
    * they violate the constraints of their link. It is used as a
    * base class for cables and rods, and could be used as a base
    * class for springs with a limit to their extension..
    */
    class ParticleLink : public ParticleContactGenerator
    {
    public:
        /**
        * Generates the contacts to keep this link from being
        * violated. This class can only ever generate a single
        * contact, so the pointer can be a pointer to a single
        * element, the limit parameter is assumed to be at least one
        * (zero isn't valid) and the return value is either 0, if the
        * cable wasn't over-extended, or one if a contact was needed.
        *
        * NB: This method is declared in the same way (as pure
        * virtual) in the parent class, but is replicated here for
        * documentation purposes.
        */
        unsigned AddContact(ParticleContact* contact, unsigned limit) const override = 0;

    protected:
        /**
        * Returns the current length of the link.
        */
        real CurrentLength() const;

    public:
        /**
        * Holds the pair of particles that are connected by this link.
        */
        Particle* particle[2];
    };
}
