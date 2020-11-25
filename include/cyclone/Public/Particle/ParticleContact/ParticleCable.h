#pragma once

#include "ParticleLink.h"

namespace cyclone
{
    /**
    * Cables link a pair of particles, generating a contact if they
    * stray too far apart.
    */
    class ParticleCable : public ParticleLink
    {
    public:
        /**
        * Fills the given contact structure with the contact needed
        * to keep the cable from over-extending.
        */
        unsigned AddContact(ParticleContact* contact, unsigned limit) const override;

    public:
        /**
        * Holds the maximum length of the cable.
        */
        real maxLength;

        /**
        * Holds the restitution (bounciness) of the cable.
        */
        real restitution;
    };
}
