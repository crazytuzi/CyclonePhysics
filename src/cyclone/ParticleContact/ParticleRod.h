#pragma once

#include "ParticleLink.h"

namespace cyclone
{
    /**
    * Rods link a pair of particles, generating a contact if they
    * stray too far apart or too close.
    */
    class ParticleRod : public ParticleLink
    {
    public:
        /**
        * Fills the given contact structure with the contact needed
        * to keep the rod from extending or compressing.
        */
        unsigned AddContact(ParticleContact* contact, unsigned limit) const override;

    public:
        /**
        * Holds the length of the rod.
        */
        real length;
    };
}
