#pragma once

#include "ParticleConstraint.h"

namespace cyclone
{
    /**
    * Cables link a particle to an anchor point, generating a contact if they
    * stray too far apart.
    */
    class ParticleCableConstraint : public ParticleConstraint
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
