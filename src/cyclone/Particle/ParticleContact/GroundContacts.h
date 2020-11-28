#pragma once

#include "ParticleContactGenerator.h"
#include "cyclone/Particle/ParticleWorld.h"

namespace cyclone
{
    /**
    * A contact generator that takes an STL vector of particle pointers and
    * collides them against the ground.
    */
    class GroundContacts : public ParticleContactGenerator
    {
    public:
        GroundContacts();

        void Init(ParticleWorld::Particles* particles);

        unsigned AddContact(ParticleContact* contact, unsigned limit) const override;

    private:
        ParticleWorld::Particles* particles;
    };
}
