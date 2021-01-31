#pragma once

#include <cyclone/Particle/ParticleForce/ParticleForceGenerator.h>

#ifndef BLOB_COUNT
#define BLOB_COUNT 5
#endif

/**
* A force generator for proximal attraction.
*/
class BlobForceGenerator : public cyclone::ParticleForceGenerator
{
public:
    void UpdateForce(cyclone::Particle* particle, cyclone::real deltaTime) override;

public:
    /**
    * Holds a pointer to the particles we might be attracting.
    */
    cyclone::Particle* particles;

    /**
    * The maximum force used to push the particles apart.
    */
    cyclone::real maxRepulsion;

    /**
    * The maximum force used to pull particles together.
    */
    cyclone::real maxAttraction;

    /**
    * The separation between particles where there is no force.
    */
    cyclone::real minNaturalDistance, maxNaturalDistance;

    /**
    * The force with which to float the head particle, if it is
    * joined to others.
    */
    cyclone::real floatHead;

    /**
    * The maximum number of particles in the blob before the head
    * is floated at maximum force.
    */
    unsigned maxFloat;

    /**
    * The separation between particles after which they 'break' apart and
    * there is no force.
    */
    cyclone::real maxDistance;
};
