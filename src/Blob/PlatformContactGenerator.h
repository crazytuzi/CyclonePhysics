#pragma once

#include <cyclone/Particle/ParticleContact/ParticleContactGenerator.h>

#ifndef BLOB_COUNT
#define BLOB_COUNT 5
#endif

#ifndef BLOB_RADIUS
#define BLOB_RADIUS 0.4f
#endif

/**
* Platforms are two dimensional: lines on which the
* particles can rest. Platforms are also contact generators for the physics.
*/
class PlatformContactGenerator : public cyclone::ParticleContactGenerator
{
public:
    unsigned AddContact(cyclone::ParticleContact* contact, unsigned limit) const override;

public:
    cyclone::Vector3 start;

    cyclone::Vector3 end;

    cyclone::Particle* particles;
};
