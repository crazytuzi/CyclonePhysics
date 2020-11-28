#pragma once

#include "Application.h"
#include "cyclone/Particle/ParticleWorld.h"
#include "cyclone/Particle/ParticleContact/GroundContacts.h"

/**
* This application adds additional functionality used in the mass-aggregate demos.
*/
class MassAggregateApplication : public Application
{
public:
    MassAggregateApplication(unsigned int particleCount);

    virtual ~MassAggregateApplication();

    /** Update the particle positions. */
    void Update() override;

    /** Sets up the graphic rendering. */
    void StartUp() override;

    /** Display the particles. */
    void Display() override;

protected:
    cyclone::ParticleWorld world;

    cyclone::Particle* particles;

    cyclone::GroundContacts groundContactGenerator;
};
