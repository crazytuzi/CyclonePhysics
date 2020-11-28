#pragma once

#include "MassAggregateApplication.h"
#include <cyclone/Particle/ParticleContact/ParticleRod.h>

#define ROD_COUNT 15

#define BASE_MASS 1

#define EXTRA_MASS 10

class PlatformApplication : public MassAggregateApplication
{
public:
    PlatformApplication();

    virtual ~PlatformApplication();

    /** Returns the window title for the demo. */
    const char* GetTitle() override;

    /** Display the particles. */
    void Display() override;

    /** Update the particle positions. */
    void Update() override;

    /** Handle a key press. */
    void Key(unsigned char key) override;

private:
    /**
    * Updates particle masses to take into account the mass
    * that's on the platform.
    */
    void UpdateAdditionalMass();

private:
    cyclone::ParticleRod* rods;

    cyclone::Vector3 massPosition;

    cyclone::Vector3 massDisplayPosition;
};
