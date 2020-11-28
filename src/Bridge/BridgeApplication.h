#pragma once

#include "MassAggregateApplication.h"
#include "cyclone/Particle/ParticleContact/ParticleRod.h"
#include "cyclone/Particle/ParticleContact/ParticleCable.h"
#include "cyclone/Particle/ParticleContact/ParticleCableConstraint.h"

#define ROD_COUNT 6

#define CABLE_COUNT 10

#define SUPPORT_COUNT 12

#define BASE_MASS 1

#define EXTRA_MASS 10

class BridgeApplication : public MassAggregateApplication
{
public:
    BridgeApplication();

    virtual ~BridgeApplication();

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
    * that's crossing the bridge.
    */
    void UpdateAdditionalMass();

private:
    cyclone::ParticleCableConstraint* supports;

    cyclone::ParticleCable* cables;

    cyclone::ParticleRod* rods;

    cyclone::Vector3 massPosition;

    cyclone::Vector3 massDisplayPosition;
};
