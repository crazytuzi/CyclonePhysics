#pragma once

#include "Application.h"
#include "RigidBody/RigidBody.h"
#include "RigidBody/Force/Buoyancy.h"
#include "RigidBody/Force/Aero.h"
#include "RigidBody/Force/ForceRegistry.h"

class SailboatApplication : public Application
{
public:
    /** Creates a new demo object. */
    SailboatApplication();

    virtual ~SailboatApplication();

    /** Returns the window title for the demo. */
    const char* GetTitle() override;

    /** Display the particles. */
    void Display() override;

    /** Update the particle positions. */
    void Update() override;

    /** Handle a key press. */
    void Key(unsigned char key) override;

private:
    cyclone::Buoyancy buoyancy;

    cyclone::Aero sail;

    cyclone::RigidBody sailboat;

    cyclone::ForceRegistry registry;

    cyclone::Random r;

    cyclone::Vector3 windspeed;

    cyclone::real sail_control;
};
