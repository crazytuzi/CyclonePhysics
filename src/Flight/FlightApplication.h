#pragma once

#include "Application.h"
#include "RigidBody/RigidBody.h"
#include "RigidBody/Force/ForceRegistry.h"
#include "RigidBody/Force/Aero.h"
#include "RigidBody/Force/AeroControl.h"

class FlightApplication : public Application
{
public:
    FlightApplication();

    virtual ~FlightApplication();

    /** Returns the window title for the demo. */
    const char* GetTitle() override;

    /** Display the particles. */
    void Display() override;

    /** Update the particle positions. */
    void Update() override;

    /** Handle a key press. */
    void Key(unsigned char key) override;

private:
    void ResetPlane();

private:
    cyclone::AeroControl left_wing;

    cyclone::AeroControl right_wing;

    cyclone::AeroControl rudder;

    cyclone::Aero tail;

    cyclone::RigidBody aircraft;

    cyclone::ForceRegistry registry;

    cyclone::Vector3 windspeed;

    cyclone::real left_wing_control;

    cyclone::real right_wing_control;

    cyclone::real rudder_control;
};
