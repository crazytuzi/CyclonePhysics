#pragma once

#define MAX_BLOCKS 9

#include "Block.h"
#include "RigidBodyApplication.h"

class FractureApplication : public RigidBodyApplication
{
public:
    /** Creates a new demo object. */
    FractureApplication();

    /** Returns the window title for the demo. */
    const char* GetTitle() override;

    /** Display the particle positions. */
    void Display() override;

private:
    /** Processes the contact generation code. */
    void GenerateContacts() override;

    /** Processes the objects in the simulation forward in time. */
    void UpdateObjects(cyclone::real deltaTime) override;

    /** Resets the position of all the blocks. */
    void Reset() override;

    /** Processes the physics. */
    void Update() override;

private:
    /** Tracks if a block has been hit. */
    bool hit;

    bool ball_active;

    unsigned fracture_contact;

    /** Handle random numbers. */
    cyclone::Random random;

    /** Holds the bodies. */
    Block blocks[MAX_BLOCKS];

    /** Holds the projectile. */
    cyclone::CollisionSphere ball;
};
