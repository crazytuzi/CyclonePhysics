#pragma once

#define NUM_BONES 12

#define NUM_JOINTS 11

#include "Bone.h"
#include "RigidBodyApplication.h"
#include "cyclone/RigidBody/Contact/Joint.h"

class RagdollApplication : public RigidBodyApplication
{
public:
    /** Creates a new demo object. */
    RagdollApplication();

    /** Sets up the rendering. */
    void StartUp() override;

    /** Returns the window title for the demo. */
    const char* GetTitle() override;

    /** Display the particle positions. */
    void Display() override;

private:
    cyclone::Random random;

    /** Holds the bone bodies. */
    Bone bones[NUM_BONES];

    /** Holds the joints. */
    cyclone::Joint joints[NUM_JOINTS];

    /** Processes the contact generation code. */
    void GenerateContacts() override;

    /** Processes the objects in the simulation forward in time. */
    void UpdateObjects(cyclone::real deltaTime) override;

    /** Resets the position of all the bones. */
    void Reset() override;
};
