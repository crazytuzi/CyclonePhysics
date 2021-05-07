#pragma once

#include "Application.h"
#include "cyclone/RigidBody/Contact/Contact.h"
#include "cyclone/RigidBody/FineCollision/CollisionDetector.h"
#include "cyclone/RigidBody/Contact/ContactResolver.h"

/**
* This application adds additional functionality used in many of the
* demos. This includes the ability to track contacts (for rigid bodies)
* and move the camera around.
*/
class RigidBodyApplication : public Application
{
public:
    /**
    * Creates a new application object.
    */
    RigidBodyApplication();

    /** Display the application. */
    void Display() override;

    /** Update the objects. */
    void Update() override;

    /** Handle a mouse click. */
    void Mouse(int button, int state, int x, int y) override;

    /** Handle a mouse drag */
    void MouseDrag(int x, int y) override;

    /** Handles a key press. */
    void Key(unsigned char key) override;

protected:
    /** Processes the contact generation code. */
    virtual void GenerateContacts() = 0;

    /** Processes the objects in the simulation forward in time. */
    virtual void UpdateObjects(cyclone::real deltaTime) = 0;

    /**
    * Finishes drawing the frame, adding debugging information
    * as needed.
    */
    void DrawDebug();

    /** Resets the simulation. */
    virtual void Reset() = 0;

protected:
    /** Holds the maximum number of contacts. */
    const static unsigned maxContacts = 256;

    /** Holds the array of contacts. */
    cyclone::Contact contacts[maxContacts];

    /** Holds the collision data structure for collision detection. */
    cyclone::CollisionData collisionData;

    /** Holds the contact resolver. */
    cyclone::ContactResolver resolver;

    /** Holds the camera angle. */
    float theta;

    /** Holds the camera elevation. */
    float phi;

    /** Holds the position of the mouse at the last frame of a drag. */
    int last_x, last_y;

    /** True if the contacts should be rendered. */
    bool renderDebugInfo;

    /** True if the simulation is paused. */
    bool pauseSimulation;

    /** Pauses the simulation after the next frame automatically */
    bool autoPauseSimulation;
};
