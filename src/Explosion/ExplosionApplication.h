#pragma once

#define OBJECTS 5

#include "Ball.h"
#include "Box.h"
#include "RigidBodyApplication.h"

class ExplosionApplication : public RigidBodyApplication
{
public:
    /** Creates a new demo object. */
    ExplosionApplication();

    /** Sets up the rendering. */
    void StartUp() override;

    /** Returns the window title for the demo. */
    const char* GetTitle() override;

    /** Handles a key press. */
    void Key(unsigned char key) override;

    /** Display the particle positions. */
    void Display() override;

    /** Handle a mouse drag */
    void MouseDrag(int x, int y) override;

private:
    /** Detonates the explosion. */
    void Fire() const;

    /** Processes the contact generation code. */
    void GenerateContacts() override;

    /** Processes the objects in the simulation forward in time. */
    void UpdateObjects(cyclone::real deltaTime) override;

    /** Resets the position of all the blocks. */
    void Reset() override;

private:
    bool editMode;

    bool upMode;

    /**
    * Holds the number of boxes in the simulation.
    */
    const static unsigned boxes = OBJECTS;

    /** Holds the box data. */
    Box boxData[boxes];

    /**
    * Holds the number of balls in the simulation.
    */
    const static unsigned balls = OBJECTS;

    /** Holds the ball data. */
    Ball ballData[balls];
};
