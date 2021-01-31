#pragma once

#include "cyclone/Particle/Particle.h"

/**
* Fireworks are particles, with additional data for rendering and
* evolution.
*/
class Firework : public cyclone::Particle
{
public:
    /**
    * Updates the firework by the given deltaTime of time. Returns true
    * if the firework has reached the end of its life and needs to be
    * removed.
    */
    bool Update(cyclone::real deltaTime);

public:
    /** Fireworks have an integer type, used for firework rules. */
    unsigned type;

    /**
    * The age of a firework determines when it detonates. Age gradually
    * decreases, when it passes zero the firework delivers its payload.
    * Think of age as fuse-left.
    */
    cyclone::real age;
};
