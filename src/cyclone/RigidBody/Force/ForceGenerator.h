#pragma once

/**
* @file
*
* This file contains the interface and sample force generators.
*/

#include "RigidBody/RigidBody.h"

namespace cyclone
{
    /**
    * A force generator can be asked to add a force to one or more
    * bodies.
    */
    class ForceGenerator
    {
    public:
        /**
        * Overload this in implementations of the interface to calculate
        * and update the force applied to the given rigid body.
        */
        virtual void UpdateForce(RigidBody* body, real deltaTime) = 0;
    };
}
