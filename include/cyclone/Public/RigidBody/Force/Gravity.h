#pragma once

#include "ForceGenerator.h"

namespace cyclone
{
    /**
    * A force generator that applies a gravitational force. One instance
    * can be used for multiple rigid bodies.
    */
    class Gravity : public ForceGenerator
    {
    public:
        /** Creates the generator with the given acceleration. */
        Gravity(const Vector3& gravity);

        /** Applies the gravitational force to the given rigid body. */
        void UpdateForce(RigidBody* body, real deltaTime) override;

    private:
        /** Holds the acceleration due to gravity. */
        Vector3 gravity;
    };
}
