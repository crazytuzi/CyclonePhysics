#pragma once

#include "ForceGenerator.h"

namespace cyclone
{
    /**
     * A force generator that applies a Spring force.
     */
    class Spring : public ForceGenerator
    {
    public:
        /** Creates a new spring with the given parameters. */
        Spring(const Vector3& localConnectionPoint, RigidBody* other, const Vector3& otherConnectionPoint,
               real springConstant, real restLength);

        /** Applies the gravitational force to the given rigid body. */
        void UpdateForce(RigidBody* body, real deltaTime) override;

    private:
        /**
        * The point of connection of the spring, in local
        * coordinates.
        */
        Vector3 connectionPoint;

        /**
        * The point of connection of the spring to the other object,
        * in that object's local coordinates.
        */
        Vector3 otherConnectionPoint;

        /** The particle at the other end of the spring. */
        RigidBody* other;

        /** Holds the sprint constant. */
        real springConstant;

        /** Holds the rest length of the spring. */
        real restLength;
    };
}
