#pragma once

#include "ForceGenerator.h"

namespace cyclone
{
    /**
    * A force generator that applies an aerodynamic force.
    */
    class Aero : public ForceGenerator
    {
    public:
        /**
        * Creates a new aerodynamic force generator with the
        * given properties.
        */
        Aero(const Matrix& tensor, const Vector3& position, const Vector3* windspeed);

        /**
        * Applies the force to the given rigid body.
        */
        void UpdateForce(RigidBody* body, real deltaTime) override;

    protected:
        /**
        * Uses an explicit tensor matrix to update the force on
        * the given rigid body. This is exactly the same as for updateForce
        * only it takes an explicit tensor.
        */
        void UpdateForceFromTensor(RigidBody* body, real deltaTime, const Matrix& tensor) const;

    protected:
        /**
        * Holds the aerodynamic tensor for the surface in body
        * space.
        */
        Matrix tensor;

        /**
        * Holds the relative position of the aerodynamic surface in
        * body coordinates.
        */
        Vector3 position;

        /**
        * Holds a pointer to a vector containing the windspeed of the
        * environment. This is easier than managing a separate
        * windspeed vector per generator and having to update it
        * manually as the wind changes.
        */
        const Vector3* windspeed;
    };
}
