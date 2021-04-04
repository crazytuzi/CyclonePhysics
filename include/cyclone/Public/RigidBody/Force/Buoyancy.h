#pragma once

#include "ForceGenerator.h"

namespace cyclone
{
    /**
    * A force generator to apply a buoyant force to a rigid body.
    */
    class Buoyancy : public ForceGenerator
    {
    public:
        /** Creates a new buoyancy force with the given parameters. */
        Buoyancy(const Vector3& centreOfBuoyancy, real maxDepth, real volume, real waterHeight,
                 real liquidDensity = 1000.f);

        /** Applies the gravitational force to the given rigid body. */
        void UpdateForce(RigidBody* body, real deltaTime) override;

    private:
        /**
        * The maximum submersion depth of the object before
        * it generates its maximum buoyancy force.
        */
        real maxDepth;

        /**
        * The volume of the object.
        */
        real volume;

        /**
        * The height of the water plane above y=0. The plane will be
        * parallel to the XZ plane.
        */
        real waterHeight;

        /**
        * The density of the liquid. Pure water has a density of
        * 1000kg per cubic meter.
        */
        real liquidDensity;

        /**
        * The centre of buoyancy of the rigid body, in body coordinates.
        */
        Vector3 centreOfBuoyancy;
    };
}
