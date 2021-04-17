#pragma once

#include "RigidBody/FineCollision/CollisionSphere.h"
#include "RigidBody/FineCollision/CollisionPlane.h"
#include "RigidBody/FineCollision/CollisionBox.h"

namespace cyclone
{
    /**
    * A wrapper class that holds fast intersection tests. These
    * can be used to drive the coarse collision detection system or
    * as an early out in the full collision tests below.
    */
    class IntersectionTests
    {
    public:
        static bool SphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane);

        static bool SphereAndSphere(const CollisionSphere& one, const CollisionSphere& two);

        static bool BoxAndBox(const CollisionBox& one, const CollisionBox& two);

        /**
        * Does an intersection test on an arbitrarily aligned box and a
        * half-space.
        *
        * The box is given as a transform matrix, including
        * position, and a vector of half-sizes for the extend of the
        * box along each local axis.
        *
        * The half-space is given as a direction (i.e. unit) vector and the
        * offset of the limiting plane from the origin, along the given
        * direction.
        */
        static bool BoxAndHalfSpace(const CollisionBox& box, const CollisionPlane& plane);
    };
}
