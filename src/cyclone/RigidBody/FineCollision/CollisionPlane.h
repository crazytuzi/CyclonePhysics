#pragma once

#include "RigidBody/FineCollision/CollisionPrimitive.h"

namespace cyclone
{
    /**
    * The plane is not a primitive: it does not represent another
    * rigid body. It is used for contacts with the immovable
    * world geometry.
    */
    class CollisionPlane : public CollisionPrimitive
    {
    public:
        /**
        * The plane normal
        */
        Vector3 direction;

        /**
        * The distance of the plane from the origin.
        */
        real offset;
    };
}
