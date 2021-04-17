#pragma once

#include "RigidBody/Contact/Contact.h"

namespace cyclone
{
    /**
    * Represents a primitive to detect collisions against.
    */
    class CollisionPrimitive
    {
    public:
        /**
        * Calculates the internals for the primitive.
        */
        void CalculateInternals();

        /**
        * This is a convenience function to allow access to the
        * axis vectors in the transform for this primitive.
        */
        Vector3 GetAxis(unsigned index) const;

        /**
        * Returns the resultant transform of the primitive, calculated from
        * the combined offset of the primitive and the transform
        * (orientation + position) of the rigid body to which it is
        * attached.
        */
        const Matrix& GetTransform() const;

    public:
        /**
        * The rigid body that is represented by this primitive.
        */
        RigidBody* body;

        /**
        * The offset of this primitive from the given rigid body.
        */
        Matrix offset;

    protected:
        /**
        * The resultant transform of the primitive. This is
        * calculated by combining the offset of the primitive
        * with the transform of the rigid body.
        */
        Matrix transform;

    private:
        /**
        * This class exists to help the collision detector
        * and intersection routines, so they should have
        * access to its data.
        */
        friend class IntersectionTests;

        friend class CollisionDetector;
    };
}
