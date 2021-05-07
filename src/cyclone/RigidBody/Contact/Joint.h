#pragma once

#include "ContactGenerator.h"

namespace cyclone
{
    class Joint : public ContactGenerator
    {
    public:
        /**
        * Configures the joint in one go.
        */
        void Set(RigidBody* a, const Vector3& a_pos, RigidBody* b, const Vector3& b_pos, real error);

        /**
        * Generates the contacts required to restore the joint if it
        * has been violated.
        */
        unsigned AddContact(Contact* contact, unsigned limit) const override;

    public:
        /**
        * Holds the two rigid bodies that are connected by this joint.
        */
        RigidBody* body[2];

        /**
        * Holds the relative location of the connection for each
        * body, given in local coordinates.
        */
        Vector3 position[2];

        /**
        * Holds the maximum displacement at the joint before the
        * joint is considered to be violated. This is normally a
        * small, epsilon value.  It can be larger, however, in which
        * case the joint will behave as if an inelastic cable joined
        * the bodies at their joint locations.
        */
        real error;
    };
}
