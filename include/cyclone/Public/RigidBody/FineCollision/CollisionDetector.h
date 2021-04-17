#pragma once
#include "CollisionBox.h"
#include "CollisionPlane.h"
#include "CollisionSphere.h"
#include "RigidBody/Contact/Contact.h"

namespace cyclone
{
    /**
     * A helper structure that contains information for the detector to use
     * in building its contact data.
     */
    struct CollisionData
    {
        /**
        * Checks if there are more contacts available in the contact
        * data.
        */
        bool HasMoreContacts() const
        {
            return contactsLeft > 0;
        }

        /**
        * Resets the data so that it has no used contacts recorded.
        */
        void Reset(const unsigned maxContacts)
        {
            contactsLeft = maxContacts;

            contactCount = 0;

            contacts = contactHead;
        }

        /**
        * Notifies the data that the given number of contacts have
        * been added.
        */
        void AddContacts(const unsigned count)
        {
            // Reduce the number of contacts remaining, add number used
            contactsLeft -= count;

            contactCount += count;

            // Move the array forward
            contacts += count;
        }

        /**
        * Holds the base of the collision data: the first contact
        * in the array. This is used so that the contact pointer (below)
        * can be incremented each time a contact is detected, while
        * this pointer points to the first contact found.
        */
        Contact* contactHead;

        /** Holds the contact array to write into. */
        Contact* contacts;

        /** Holds the maximum number of contacts the array can take. */
        int contactsLeft;

        /** Holds the number of contacts found so far. */
        unsigned contactCount;

        /** Holds the friction value to write into any collisions. */
        real friction;

        /** Holds the restitution value to write into any collisions. */
        real restitution;

        /**
        * Holds the collision tolerance, even uncolliding objects this
        * close should have collisions generated.
        */
        real tolerance;
    };

    /**
    * A wrapper class that holds the fine grained collision detection
    * routines.
    *
    * Each of the functions has the same format: it takes the details
    * of two objects, and a pointer to a contact array to fill. It
    * returns the number of contacts it wrote into the array.
    */
    class CollisionDetector
    {
    public:
        static unsigned SphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane,
                                           CollisionData* data);

        static unsigned SphereAndTruePlane(const CollisionSphere& sphere, const CollisionPlane& plane,
                                           CollisionData* data);

        static unsigned SphereAndSphere(const CollisionSphere& one, const CollisionSphere& two, CollisionData* data);

        /**
        * Does a collision test on a collision box and a plane representing
        * a half-space (i.e. the normal of the plane
        * points out of the half-space).
        */
        static unsigned BoxAndHalfSpace(const CollisionBox& box, const CollisionPlane& plane, CollisionData* data);

        static unsigned BoxAndBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data);

        static unsigned BoxAndPoint(const CollisionBox& box, const Vector3& point, CollisionData* data);

        static unsigned BoxAndSphere(const CollisionBox& box, const CollisionSphere& sphere, CollisionData* data);
    };
}
