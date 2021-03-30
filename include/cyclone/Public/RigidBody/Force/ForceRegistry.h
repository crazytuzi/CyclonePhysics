#pragma once

#include "ForceGenerator.h"
#include "RigidBody/RigidBody.h"
#include <vector>

namespace cyclone
{
    /**
    * Holds all the force generators and the bodies they apply to.
    */
    class ForceRegistry
    {
    public:
        /**
        * Registers the given force generator to apply to the
        * given body.
        */
        void Add(RigidBody* body, ForceGenerator* forceGenerator);

        /**
        * Removes the given registered pair from the registry.
        * If the pair is not registered, this method will have
        * no effect.
        */
        void Remove(RigidBody* body, ForceGenerator* forceGenerator);

        /**
        * Clears all registrations from the registry. This will
        * not delete the bodies or the force generators
        * themselves, just the records of their connection.
        */
        void Clear();

        /**
        * Calls all the force generators to update the forces of
        * their corresponding bodies.
        */
        void UpdateForces(real deltaTime);

    protected:
        /**
        * Keeps track of one force generator and the body it
        * applies to.
        */
        struct ForceRegistration
        {
            RigidBody* body;
            ForceGenerator* forceGenerator;
        };

        /**
        * Holds the list of registrations.
        */
        typedef std::vector<ForceRegistration> Registry;

        Registry registrations;
    };
}
