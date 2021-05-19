#pragma once

#include "Contact.h"
#include "RigidBody/RigidBody.h"

namespace cyclone
{
    /**
     * The contact resolution routine. One resolver instance
     * can be shared for the whole simulation, as long as you need
     * roughly the same parameters each time (which is normal).
     *
     * @section algorithm Resolution Algorithm
     *
     * The resolver uses an iterative satisfaction algorithm; it loops
     * through each contact and tries to resolve it. Each contact is
     * resolved locally, which may in turn put other contacts in a worse
     * position. The algorithm then revisits other contacts and repeats
     * the process up to a specified iteration limit. It can be proved
     * that given enough iterations, the simulation will get to the
     * correct result. As with all approaches, numerical stability can
     * cause problems that make a correct resolution impossible.
     *
     * @subsection strengths Strengths
     *
     * This algorithm is very fast, much faster than other physics
     * approaches. Even using many more iterations than there are
     * contacts, it will be faster than global approaches.
     *
     * Many global algorithms are unstable under high friction, this
     * approach is very robust indeed for high friction and low
     * restitution values.
     *
     * The algorithm produces visually believable behaviour. Tradeoffs
     * have been made to err on the side of visual realism rather than
     * computational expense or numerical accuracy.
     *
     * @subsection weaknesses Weaknesses
     *
     * The algorithm does not cope well with situations with many
     * inter-related contacts: stacked boxes, for example. In this
     * case the simulation may appear to jiggle slightly, which often
     * dislodges a box from the stack, allowing it to collapse.
     *
     * Another issue with the resolution mechanism is that resolving
     * one contact may make another contact move sideways against
     * friction, because each contact is handled independently, this
     * friction is not taken into account. If one object is pushing
     * against another, the pushed object may move across its support
     * without friction, even though friction is set between those bodies.
     *
     * In general this resolver is not suitable for stacks of bodies,
     * but is perfect for handling impact, explosive, and flat resting
     * situations.
     */
    class ContactResolver
    {
    public:
        /**
        * Creates a new contact resolver with the given number of iterations
        * per resolution call, and optional epsilon values.
        */
        ContactResolver(unsigned iterations, real velocityEpsilon = 0.01f, real positionEpsilon = 0.01f);

        /**
        * Creates a new contact resolver with the given number of iterations
        * for each kind of resolution, and optional epsilon values.
        */
        ContactResolver(unsigned velocityIterations, unsigned positionIterations, real velocityEpsilon = 0.01f,
                        real positionEpsilon = 0.01f);

        /**
        * Returns true if the resolver has valid settings and is ready to go.
        */
        bool IsValid() const;

        /**
        * Sets the number of iterations for both resolution stages.
        */
        void SetIterations(unsigned iterations);

        /**
        * Sets the number of iterations for each resolution stage.
        */
        void SetIterations(unsigned velocityIterations, unsigned positionIterations);

        /**
        * Sets the tolerance value for both velocity and position.
        */
        void SetEpsilon(real velocityEpsilon, real positionEpsilon);

        /**
        * Resolves a set of contacts for both penetration and velocity.
        *
        * Contacts that cannot interact with
        * each other should be passed to separate calls to resolveContacts,
        * as the resolution algorithm takes much longer for lots of
        * contacts than it does for the same number of contacts in small
        * sets.
        *
        * @param contacts Pointer to an array of contact objects.
        *
        * @param numContacts The number of contacts in the array to resolve.
        *
        * @param numContacts The number of iterations through the
        * resolution algorithm. This should be at least the number of
        * contacts (otherwise some constraints will not be resolved -
        * although sometimes this is not noticeable). If the iterations are
        * not needed they will not be used, so adding more iterations may
        * not make any difference. In some cases you would need millions
        * of iterations. Think about the number of iterations as a bound:
        * if you specify a large number, sometimes the algorithm WILL use
        * it, and you may drop lots of frames.
        *
        * @param deltaTime The duration of the previous integration step.
        * This is used to compensate for forces applied.
        */
        void ResolveContacts(Contact* contacts, unsigned numContacts, real deltaTime);

    protected:
        /**
        * Sets up contacts ready for processing. This makes sure their
        * internal data is configured correctly and the correct set of bodies
        * is made alive.
        */
        static void PrepareContacts(Contact* contacts, unsigned numContacts, real deltaTime);

        /**
        * Resolves the velocity issues with the given array of constraints,
        * using the given number of iterations.
        */
        void AdjustVelocities(Contact* contacts, unsigned numContacts, real deltaTime);

        /**
        * Resolves the positional issues with the given array of constraints,
        * using the given number of iterations.
        */
        void AdjustPositions(Contact* contacts, unsigned numContacts, real deltaTime);

    public:
        /**
        * Stores the number of velocity iterations used in the
        * last call to resolve contacts.
        */
        unsigned velocityIterationsUsed;

        /**
        * Stores the number of position iterations used in the
        * last call to resolve contacts.
        */
        unsigned positionIterationsUsed;

    protected:
        /**
        * Holds the number of iterations to perform when resolving
        * velocity.
        */
        unsigned velocityIterations;

        /**
        * Holds the number of iterations to perform when resolving
        * position.
        */
        unsigned positionIterations;

        /**
        * To avoid instability velocities smaller
        * than this value are considered to be zero. Too small and the
        * simulation may be unstable, too large and the bodies may
        * interpenetrate visually. A good starting point is the default
        * of 0.01.
        */
        real velocityEpsilon;

        /**
        * To avoid instability penetrations
        * smaller than this value are considered to be not interpenetrating.
        * Too small and the simulation may be unstable, too large and theo
        * bodies may interpenetrate visually. A good starting point is
        * the default of0.01.
        */
        real positionEpsilon;
    };
}