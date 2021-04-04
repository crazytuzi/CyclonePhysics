#pragma once

#include "Particle/Particle.h"

namespace cyclone
{
    /**
    * A Contact represents two objects in contact (in this case
    * ParticleContact representing two Particles). Resolving a
    * contact removes their interpenetration, and applies sufficient
    * impulse to keep them apart. Colliding bodies may also rebound.
    *
    * The contact has no callable functions, it just holds the
    * contact details. To resolve a set of contacts, use the particle
    * contact resolver class.
    */
    class ParticleContact
    {
    protected:
        /**
        * Resolves this contact, for both velocity and interpenetration.
        */
        void Resolve(real deltaTime);

        /**
        * Calculates the separating velocity at this contact.
        */
        real CalculateSeparatingVelocity() const;

    private:
        /**
        * Handles the impulse calculations for this collision.
        */
        void ResolveVelocity(real deltaTime);

        /**
        * Handles the interpenetration resolution for this contact.
        */
        void ResolveInterpenetration(real deltaTime);

    public:
        /**
        * Holds the particles that are involved in the contact. The
        * second of these can be NULL, for contacts with the scenery.
        */
        Particle* particle[2];

        /**
        * Holds the normal restitution coefficient at the contact.
        */
        real restitution;

        /**
        * Holds the direction of the contact in world coordinates.
        */
        Vector3 contactNormal;

        /**
        * Holds the depth of penetration at the contact.
        */
        real penetration;

        /**
        * Holds the amount each particle is moved by during interpenetration
        * resolution.
        */
        Vector3 particleMovement[2];

        friend class ParticleContactResolver;
    };
}
