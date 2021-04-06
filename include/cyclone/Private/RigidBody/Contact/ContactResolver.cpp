#include "RigidBody/Contact/ContactResolver.h"

using namespace cyclone;

ContactResolver::ContactResolver(const unsigned iterations, const real velocityEpsilon, const real positionEpsilon):
    velocityIterationsUsed(0), positionIterationsUsed(0), velocityIterations(iterations),
    positionIterations(iterations), velocityEpsilon(velocityEpsilon), positionEpsilon(positionEpsilon)
{
}

ContactResolver::ContactResolver(const unsigned velocityIterations, const unsigned positionIterations,
                                 const real velocityEpsilon, const real positionEpsilon): velocityIterationsUsed(0),
    positionIterationsUsed(0), velocityIterations(velocityIterations), positionIterations(positionIterations),
    velocityEpsilon(velocityEpsilon), positionEpsilon(positionEpsilon)
{
}

bool ContactResolver::IsValid() const
{
    return velocityIterations > 0 && positionIterationsUsed > 0 && velocityEpsilon >= 0.f && positionEpsilon >= 0.f;
}

void ContactResolver::SetIterations(const unsigned iterations)
{
    SetIterations(iterations, iterations);
}

void ContactResolver::SetIterations(const unsigned velocityIterations, const unsigned positionIterations)
{
    ContactResolver::velocityIterations = velocityIterations;

    ContactResolver::positionIterations = positionIterations;
}

void ContactResolver::SetEpsilon(const real velocityEpsilon, const real positionEpsilon)
{
    ContactResolver::velocityEpsilon = velocityEpsilon;

    ContactResolver::positionEpsilon = positionEpsilon;
}

void ContactResolver::ResolveContacts(Contact* contacts, const unsigned numContacts, const real deltaTime)
{
    // Make sure we have something to do.
    if (contacts == nullptr || numContacts == 0)
    {
        return;
    }

    if (!IsValid())
    {
        return;
    }

    // Prepare the contacts for processing
    PrepareContacts(contacts, numContacts, deltaTime);

    // Resolve the interpenetration problems with the contacts.
    AdjustPositions(contacts, numContacts, deltaTime);

    // Resolve the velocity problems with the contacts.
    AdjustVelocities(contacts, numContacts, deltaTime);
}

void ContactResolver::PrepareContacts(Contact* contacts, const unsigned numContacts, const real deltaTime)
{
    // Generate contact velocity and axis information.
    const auto LastContact = contacts + numContacts;

    for (auto contact = contacts; contact < LastContact; ++contact)
    {
        // Calculate the internal contact data (inertia, basis, etc).
        contact->CalculateInternals(deltaTime);
    }
}

void ContactResolver::AdjustVelocities(Contact* contacts, const unsigned numContacts, const real deltaTime)
{
    Vector3 velocityChange[2], rotationChange[2];

    // iteratively handle impacts in order of severity.
    velocityIterationsUsed = 0u;

    while (velocityIterationsUsed < velocityIterations)
    {
        // Find contact with maximum magnitude of probable velocity change.
        auto maxVelocity = velocityEpsilon;

        auto index = numContacts;

        for (auto i = 0u; i < numContacts; ++i)
        {
            if (contacts[i].desiredDeltaVelocity > maxVelocity)
            {
                maxVelocity = contacts[i].desiredDeltaVelocity;

                index = i;
            }
        }

        if (index == numContacts)
        {
            break;
        }

        // Match the awake state at the contact
        contacts[index].MatchAwakeState();

        // Do the resolution on the contact that came out top.
        contacts[index].ApplyVelocityChange(velocityChange, rotationChange);

        // With the change in velocity of the two bodies, the update of
        // contact velocities means that some of the relative closing
        // velocities need recomputing.
        for (auto i = 0u; i < numContacts; ++i)
        {
            // Check each body in the contact
            for (auto b = 0; b < 2; ++b)
            {
                if (contacts[i].body[b])
                {
                    // Check for a match with each body in the newly
                    // resolved contact
                    for (auto d = 0; d < 2; ++d)
                    {
                        if (contacts[i].body[b] == contacts[index].body[d])
                        {
                            auto deltaVelocity = velocityChange[d] + rotationChange[d] ^ contacts[i].
                                relativeContactPosition[b];

                            // The sign of the change is negative if we're dealing
                            // with the second body in a contact.
                            contacts[i].contactVelocity += contacts[i].contactToWorld.InverseTransformPosition(
                                deltaVelocity) * (b ? -1.f : 1.f);

                            contacts[i].CalculateDesiredDeltaVelocity(deltaTime);
                        }
                    }
                }
            }
        }

        ++velocityIterationsUsed;
    }
}

void ContactResolver::AdjustPositions(Contact* contacts, const unsigned numContacts, const real deltaTime)
{
    Vector3 linearChange[2], angularChange[2];

    // iteratively resolve interpenetration in order of severity.
    positionIterationsUsed = 0u;

    while (positionIterationsUsed < positionIterations)
    {
        // Find biggest penetration
        auto maxPenetration = positionEpsilon;

        auto index = numContacts;

        for (auto i = 0u; i < numContacts; ++i)
        {
            if (contacts[i].penetration > maxPenetration)
            {
                maxPenetration = contacts[i].penetration;

                index = i;
            }
        }

        if (index == numContacts)
        {
            return;
        }

        // Match the awake state at the contact
        contacts[index].MatchAwakeState();

        // Resolve the penetration.
        contacts[index].ApplyPositionChange(linearChange, angularChange, maxPenetration);

        // Again this action may have changed the penetration of other
        // bodies, so we update contacts.
        for (auto i = 0u; i < numContacts; ++i)
        {
            // Check each body in the contact
            for (auto b = 0; b < 2; ++b)
            {
                if (contacts[i].body[b])
                {
                    // Check for a match with each body in the newly
                    // resolved contact
                    for (auto d = 0; d < 2; ++d)
                    {
                        if (contacts[i].body[b] == contacts[index].body[d])
                        {
                            auto deltaPosition = linearChange[d] + angularChange[d] ^ contacts[i].
                                relativeContactPosition[b];

                            // The sign of the change is positive if we're
                            // dealing with the second body in a contact
                            // and negative otherwise (because we're
                            // subtracting the resolution)..
                            contacts[i].penetration += deltaPosition | contacts[i].contactNormal * (b ? 1.f : -1.f);
                        }
                    }
                }
            }
        }

        ++positionIterationsUsed;
    }
}
