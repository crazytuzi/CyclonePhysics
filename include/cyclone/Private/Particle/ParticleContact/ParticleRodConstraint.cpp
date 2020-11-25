#include "Particle/ParticleContact/ParticleRodConstraint.h"

using namespace cyclone;

unsigned ParticleRodConstraint::AddContact(ParticleContact* contact, unsigned limit) const
{
    // Find the length of the rod
    const auto currentLength = CurrentLength();

    // Check if we're over-extended
    if (currentLength == length)
    {
        return 0;
    }

    if (particle == nullptr)
    {
        return 0;
    }

    // Otherwise return the contact
    contact->particle[0] = particle;

    contact->particle[1] = nullptr;

    // Calculate the normal
    auto normal = anchor - particle->GetPosition();

    normal.Normalize();

    // The contact normal depends on whether we're extending or compressing
    if (currentLength > length)
    {
        contact->contactNormal = normal;

        contact->penetration = currentLength - length;
    }
    else
    {
        contact->contactNormal = normal * -1;

        contact->penetration = length - currentLength;
    }

    // Always use zero restitution (no bounciness)
    contact->restitution = 0;

    return 1;
}
