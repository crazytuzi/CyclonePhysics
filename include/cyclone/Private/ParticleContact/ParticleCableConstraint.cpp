#include "ParticleContact/ParticleCableConstraint.h"

using namespace cyclone;

unsigned ParticleCableConstraint::AddContact(ParticleContact* contact, unsigned limit) const
{
    // Find the length of the cable
    const auto length = CurrentLength();

    // Check if we're over-extended
    if (length < maxLength)
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

    contact->contactNormal = normal;

    contact->penetration = length - maxLength;

    contact->restitution = restitution;

    return 1;
}
