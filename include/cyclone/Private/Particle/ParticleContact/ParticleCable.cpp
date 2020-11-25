#include "Particle/ParticleContact/ParticleCable.h"

using namespace cyclone;

unsigned ParticleCable::AddContact(ParticleContact* contact, unsigned limit) const
{
    // Find the length of the cable
    const auto length = CurrentLength();

    // Check if we're over-extended
    if (length < maxLength)
    {
        return 0;
    }

    if (particle[0] == nullptr || particle[1] == nullptr)
    {
        return 0;
    }

    // Otherwise return the contact
    contact->particle[0] = particle[0];

    contact->particle[1] = particle[1];

    // Calculate the normal
    auto normal = particle[1]->GetPosition() - particle[0]->GetPosition();

    normal.Normalize();

    contact->contactNormal = normal;

    contact->penetration = length - maxLength;

    contact->restitution = restitution;

    return 1;
}
