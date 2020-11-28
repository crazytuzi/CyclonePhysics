#include "Particle/ParticleContact/GroundContacts.h"

using namespace cyclone;

GroundContacts::GroundContacts(): particles(nullptr)
{
}

void GroundContacts::Init(ParticleWorld::Particles* particles)
{
    GroundContacts::particles = particles;
}

unsigned GroundContacts::AddContact(ParticleContact* contact, const unsigned limit) const
{
    if (contact == nullptr || particles == nullptr)
    {
        return 0;
    }

    auto count = 0u;

    for (auto particle = particles->begin(); particle != particles->end(); ++particle)
    {
        const auto y = (*particle)->GetPosition().y;

        if (y < 0.f)
        {
            contact->contactNormal = Vector3::Up;

            contact->particle[0] = *particle;

            contact->particle[1] = nullptr;

            contact->penetration = -y;

            contact->restitution = 0.2f;

            ++contact;

            ++count;
        }

        if (count >= limit)
        {
            return count;
        }
    }

    return count;
}
