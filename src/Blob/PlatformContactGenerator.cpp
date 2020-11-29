#include "PlatformContactGenerator.h"
#include <cmath>

unsigned PlatformContactGenerator::AddContact(cyclone::ParticleContact* contact, const unsigned limit) const
{
    const static cyclone::real restitution = 0.f;

    auto used = 0u;

    for (auto i = 0; i < BLOB_COUNT; ++i)
    {
        if (used >= limit)
        {
            break;
        }

        // Check for penetration
        auto toParticle = particles[i].GetPosition() - start;

        auto lineDirection = end - start;

        const auto projected = toParticle | lineDirection;

        const auto platformSquareLength = lineDirection.SizeSquared();

        if (projected <= 0)
        {
            // The blob is nearest to the start point
            if (toParticle.SizeSquared() < BLOB_RADIUS * BLOB_RADIUS)
            {
                // We have a collision
                contact->contactNormal = toParticle.Unit();

                contact->contactNormal.z = 0;

                contact->restitution = restitution;

                contact->particle[0] = particles + i;

                contact->particle[1] = nullptr;

                contact->penetration = BLOB_RADIUS - toParticle.Size();

                ++used;

                ++contact;
            }
        }
        else if (projected >= platformSquareLength)
        {
            // The blob is nearest to the end point
            toParticle = particles[i].GetPosition() - end;

            if (toParticle.SizeSquared() < BLOB_RADIUS * BLOB_RADIUS)
            {
                // We have a collision
                contact->contactNormal = toParticle.Unit();

                contact->contactNormal.z = 0;

                contact->restitution = restitution;

                contact->particle[0] = particles + i;

                contact->particle[1] = nullptr;

                contact->penetration = BLOB_RADIUS - toParticle.SizeSquared();

                ++used;

                ++contact;
            }
        }
        else
        {
            // the blob is nearest to the middle.
            const auto distanceToPlatform = toParticle.SizeSquared() - projected * projected / platformSquareLength;

            if (distanceToPlatform < BLOB_RADIUS * BLOB_RADIUS)
            {
                // We have a collision
                auto closestPoint = start + lineDirection * (projected / platformSquareLength);

                contact->contactNormal = (particles[i].GetPosition() - closestPoint).Unit();

                contact->contactNormal.z = 0;

                contact->restitution = restitution;

                contact->particle[0] = particles + i;

                contact->particle[1] = nullptr;

                contact->penetration = BLOB_RADIUS - real_sqrt(distanceToPlatform);

                ++used;

                ++contact;
            }
        }
    }

    return used;
}
