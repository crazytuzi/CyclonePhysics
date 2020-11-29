#include "BlobForceGenerator.h"

void BlobForceGenerator::UpdateForce(cyclone::Particle* particle, cyclone::real duration)
{
    auto joinCount = 0u;

    for (auto i = 0; i < BLOB_COUNT; ++i)
    {
        // Don't attract yourself
        if (particles + i == particle)
        {
            continue;
        }

        // Work out the separation distance
        auto separation = particles[i].GetPosition() - particle->GetPosition();

        separation.z = 0.f;

        auto distance = separation.Size();

        if (distance < minNaturalDistance)
        {
            // Use a repulsion force.
            distance = 1.f - distance / minNaturalDistance;

            particle->AddForce(separation.Unit() * (1.f - distance) * maxRepulsion * -1.f);

            ++joinCount;
        }
        else if (distance > maxNaturalDistance && distance < maxDistance)
        {
            // Use an attraction force.
            distance = (distance - maxNaturalDistance) / (maxDistance - maxNaturalDistance);

            particle->AddForce(separation.Unit() * distance * maxAttraction);

            ++joinCount;
        }
    }

    // If the particle is the head, and we've got a join count, then float it.
    if (particle == particles && joinCount > 0 && maxFloat > 0)
    {
        auto force = 1.f * joinCount / maxFloat * floatHead;

        if (force > floatHead)
        {
            force = floatHead;
        }

        particle->AddForce(cyclone::Vector3(0, force, 0));
    }
}
