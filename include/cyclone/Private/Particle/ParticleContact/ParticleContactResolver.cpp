#include "Particle/ParticleContact/ParticleContactResolver.h"
#include <cfloat>

using namespace cyclone;

ParticleContactResolver::ParticleContactResolver(const unsigned iterations): iterations(iterations), iterationsUsed(0)
{
}

void ParticleContactResolver::SetIterations(const unsigned iterations)
{
    ParticleContactResolver::iterations = iterations;
}

void ParticleContactResolver::ResolveContacts(ParticleContact* contactArray, const unsigned numContacts,
                                              const real deltaTime)
{
    iterationsUsed = 0;

    while (iterationsUsed < iterations)
    {
        // Find the contact with the largest closing velocity;
        auto max = REAL_MAX;

        auto maxIndex = numContacts;

        for (auto i = 0u; i < numContacts; ++i)
        {
            const auto SeparatingVelocity = contactArray[i].CalculateSeparatingVelocity();

            if (SeparatingVelocity < max && (SeparatingVelocity < 0 || contactArray[i].penetration > 0))
            {
                max = SeparatingVelocity;

                maxIndex = i;
            }
        }

        // Do we have anything worth resolving?
        if (maxIndex == numContacts)
        {
            break;
        }

        // Resolve this contact
        contactArray[maxIndex].Resolve(deltaTime);

        // Update the inter penetrations for all particles
        const auto& move = contactArray[maxIndex].particleMovement;

        for (auto i = 0u; i < numContacts; ++i)
        {
            if (contactArray[i].particle[0] != nullptr)
            {
                if (contactArray[i].particle[0] == contactArray[maxIndex].particle[0])
                {
                    contactArray[i].penetration -= move[0] | contactArray[i].contactNormal;
                }
                else if (contactArray[i].particle[0] == contactArray[maxIndex].particle[1])
                {
                    contactArray[i].penetration -= move[1] | contactArray[i].contactNormal;
                }
            }

            if (contactArray[i].particle[1] != nullptr)
            {
                if (contactArray[i].particle[1] == contactArray[maxIndex].particle[0])
                {
                    contactArray[i].penetration += move[0] | contactArray[i].contactNormal;
                }
                else if (contactArray[i].particle[1] == contactArray[maxIndex].particle[1])
                {
                    contactArray[i].penetration += move[1] | contactArray[i].contactNormal;
                }
            }
        }

        ++iterationsUsed;
    }
}
