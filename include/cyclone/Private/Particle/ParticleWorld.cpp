#include "Particle/ParticleWorld.h"

using namespace cyclone;

ParticleWorld::ParticleWorld(const unsigned maxContacts, const unsigned iterations):
    bCalculateIterations(iterations == 0),
    resolver(iterations),
    contacts(new ParticleContact[maxContacts]),
    maxContacts(maxContacts)
{
}

ParticleWorld::~ParticleWorld()
{
    if (contacts)
    {
        delete[] contacts;

        contacts = nullptr;
    }
}

unsigned ParticleWorld::GenerateContacts()
{
    auto limit = maxContacts;

    auto nextContact = contacts;

    for (const auto& contactGenerator : contactGenerators)
    {
        const auto used = contactGenerator->AddContact(nextContact, limit);

        limit -= used;

        nextContact += used;

        // We've run out of contacts to fill. This means we're missing
        // contacts.
        if (limit <= 0)
        {
            break;
        }
    }

    // Return the number of contacts used.
    return maxContacts - limit;
}

void ParticleWorld::Integrate(const real deltaTime)
{
    for (auto& particle : particles)
    {
        // Remove all forces from the accumulator
        particle->Integrate(deltaTime);
    }
}

void ParticleWorld::RunPhysics(const real deltaTime)
{
    // First apply the force generators
    registry.UpdateForces(deltaTime);

    // Then integrate the objects
    Integrate(deltaTime);

    // Generate contacts
    const auto usedContacts = GenerateContacts();

    // And process them
    if (usedContacts)
    {
        if (bCalculateIterations)
        {
            resolver.SetIterations(usedContacts * 2);
        }

        resolver.ResolveContacts(contacts, usedContacts, deltaTime);
    }
}

void ParticleWorld::StartFrame()
{
    for (auto& particle : particles)
    {
        // Remove all forces from the accumulator
        particle->ClearAccumulator();
    }
}

ParticleWorld::Particles& ParticleWorld::GetParticles()
{
    return particles;
}

ParticleWorld::ContactGenerators& ParticleWorld::GetContactGenerators()
{
    return contactGenerators;
}

ParticleForceRegistry& ParticleWorld::GetForceRegistry()
{
    return registry;
}
