#include "Particle/ParticleForce/ParticleForceRegistry.h"

using namespace cyclone;

void ParticleForceRegistry::Add(Particle* particle, ParticleForceGenerator* forceGenerator)
{
    registrations.emplace_back(particle, forceGenerator);
}

void ParticleForceRegistry::Remove(Particle* particle, ParticleForceGenerator* forceGenerator)
{
    for (auto registry = registrations.begin(); registry != registrations.end(); ++registry)
    {
        if (registry->particle == particle && registry->forceGenerator == forceGenerator)
        {
            registrations.erase(registry);

            break;
        }
    }
}

void ParticleForceRegistry::Clear()
{
    registrations.clear();
}

void ParticleForceRegistry::UpdateForces(const real deltaTime)
{
    for (const auto& registry : registrations)
    {
        if (registry.forceGenerator != nullptr)
        {
            registry.forceGenerator->UpdateForce(registry.particle, deltaTime);
        }
    }
}
