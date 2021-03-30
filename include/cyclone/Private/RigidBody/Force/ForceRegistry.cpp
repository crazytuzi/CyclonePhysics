#include "RigidBody/Force/ForceRegistry.h"

using namespace cyclone;

void ForceRegistry::Add(RigidBody* body, ForceGenerator* forceGenerator)
{
    registrations.emplace_back(body, forceGenerator);
}

void ForceRegistry::Remove(RigidBody* body, ForceGenerator* forceGenerator)
{
    for (auto registry = registrations.begin(); registry != registrations.end(); ++registry)
    {
        if (registry->body == body && registry->forceGenerator == forceGenerator)
        {
            registrations.erase(registry);

            break;
        }
    }
}

void ForceRegistry::Clear()
{
    registrations.clear();
}

void ForceRegistry::UpdateForces(const real deltaTime)
{
    for (const auto& registry : registrations)
    {
        if (registry.forceGenerator != nullptr)
        {
            registry.forceGenerator->UpdateForce(registry.body, deltaTime);
        }
    }
}
