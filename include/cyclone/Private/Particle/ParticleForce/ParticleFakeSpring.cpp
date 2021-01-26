#include "Particle/ParticleForce/ParticleFakeSpring.h"
#include <cmath>

using namespace cyclone;

ParticleFakeSpring::ParticleFakeSpring(Vector3* anchor, const real springConstant, const real damping): anchor(anchor),
    springConstant(springConstant), damping(damping)
{
}

void ParticleFakeSpring::UpdateForce(Particle* particle, const real deltaTime)
{
    if (particle != nullptr && anchor != nullptr)
    {
        // Check that we do not have infinite mass
        if (!particle->HasFiniteMass())
        {
            return;
        }

        // Calculate the relative position of the particle to the anchor
        Vector3 position;

        particle->GetPosition(&position);

        position -= *anchor;

        // Calculate the constants and check they are in bounds.
        const auto gamma = 0.5f * real_sqrt(4 * springConstant - damping * damping);

        if (gamma == 0.f)
        {
            return;
        }

        const auto c = position * (damping / (2.f * gamma)) + particle->GetVelocity() * (1.f / gamma);

        // Calculate the target position
        auto target = position * real_cos(gamma * deltaTime) + c * real_sin(gamma * deltaTime);

        target *= real_exp(-0.5f * deltaTime * damping);

        // Calculate the resulting acceleration and therefore the force
        const auto acceleration = (target - position) * (static_cast<real>(1.f) / (deltaTime * deltaTime))
            - particle->GetVelocity();

        particle->AddForce(acceleration * particle->GetMass());
    }
}
