#include "Particle/ParticleForce/ParticleDrag.h"

using namespace cyclone;

ParticleDrag::ParticleDrag(const real k1, const real k2): k1(k1), k2(k2)
{
}

void ParticleDrag::UpdateForce(Particle* particle, real duration)
{
    if (particle != nullptr)
    {
        Vector3 force;

        particle->GetVelocity(&force);

        // Calculate the total drag coefficient
        auto DragCoefficient = force.Size();

        DragCoefficient = k1 * DragCoefficient + k2 * DragCoefficient * DragCoefficient;

        // Calculate the final force and apply it
        force.Normalize();

        force *= -DragCoefficient;

        particle->AddForce(force);
    }
}
