#include "Particle/ParticleContact/ParticleContact.h"

using namespace cyclone;

void ParticleContact::Resolve(const real deltaTime)
{
    ResolveVelocity(deltaTime);

    ResolveInterpenetration(deltaTime);
}

real ParticleContact::CalculateSeparatingVelocity() const
{
    if (particle[0] == nullptr)
    {
        return 0;
    }

    auto relativeVelocity = particle[0]->GetVelocity();

    if (particle[1] != nullptr)
    {
        relativeVelocity -= particle[1]->GetVelocity();
    }

    return relativeVelocity | contactNormal;
}

void ParticleContact::ResolveVelocity(const real deltaTime)
{
    if (particle[0] == nullptr)
    {
        return;
    }

    // Find the velocity in the direction of the contact
    const auto separatingVelocity = CalculateSeparatingVelocity();

    // Check if it needs to be resolved
    if (separatingVelocity > 0)
    {
        // The contact is either separating, or stationary - there's
        // no impulse required.
        return;
    }

    // Calculate the new separating velocity
    auto newSepVelocity = -separatingVelocity * restitution;

    // Check the velocity build-up due to acceleration only
    auto accelerationVelocity = particle[0]->GetAcceleration();

    if (particle[1] != nullptr)
    {
        accelerationVelocity -= particle[1]->GetAcceleration();
    }

    const auto accelerationSepVelocity = accelerationVelocity | contactNormal * deltaTime;

    // If we've got a closing velocity due to acceleration build-up,
    // remove it from the new separating velocity
    if (accelerationSepVelocity < 0)
    {
        newSepVelocity += restitution * accelerationSepVelocity;

        // Make sure we haven't removed more than was
        // there to remove.
        if (newSepVelocity < 0)
        {
            newSepVelocity = 0;
        }
    }

    const auto deltaVelocity = newSepVelocity - separatingVelocity;

    // We apply the change in velocity to each object in proportion to
    // their inverse mass (i.e. those with lower inverse mass [higher
    // actual mass] get less change in velocity)..
    auto totalInverseMass = particle[0]->GetInverseMass();

    if (particle[1] != nullptr)
    {
        totalInverseMass += particle[1]->GetInverseMass();
    }

    // If all particles have infinite mass, then impulses have no effect
    if (totalInverseMass <= 0)
    {
        return;
    }

    // Calculate the impulse to apply
    const auto impulse = deltaVelocity / totalInverseMass;

    // Find the amount of impulse per unit of inverse mass
    const auto impulsePerInverseMass = contactNormal * impulse;

    // Apply impulses: they are applied in the direction of the contact,
    // and are proportional to the inverse mass.
    particle[0]->SetVelocity(particle[0]->GetVelocity() + impulsePerInverseMass * particle[0]->GetInverseMass());

    if (particle[1] != nullptr)
    {
        // Particle 1 goes in the opposite direction
        particle[1]->SetVelocity(particle[1]->GetVelocity() + impulsePerInverseMass * -particle[1]->GetInverseMass());
    }
}

void ParticleContact::ResolveInterpenetration(const real deltaTime)
{
    // If we don't have any penetration, skip this step.
    if (penetration <= 0)
    {
        return;
    }

    if (particle[0] == nullptr)
    {
        return;
    }

    // The movement of each object is based on their inverse mass, so
    // total that.
    auto totalInverseMass = particle[0]->GetInverseMass();

    if (particle[1] != nullptr)
    {
        totalInverseMass += particle[1]->GetInverseMass();
    }

    // If all particles have infinite mass, then we do nothing
    if (totalInverseMass <= 0)
    {
        return;
    }

    // Find the amount of penetration resolution per unit of inverse mass
    const auto movePerInverseMass = contactNormal * (penetration / totalInverseMass);

    // Calculate the the movement amounts
    particleMovement[0] = movePerInverseMass * particle[0]->GetInverseMass();

    if (particle[1] != nullptr)
    {
        particleMovement[1] = movePerInverseMass * -particle[1]->GetInverseMass();
    }
    else
    {
        particleMovement[1].Reset();
    }

    // Apply the penetration resolution
    particle[0]->SetPosition(particle[0]->GetPosition() + particleMovement[0]);

    if (particle[1] != nullptr)
    {
        particle[1]->SetPosition(particle[1]->GetPosition() + particleMovement[1]);
    }
}
