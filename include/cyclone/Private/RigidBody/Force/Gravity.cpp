#include "RigidBody/Force/Gravity.h"

using namespace cyclone;

Gravity::Gravity(const Vector3& gravity): gravity(gravity)
{
}

void Gravity::UpdateForce(RigidBody* body, const real deltaTime)
{
    if (body != nullptr)
    {
        // Check that we do not have infinite mass
        if (!body->HasFiniteMass())
        {
            return;
        }

        // Apply the mass-scaled force to the body
        body->AddForce(gravity * body->GetMass());
    }
}
