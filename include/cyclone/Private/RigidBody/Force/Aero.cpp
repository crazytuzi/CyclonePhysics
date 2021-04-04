#include "RigidBody/Force/Aero.h"

using namespace cyclone;

Aero::Aero(const Matrix& tensor, const Vector3& position, const Vector3* windspeed): tensor(tensor), position(position),
    windspeed(windspeed)
{
}

void Aero::UpdateForce(RigidBody* body, const real deltaTime)
{
    UpdateForceFromTensor(body, deltaTime, tensor);
}

void Aero::UpdateForceFromTensor(RigidBody* body, const real deltaTime, const Matrix& tensor) const
{
    if (body != nullptr && windspeed != nullptr)
    {
        // Calculate total velocity (windspeed and body's velocity).
        auto velocity = body->GetVelocity();

        velocity += *windspeed;

        // Calculate the velocity in body coordinates
        const auto bodyVelocity = body->GetTransform().InverseTransformVector(velocity);

        // Calculate the force in body coordinates
        const auto bodyForce = tensor.TransformVector(bodyVelocity);

        const auto force = body->GetTransform().TransformVector(bodyForce);

        // Apply the force
        body->AddForceAtBodyPoint(force, position);
    }
}
