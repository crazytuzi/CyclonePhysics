#include "RigidBody/Force/Buoyancy.h"

using namespace cyclone;

Buoyancy::Buoyancy(const Vector3& centreOfBuoyancy, const real maxDepth, const real volume, const real waterHeight,
                   const real liquidDensity): maxDepth(maxDepth), volume(volume), waterHeight(waterHeight),
                                              liquidDensity(liquidDensity), centreOfBuoyancy(centreOfBuoyancy)
{
}

void Buoyancy::UpdateForce(RigidBody* body, const real deltaTime)
{
    if (body != nullptr)
    {
        // Calculate the submersion depth
        const auto pointInWorld = body->GetPointInWorldSpace(centreOfBuoyancy);

        const auto depth = pointInWorld.y;

        // Check if we're out of the water
        if (depth >= waterHeight + maxDepth)
        {
            return;
        }

        Vector3 force(0.f, 0.f, 0.f);

        // Check if we're at maximum depth
        if (depth <= waterHeight - maxDepth)
        {
            force.y = liquidDensity * volume;

            body->AddForceAtBodyPoint(force, centreOfBuoyancy);

            return;
        }

        // Otherwise we are partly submerged
        force.y = liquidDensity * volume * (depth - maxDepth - waterHeight) / 2 * maxDepth;

        body->AddForceAtBodyPoint(force, centreOfBuoyancy);
    }
}
