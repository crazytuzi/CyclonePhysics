#include "RigidBody/Force/Spring.h"
#include <cmath>

using namespace cyclone;

Spring::Spring(const Vector3& localConnectionPoint, RigidBody* other, const Vector3& otherConnectionPoint,
               const real springConstant, const real restLength): connectionPoint(localConnectionPoint),
                                                                  otherConnectionPoint(otherConnectionPoint),
                                                                  other(other), springConstant(springConstant),
                                                                  restLength(restLength)
{
}

void Spring::UpdateForce(RigidBody* body, const real deltaTime)
{
    if (body != nullptr && other != nullptr)
    {
        // Calculate the two ends in world space
        const auto localWorldSpace = body->GetPointInWorldSpace(connectionPoint);

        const auto otherWorldSpace = other->GetPointInWorldSpace(otherConnectionPoint);

        // Calculate the vector of the spring
        auto force = localWorldSpace - otherWorldSpace;

        // Calculate the magnitude of the force
        auto magnitude = force.Size();

        magnitude = real_abs(magnitude - restLength);

        magnitude *= springConstant;

        // Calculate the final force and apply it
        force.Normalize();

        force *= -magnitude;

        body->AddForceAtPoint(force, localWorldSpace);
    }
}
