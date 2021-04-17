#include "RigidBody/FineCollision/CollisionPrimitive.h"

using namespace cyclone;

void CollisionPrimitive::CalculateInternals()
{
    if (body != nullptr)
    {
        transform = body->GetTransform() * offset;
    }
}

Vector3 CollisionPrimitive::GetAxis(const unsigned index) const
{
    return Vector3(transform.M[0][index], transform.M[1][index], transform.M[2][index]);
}

const Matrix& CollisionPrimitive::GetTransform() const
{
    return transform;
}
