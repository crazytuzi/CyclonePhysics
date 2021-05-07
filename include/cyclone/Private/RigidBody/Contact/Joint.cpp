#include "RigidBody/Contact/Joint.h"
#include <cmath>

using namespace cyclone;

void Joint::Set(RigidBody* a, const Vector3& a_pos, RigidBody* b, const Vector3& b_pos, const real error)
{
    body[0] = a;

    body[1] = b;

    position[0] = a_pos;

    position[1] = b_pos;

    Joint::error = error;
}

unsigned Joint::AddContact(Contact* contact, unsigned limit) const
{
    // Calculate the position of each connection point in world coordinates
    const auto a_pos_world = body[0]->GetPointInWorldSpace(position[0]);

    const auto b_pos_world = body[1]->GetPointInWorldSpace(position[1]);

    // Calculate the length of the joint
    const auto a_to_b = b_pos_world - a_pos_world;

    auto normal = a_to_b;

    normal.Normalize();

    const auto length = a_to_b.Size();

    // Check if it is violated
    if (real_abs(length) > error)
    {
        contact->body[0] = body[0];

        contact->body[1] = body[1];

        contact->contactNormal = normal;

        contact->contactPoint = (a_pos_world + b_pos_world) * 0.5f;

        contact->penetration = length - error;

        contact->friction = 1.f;

        contact->restitution = 0.f;

        return 1;
    }

    return 0;
}
