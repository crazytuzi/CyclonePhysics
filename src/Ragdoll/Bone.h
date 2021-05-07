#pragma once

#include "RigidBody/FineCollision/CollisionBox.h"
#include "RigidBody/FineCollision/CollisionSphere.h"

class Bone : public cyclone::CollisionBox
{
public:
    Bone();

    ~Bone();

    /**
    * We use a sphere to collide bone on bone to allow some limited
    * interpenetration.
    */
    cyclone::CollisionSphere GetCollisionSphere() const;

    /** Draws the bone. */
    void Render() const;

    /** Sets the bone to a specific location. */
    void SetState(const cyclone::Vector3& position, const cyclone::Vector3& extents);
};
