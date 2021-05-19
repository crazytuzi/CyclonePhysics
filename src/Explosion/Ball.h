#pragma once

#include "cyclone/RigidBody/FineCollision/CollisionSphere.h"

class Ball : public cyclone::CollisionSphere
{
public:
    Ball();

    ~Ball();

    /** Draws the box, excluding its shadow. */
    void Render() const;

    /** Draws the ground plane shadow for the box. */
    void RenderShadow() const;

    /** Sets the box to a specific location. */
    void SetState(const cyclone::Vector3& position, const cyclone::Quaternion& orientation, cyclone::real radius,
                  const cyclone::Vector3& velocity);

    /** Positions the box at a random location. */
    void Random(cyclone::Random* random);
};
