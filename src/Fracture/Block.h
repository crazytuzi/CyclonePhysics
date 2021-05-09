#pragma once

#include "RigidBody/FineCollision/CollisionBox.h"

class Block : public cyclone::CollisionBox
{
public:
    Block();

    ~Block();

    /** Draws the block. */
    void Render() const;

    /** Sets the block to a specific location. */
    void SetState(const cyclone::Vector3& position, const cyclone::Quaternion& orientation,
                  const cyclone::Vector3& extents, const cyclone::Vector3& velocity);

    /**
    * Calculates and sets the mass and inertia tensor of this block,
    * assuming it has the given constant density.
    */
    void CalculateMassProperties(cyclone::real inverseDensity) const;

    /**
    * Performs the division of the given block into four, writing the
    * eight new blocks into the given blocks array. The blocks array can be
    * a pointer to the same location as the target pointer: since the
    * original block is always deleted, this effectively reuses its storage.
    * The algorithm is structured to allow this reuse.
    */
    void DivideBlock(const cyclone::Contact& contact, Block* target, Block* blocks) const;

public:
    bool exists;
};
