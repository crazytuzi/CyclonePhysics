#pragma once

#include "cyclone/Core.h"
#include "Fireworks.h"

/**
 * Firework rules control the length of a firework's fuse and the
 * particles it should evolve into.
 */
class FireworkRule
{
    /**
    * The payload is the new firework type to create when this
    * firework's fuse is over.
    */
    struct Payload
    {
        /** The type of the new particle to create. */
        unsigned type;

        /** The number of particles in this payload. */
        unsigned count;

        /** Sets the payload properties in one go. */
        void Set(const unsigned type, const unsigned count)
        {
            Payload::type = type;

            Payload::count = count;
        }
    };

public:
    FireworkRule();

    ~FireworkRule();

    void Init(unsigned payloadCount);

    /**
    * Set all the rule parameters in one go.
    */
    void SetParameters(unsigned type, cyclone::real minAge, cyclone::real maxAge,
                       const cyclone::Vector3& minVelocity, const cyclone::Vector3& maxVelocity,
                       cyclone::real damping);

    /**
    * Creates a new firework of this type and writes it into the given
    * instance. The optional parent firework is used to base position
    * and velocity on.
    */
    void Create(Firework* firework, const Firework* parent = nullptr) const;

public:
    /** The type of firework that is managed by this rule. */
    unsigned type;

    /** The minimum length of the fuse. */
    cyclone::real minAge;

    /** The maximum length of the fuse. */
    cyclone::real maxAge;

    /** The minimum relative velocity of this firework. */
    cyclone::Vector3 minVelocity;

    /** The maximum relative velocity of this firework. */
    cyclone::Vector3 maxVelocity;

    /** The damping of this firework type. */
    cyclone::real damping;

    /** The number of payloads for this firework type. */
    unsigned payloadCount;

    /** The set of payloads. */
    Payload* payloads;

    friend class FireworksApplication;
};
