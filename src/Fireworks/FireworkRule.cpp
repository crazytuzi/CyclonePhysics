#include "FireworkRule.h"

static cyclone::Random Random;

FireworkRule::FireworkRule(): type(0), minAge(0), maxAge(0), minVelocity(0), maxVelocity(0), damping(0),
                              payloadCount(0), payloads(nullptr)
{
}

FireworkRule::~FireworkRule()
{
    if (payloads != nullptr)
    {
        delete payloads;

        payloads = nullptr;
    }
}

void FireworkRule::Init(const unsigned payloadCount)
{
    FireworkRule::payloadCount = payloadCount;

    payloads = new Payload[payloadCount];
}

void FireworkRule::SetParameters(const unsigned type, const cyclone::real minAge, const cyclone::real maxAge,
                                 const cyclone::Vector3& minVelocity, const cyclone::Vector3& maxVelocity,
                                 const cyclone::real damping)
{
    FireworkRule::type = type;

    FireworkRule::minAge = minAge;

    FireworkRule::maxAge = maxAge;

    FireworkRule::minVelocity = minVelocity;

    FireworkRule::maxVelocity = maxVelocity;

    FireworkRule::damping = damping;
}

void FireworkRule::Create(Firework* firework, const Firework* parent) const
{
    if (firework != nullptr)
    {
        firework->type = type;

        firework->age = Random.RandomReal(minAge, maxAge);

        cyclone::Vector3 velocity;

        if (parent != nullptr)
        {
            // The position and velocity are based on the parent.
            firework->SetPosition(parent->GetPosition());

            velocity += parent->GetVelocity();
        }
        else
        {
            cyclone::Vector3 start;

            const auto x = static_cast<int>(Random.RandomInt(2));

            start.x = 5.f * static_cast<cyclone::real>(x);

            firework->SetPosition(start);
        }

        velocity += Random.RandomVector(minVelocity, maxVelocity);

        firework->SetVelocity(velocity);

        // We use a mass of one in all cases (no point having fireworks
        // with different masses, since they are only under the influence
        // of gravity).
        firework->SetMass(1.f);

        firework->SetDamping(damping);

        firework->SetAcceleration(cyclone::Vector3::Gravity);

        firework->ClearAccumulator();
    }
}
