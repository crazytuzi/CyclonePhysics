#include "Fireworks.h"

bool Firework::Update(const cyclone::real deltaTime)
{
    // Update our physical state
    Integrate(deltaTime);

    // We work backwards from our age to zero.
    age -= deltaTime;

    return age < 0 || position.y < 0;
}
