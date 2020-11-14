#include "Fireworks.h"

bool Firework::Update(const cyclone::real duration)
{
    // Update our physical state
    Integrate(duration);

    // We work backwards from our age to zero.
    age -= duration;

    return age < 0 || position.y < 0;
}
