#include "ParticleContact/ParticleConstraint.h"

using namespace cyclone;

real ParticleConstraint::CurrentLength() const
{
    if (particle == nullptr)
    {
        return 0;
    }

    const auto relativePosition = particle->GetPosition() - anchor;

    return relativePosition.Size();
}
