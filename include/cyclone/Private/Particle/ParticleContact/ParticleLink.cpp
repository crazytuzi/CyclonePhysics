#include "Particle/ParticleContact/ParticleLink.h"

using namespace cyclone;

real ParticleLink::CurrentLength() const
{
    if (particle[0] == nullptr || particle[1] == nullptr)
    {
        return 0;
    }

    const auto relativePosition = particle[0]->GetPosition() - particle[1]->GetPosition();

    return relativePosition.Size();
}
