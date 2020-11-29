#pragma once

#include "Application.h"
#include "PlatformContactGenerator.h"
#include "BlobForceGenerator.h"
#include <cyclone/Particle/Particle.h>
#include <cyclone/Particle/ParticleWorld.h>

#ifndef BLOB_COUNT
#define BLOB_COUNT 5
#endif

#ifndef BLOB_RADIUS
#define BLOB_RADIUS 0.4f
#endif

#ifndef PLATFORM_COUNT
#define PLATFORM_COUNT 10
#endif

class BlobApplication : public Application
{
public:
    BlobApplication();

    ~BlobApplication();

    /** Returns the window title for the demo. */
    const char* GetTitle() override;

    /** Display the particles. */
    void Display() override;

    /** Update the particle positions. */
    void Update() override;

    /** Handle a key press. */
    void Key(unsigned char key) override;

private:
    void Reset() const;

private:
    cyclone::Particle* blobs;

    PlatformContactGenerator* platforms;

    cyclone::ParticleWorld world;

    BlobForceGenerator blobForceGenerator;

    /* The control for the x-axis. */
    cyclone::real xAxis;

    /* The control for the y-axis. */
    cyclone::real yAxis;
};
