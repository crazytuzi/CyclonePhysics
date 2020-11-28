#include "MassAggregateApplication.h"
#include "Timing.h"
#include "gl/glut.h"

MassAggregateApplication::MassAggregateApplication(const unsigned particleCount): world(particleCount * 10),
    particles(new cyclone::Particle[particleCount])
{
    for (auto i = 0u; i < particleCount; ++i)
    {
        world.GetParticles().push_back(particles + i);
    }

    groundContactGenerator.Init(&world.GetParticles());

    world.GetContactGenerators().push_back(&groundContactGenerator);
}

MassAggregateApplication::~MassAggregateApplication()
{
    if (particles)
    {
        delete[] particles;

        particles = nullptr;
    }
}

void MassAggregateApplication::Update()
{
    // Clear accumulators
    world.StartFrame();

    // Find the duration of the last frame in seconds

    const auto duration = Timing::Get().lastFrameDuration * 0.001f;

    if (duration <= 0.f)
    {
        return;
    }

    // Run the simulation
    world.RunPhysics(duration);

    Application::Update();
}

void MassAggregateApplication::StartUp()
{
    // Call the superclass
    Application::StartUp();
}

void MassAggregateApplication::Display()
{
    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    gluLookAt(0.f, 3.5f, 8.f, 0.f, 3.5f, 0.f, 0.f, 1.f, 0.f);

    glColor3f(0, 0, 0);

    const auto& particles = world.GetParticles();

    for (const auto& particle : particles)
    {
        const auto& position = particle->GetPosition();

        glPushMatrix();

        glTranslatef(position.x, position.y, position.z);

        glutSolidSphere(0.1f, 20, 10);

        glPopMatrix();
    }
}
