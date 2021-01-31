#include "BlobApplication.h"
#include "Timing.h"
#include "gl/glut.h"
#include <cmath>
#include <cyclone/Core/Random.h>

/**
* Called by the common demo framework to create an application
* object (with new) and return a pointer.
*/
Application* GetApplication()
{
    return new BlobApplication();
}

BlobApplication::BlobApplication(): world(PLATFORM_COUNT + BLOB_COUNT,PLATFORM_COUNT), xAxis(0), yAxis(0)
{
    cyclone::Random r;

    // Create the blob storage
    blobs = new cyclone::Particle[BLOB_COUNT];

    // Create the force generator
    blobForceGenerator.particles = blobs;

    blobForceGenerator.maxAttraction = 20.f;

    blobForceGenerator.maxRepulsion = 10.f;

    blobForceGenerator.minNaturalDistance = BLOB_RADIUS * 0.75f;

    blobForceGenerator.maxNaturalDistance = BLOB_RADIUS * 1.5f;

    blobForceGenerator.maxDistance = BLOB_RADIUS * 2.5f;

    blobForceGenerator.maxFloat = 2;

    blobForceGenerator.floatHead = 8.f;

    // Create the platforms
    platforms = new PlatformContactGenerator[PLATFORM_COUNT];

    for (auto i = 0; i < PLATFORM_COUNT; ++i)
    {
        platforms[i].start = cyclone::Vector3(
            i % 2 * 10.f - 5.f,
            i * 4.f + (i % 2 ? 0.f : 2.f),
            0.f
        );

        platforms[i].start.x += r.RandomBinomial(2.f);

        platforms[i].start.y += r.RandomBinomial(2.f);

        platforms[i].end = cyclone::Vector3(
            i % 2 * 10.f + 5.f,
            i * 4.f + (i % 2 ? 2.f : 0.f),
            0.f
        );

        platforms[i].end.x += r.RandomBinomial(2.f);

        platforms[i].end.y += r.RandomBinomial(2.f);

        // Make sure the platform knows which particles it
        // should collide with.
        platforms[i].particles = blobs;

        world.GetContactGenerators().push_back(platforms + i);
    }

    // Create the blobs.
    const auto p = platforms + (PLATFORM_COUNT - 2);

    const auto fraction = 1.f / BLOB_COUNT;

    const auto delta = p->end - p->start;

    for (auto i = 0; i < BLOB_COUNT; ++i)
    {
        const auto me = (i + BLOB_COUNT / 2) % BLOB_COUNT;

        blobs[i].SetPosition(
            p->start + delta * (me * 0.8f * fraction + 0.1f) + cyclone::Vector3(0, 1.f + r.RandomReal(), 0));

        blobs[i].SetVelocity(0, 0, 0);

        blobs[i].SetDamping(0.2f);

        blobs[i].SetAcceleration(cyclone::Vector3::Gravity * 0.4f);

        blobs[i].SetMass(1.f);

        blobs[i].ClearAccumulator();

        world.GetParticles().push_back(blobs + i);

        world.GetForceRegistry().Add(blobs + i, &blobForceGenerator);
    }
}

BlobApplication::~BlobApplication()
{
    if (blobs)
    {
        delete[] blobs;

        blobs = nullptr;
    }

    if(platforms)
    {
        delete[] platforms;

        platforms = nullptr;
    }
}

const char* BlobApplication::GetTitle()
{
    return "Blob";
}

void BlobApplication::Display()
{
    const auto position = blobs[0].GetPosition();

    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    gluLookAt(position.x, position.y, 6.f, position.x, position.y, 0.f, 0.f, 1.f, 0.f);

    glColor3f(0, 0, 0);

    glBegin(GL_LINES);

    glColor3f(0, 0, 1);

    for (auto i = 0; i < PLATFORM_COUNT; ++i)
    {
        const auto& position0 = platforms[i].start;

        const auto& position1 = platforms[i].end;

        glVertex3f(position0.x, position0.y, position0.z);

        glVertex3f(position1.x, position1.y, position1.z);
    }

    glEnd();

    glColor3f(1, 0, 0);

    for (auto i = 0; i < BLOB_COUNT; ++i)
    {
        const auto& blobPosition = blobs[i].GetPosition();

        glPushMatrix();

        glTranslatef(blobPosition.x, blobPosition.y, blobPosition.z);

        glutSolidSphere(BLOB_RADIUS, 12, 12);

        glPopMatrix();
    }

    auto p = blobs[0].GetPosition();

    auto v = blobs[0].GetVelocity() * 0.05f;

    v.Trim(BLOB_RADIUS * 0.5f);

    p = p + v;

    glPushMatrix();

    glTranslatef(p.x - BLOB_RADIUS * 0.2f, p.y, BLOB_RADIUS);

    glColor3f(1, 1, 1);

    glutSolidSphere(BLOB_RADIUS * 0.2f, 8, 8);

    glTranslatef(0, 0,BLOB_RADIUS * 0.2f);

    glColor3f(0, 0, 0);

    glutSolidSphere(BLOB_RADIUS * 0.1f, 8, 8);

    glTranslatef(BLOB_RADIUS * 0.4f, 0, -BLOB_RADIUS * 0.2f);

    glColor3f(1, 1, 1);

    glutSolidSphere(BLOB_RADIUS * 0.2f, 8, 8);

    glTranslatef(0, 0,BLOB_RADIUS * 0.2f);

    glColor3f(0, 0, 0);

    glutSolidSphere(BLOB_RADIUS * 0.1f, 8, 8);

    glPopMatrix();
}

void BlobApplication::Update()
{
    // Clear accumulators
    world.StartFrame();

    // Find the duration of the last frame in seconds
    const auto duration = Timing::Get().lastFrameDuration * 0.001f;

    if (duration <= 0.f)
    {
        return;
    }

    // Recenter the axes
    xAxis *= real_pow(0.1f, duration);

    yAxis *= real_pow(0.1f, duration);

    // Move the controlled blob
    blobs[0].AddForce(cyclone::Vector3(xAxis, yAxis, 0) * 10.f);

    // Run the simulation
    world.RunPhysics(duration);

    // Bring all the particles back to 2d
    cyclone::Vector3 bPosition;

    for (auto i = 0; i < BLOB_COUNT; ++i)
    {
        blobs[i].GetPosition(&bPosition);

        bPosition.z = 0.f;

        blobs[i].SetPosition(bPosition);
    }

    Application::Update();
}

void BlobApplication::Key(const unsigned char key)
{
    switch (key)
    {
    case 'w':
    case 'W':
        {
            yAxis = 1.f;
        }
        break;
    case 's':
    case 'S':
        {
            yAxis = -1.f;
        }
        break;
    case 'a':
    case 'A':
        {
            xAxis = -1.f;
        }
        break;
    case 'd':
    case 'D':
        {
            xAxis = 1.f;
        }
        break;
    case 'r':
    case 'R':
        {
            Reset();
        }
        break;
    default:
        Application::Key(key);
    }
}

void BlobApplication::Reset() const
{
    cyclone::Random r;

    const auto p = platforms + (PLATFORM_COUNT - 2);

    const auto fraction = 1.f / BLOB_COUNT;

    const auto delta = p->end - p->start;

    for (auto i = 0; i < BLOB_COUNT; ++i)
    {
        const auto me = (i + BLOB_COUNT / 2) % BLOB_COUNT;

        blobs[i].SetPosition(
            p->start + delta * (me * 0.8f * fraction + 0.1f) + cyclone::Vector3(0, 1.f + r.RandomReal(), 0));

        blobs[i].SetVelocity(0, 0, 0);

        blobs[i].ClearAccumulator();
    }
}
