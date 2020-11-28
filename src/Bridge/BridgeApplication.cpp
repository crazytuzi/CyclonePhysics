#include "BridgeApplication.h"
#include "gl/glut.h"
#include <cmath>

/**
* Called by the common demo framework to create an application
* object (with new) and return a pointer.
*/
Application* GetApplication()
{
    return new BridgeApplication();
}

BridgeApplication::BridgeApplication(): MassAggregateApplication(SUPPORT_COUNT), supports(nullptr), cables(nullptr),
                                        rods(nullptr), massPosition(0, 0, 0.5f)
{
    // Create the masses and connections.
    for (auto i = 0; i < SUPPORT_COUNT; ++i)
    {
        particles[i].SetPosition(
            i * 2 / 2.f - 5.f,
            4.f,
            i % 2 * 2.f - 1.f
        );

        particles[i].SetVelocity(0.f, 0.f, 0.f);

        particles->SetDamping(0.9f);

        particles[i].SetAcceleration(cyclone::Vector3::Gravity);

        particles[i].ClearAccumulator();
    }

    // Add the links
    cables = new cyclone::ParticleCable[CABLE_COUNT];

    for (auto i = 0; i < CABLE_COUNT; ++i)
    {
        cables[i].particle[0] = &particles[i];

        cables[i].particle[1] = &particles[i + 2];

        cables[i].maxLength = 1.9f;

        cables[i].restitution = 0.3f;

        world.GetContactGenerators().push_back(&cables[i]);
    }

    supports = new cyclone::ParticleCableConstraint[SUPPORT_COUNT];

    for (auto i = 0; i < SUPPORT_COUNT; ++i)
    {
        supports[i].particle = particles + i;

        supports[i].anchor = cyclone::Vector3(
            i / 2.f * 2.2f - 5.5f,
            6.f,
            i % 2 * 1.6f - 0.8f
        );

        if (i < 6)
        {
            supports[i].maxLength = i / 2.f * 0.5f + 3.f;
        }
        else
        {
            supports[i].maxLength = 5.5f - i / 2.f * 0.5f;
        }

        supports[i].restitution = 0.5f;

        world.GetContactGenerators().push_back(&supports[i]);
    }

    rods = new cyclone::ParticleRod[ROD_COUNT];

    for (auto i = 0; i < ROD_COUNT; ++i)
    {
        rods[i].particle[0] = &particles[i * 2];

        rods[i].particle[1] = &particles[i * 2 + 1];

        rods[i].length = 2;

        world.GetContactGenerators().push_back(&rods[i]);
    }

    UpdateAdditionalMass();
}

BridgeApplication::~BridgeApplication()
{
    if (cables)
    {
        delete[] cables;

        cables = nullptr;
    }

    if (rods)
    {
        delete[] rods;

        rods = nullptr;
    }

    if (supports)
    {
        delete[] supports;

        supports = nullptr;
    }
}

const char* BridgeApplication::GetTitle()
{
    return "Bridge";
}

void BridgeApplication::Display()
{
    MassAggregateApplication::Display();

    glBegin(GL_LINES);

    glColor3f(0, 0, 1);

    for (auto i = 0; i < ROD_COUNT; ++i)
    {
        const auto particles = rods[i].particle;

        const auto& position0 = particles[0]->GetPosition();

        const auto& position1 = particles[1]->GetPosition();

        glVertex3f(position0.x, position0.y, position0.z);

        glVertex3f(position1.x, position1.y, position1.z);
    }

    glColor3f(0, 1, 0);

    for (auto i = 0; i < CABLE_COUNT; ++i)
    {
        const auto particles = cables[i].particle;

        const auto& position0 = particles[0]->GetPosition();

        const auto& position1 = particles[1]->GetPosition();

        glVertex3f(position0.x, position0.y, position0.z);

        glVertex3f(position1.x, position1.y, position1.z);
    }

    glColor3f(0.7f, 0.7f, 0.7f);

    for (auto i = 0; i < SUPPORT_COUNT; ++i)
    {
        const auto& position0 = supports[i].particle->GetPosition();

        const auto& position1 = supports[i].anchor;

        glVertex3f(position0.x, position0.y, position0.z);

        glVertex3f(position1.x, position1.y, position1.z);
    }

    glEnd();

    glColor3f(1, 0, 0);

    glPushMatrix();

    glTranslatef(massDisplayPosition.x, massDisplayPosition.y + 0.25f, massDisplayPosition.z);

    glutSolidSphere(0.25f, 20, 10);

    glPopMatrix();
}

void BridgeApplication::Update()
{
    MassAggregateApplication::Update();

    UpdateAdditionalMass();
}

void BridgeApplication::Key(const unsigned char key)
{
    switch (key)
    {
    case 's':
    case 'S':
        {
            massPosition.z += 0.1f;

            if (massPosition.z > 1.f)
            {
                massPosition.z = 1.f;
            }
        }
        break;
    case 'w':
    case 'W':
        {
            massPosition.z -= 0.1f;

            if (massPosition.z < 0.f)
            {
                massPosition.z = 0.f;
            }
        }
        break;
    case 'a':
    case 'A':
        {
            massPosition.x -= 0.1f;

            if (massPosition.x < 0.f)
            {
                massPosition.x = 0.f;
            }
        }
        break;
    case 'd':
    case 'D':
        {
            massPosition.x += 0.1f;

            if (massPosition.x > 5.f)
            {
                massPosition.x = 5.f;
            }
        }
        break;
    default:
        MassAggregateApplication::Key(key);
    }
}

void BridgeApplication::UpdateAdditionalMass()
{
    for (auto i = 0; i < SUPPORT_COUNT; ++i)
    {
        particles[i].SetMass(BASE_MASS);
    }

    // Find the coordinates of the mass as an index and proportion
    auto x = static_cast<int>(massPosition.x);

    auto xp = real_fmod(massPosition.x, 1.f);

    if (x < 0)
    {
        x = 0;

        xp = 0;
    }

    if (x >= 5)
    {
        x = 5;

        xp = 0;
    }

    auto z = static_cast<int>(massPosition.z);

    auto zp = real_fmod(massPosition.z, 1.f);

    if (z < 0)
    {
        z = 0;

        zp = 0;
    }

    if (z >= 1)
    {
        z = 1;

        zp = 0;
    }

    // Calculate where to draw the mass
    massDisplayPosition.Reset();

    // Add the proportion to the correct masses
    particles[x * 2 + z].SetMass(BASE_MASS + EXTRA_MASS * (1 - xp) * (1 - zp));

    massDisplayPosition += particles[x * 2 + z].GetPosition() * (1 - xp) * (1 - zp);

    if (xp > 0)
    {
        particles[x * 2 + z + 2].SetMass(BASE_MASS + EXTRA_MASS * xp * (1 - zp));

        massDisplayPosition += particles[x * 2 + z + 2].GetPosition() * xp * (1 - zp);

        if (zp > 0)
        {
            particles[x * 2 + z + 3].SetMass(BASE_MASS + EXTRA_MASS * xp * zp);

            massDisplayPosition += particles[x * 2 + z + 3].GetPosition() * xp * zp;
        }
    }

    if (zp > 0)
    {
        particles[x * 2 + z + 1].SetMass(BASE_MASS + EXTRA_MASS * (1 - xp) * zp);

        massDisplayPosition += particles[x * 2 + z + 1].GetPosition() * (1 - xp) * zp;
    }
}
