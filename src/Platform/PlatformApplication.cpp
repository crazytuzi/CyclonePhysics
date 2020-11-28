#include "PlatformApplication.h"
#include "gl/glut.h"

/**
* Called by the common demo framework to create an application
* object (with new) and return a pointer.
*/
Application* GetApplication()
{
    return new PlatformApplication();
}

PlatformApplication::PlatformApplication(): MassAggregateApplication(6), rods(nullptr), massPosition(0, 0, 0.5f)
{
    particles[0].SetPosition(0.f, 0.f, 1.f);

    particles[1].SetPosition(0.f, 0.f, -1.f);

    particles[2].SetPosition(-3.f, 2.f, 1.f);

    particles[3].SetPosition(-3.f, 2.f, -1.f);

    particles[4].SetPosition(4.f, 2.f, 1.f);

    particles[5].SetPosition(4.f, 2.f, -1.f);

    for (auto i = 0; i < 6; ++i)
    {
        particles[i].SetMass(BASE_MASS);

        particles[i].SetVelocity(0, 0, 0);

        particles[i].SetDamping(0.9f);

        particles[i].SetAcceleration(cyclone::Vector3::Gravity);

        particles[i].ClearAccumulator();
    }

    rods = new cyclone::ParticleRod[ROD_COUNT];

    rods[0].particle[0] = &particles[0];

    rods[0].particle[1] = &particles[1];

    rods[0].length = 2;

    rods[1].particle[0] = &particles[2];

    rods[1].particle[1] = &particles[3];

    rods[1].length = 2;

    rods[2].particle[0] = &particles[4];

    rods[2].particle[1] = &particles[5];

    rods[2].length = 2;

    rods[3].particle[0] = &particles[2];

    rods[3].particle[1] = &particles[4];

    rods[3].length = 7;

    rods[4].particle[0] = &particles[3];

    rods[4].particle[1] = &particles[5];

    rods[4].length = 7;

    rods[5].particle[0] = &particles[0];

    rods[5].particle[1] = &particles[2];

    rods[5].length = 3.606f;

    rods[6].particle[0] = &particles[1];

    rods[6].particle[1] = &particles[3];

    rods[6].length = 3.606f;

    rods[7].particle[0] = &particles[0];

    rods[7].particle[1] = &particles[4];

    rods[7].length = 4.472f;

    rods[8].particle[0] = &particles[1];

    rods[8].particle[1] = &particles[5];

    rods[8].length = 4.472f;

    rods[9].particle[0] = &particles[0];

    rods[9].particle[1] = &particles[3];

    rods[9].length = 4.123f;

    rods[10].particle[0] = &particles[2];

    rods[10].particle[1] = &particles[5];

    rods[10].length = 7.28f;

    rods[11].particle[0] = &particles[4];

    rods[11].particle[1] = &particles[1];

    rods[11].length = 4.899f;

    rods[12].particle[0] = &particles[1];

    rods[12].particle[1] = &particles[2];

    rods[12].length = 4.123f;

    rods[13].particle[0] = &particles[3];

    rods[13].particle[1] = &particles[4];

    rods[13].length = 7.28f;

    rods[14].particle[0] = &particles[5];

    rods[14].particle[1] = &particles[0];

    rods[14].length = 4.899f;

    for (auto i = 0; i < ROD_COUNT; ++i)
    {
        world.GetContactGenerators().push_back(&rods[i]);
    }

    UpdateAdditionalMass();
}

PlatformApplication::~PlatformApplication()
{
    if (rods)
    {
        delete[] rods;

        rods = nullptr;
    }
}

const char* PlatformApplication::GetTitle()
{
    return "Platform";
}

void PlatformApplication::Display()
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

    glEnd();

    glColor3f(1, 0, 0);

    glPushMatrix();

    glTranslatef(massDisplayPosition.x, massDisplayPosition.y + 0.25f, massDisplayPosition.z);

    glutSolidSphere(0.25f, 20, 10);

    glPopMatrix();
}

void PlatformApplication::Update()
{
    MassAggregateApplication::Update();

    UpdateAdditionalMass();
}

void PlatformApplication::Key(const unsigned char key)
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

            if (massPosition.x > 1.f)
            {
                massPosition.x = 1.f;
            }
        }
        break;
    default:
        MassAggregateApplication::Key(key);
    }
}

void PlatformApplication::UpdateAdditionalMass()
{
    for (auto i = 2; i < 6; ++i)
    {
        particles[i].SetMass(BASE_MASS);
    }

    // Find the coordinates of the mass as an index and proportion
    auto xp = massPosition.x;

    if (xp < 0)
    {
        xp = 0;
    }

    if (xp > 1)
    {
        xp = 1;
    }

    auto zp = massPosition.z;

    if (zp < 0)
    {
        zp = 0;
    }

    if (zp > 1)
    {
        zp = 1;
    }

    // Calculate where to draw the mass

    massDisplayPosition.Reset();

    // Add the proportion to the correct masses
    particles[2].SetMass(BASE_MASS + EXTRA_MASS * (1 - xp) * (1 - zp));

    massDisplayPosition += particles[2].GetPosition() * (1 - xp) * (1 - zp);

    if (xp > 0)
    {
        particles[4].SetMass(BASE_MASS + EXTRA_MASS * xp * (1 - zp));

        massDisplayPosition += particles[4].GetPosition() * xp * (1 - zp);

        if (zp > 0)
        {
            particles[5].SetMass(BASE_MASS + EXTRA_MASS * xp * zp);

            massDisplayPosition += particles[5].GetPosition() * xp * zp;
        }
    }

    if (zp > 0)
    {
        particles[3].SetMass(BASE_MASS + EXTRA_MASS * (1 - xp) * zp);

        massDisplayPosition += particles[3].GetPosition() * (1 - xp) * zp;
    }
}
