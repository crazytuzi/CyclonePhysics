#include "SailboatApplication.h"
#include "Timing.h"
#include "gl/glut.h"

/**
* Called by the common demo framework to create an application
* object (with new) and return a pointer.
*/
Application* GetApplication()
{
    return new SailboatApplication();
}

static void DrawBoat()
{
    // Left Hull
    glPushMatrix();

    glTranslatef(0.f, 0.f, -1.f);

    glScalef(2.f, 0.4f, 0.4f);

    glutSolidCube(1.f);

    glPopMatrix();

    // Right Hull
    glPushMatrix();

    glTranslatef(0.f, 0.f, 1.f);

    glScalef(2.f, 0.4f, 0.4f);

    glutSolidCube(1.f);

    glPopMatrix();

    // Deck
    glPushMatrix();

    glTranslatef(0.f, 0.3f, 0.f);

    glScalef(1.f, 0.1f, 2.f);

    glutSolidCube(1.f);

    glPopMatrix();

    // Mast
    glPushMatrix();

    glTranslatef(0.f, 1.8f, 0.f);

    glScalef(0.1f, 3.f, 0.1f);

    glutSolidCube(1.f);

    glPopMatrix();
}

SailboatApplication::SailboatApplication(): Application(), buoyancy(cyclone::Vector3(0.f, 0.5f, 0.f), 1.f, 3.f, 1.6f),
                                            sail(cyclone::Matrix(0.f, 0.f, 0.f, 0.f,
                                                                 0.f, 0.f, 0.f, 0.f,
                                                                 0.f, 0.f, -1.f, 0.f,
                                                                 0.f, 0.f, 0.f, 1.f),
                                                 cyclone::Vector3(2.f, 0.f, 0.f), &windspeed),
                                            windspeed(0.f, 0.f, 0.f),
                                            sail_control(0.f)

{
    // Set up the boat's rigid body.
    sailboat.SetPosition(0.f, 1.6f, 0.f);

    sailboat.SetOrientation(0.f, 0.f, 0.f, 1.f);

    sailboat.SetVelocity(0.f, 0.f, 0.f);

    sailboat.SetRotation(0.f, 0.f, 0.f);

    sailboat.SetMass(200.f);

    cyclone::Matrix inertiaTensor;

    const auto halfSizes = cyclone::Vector3(2.f, 1.f, 1.f);

    const auto mass = 100.f;

    const auto squares = halfSizes * halfSizes;

    inertiaTensor.M[0][0] = 0.3f * mass * (squares.y + squares.z);

    inertiaTensor.M[0][1] = inertiaTensor.M[1][0] = 0.f;

    inertiaTensor.M[0][2] = inertiaTensor.M[2][0] = 0.f;

    inertiaTensor.M[1][1] = 0.3f * mass * (squares.x + squares.z);

    inertiaTensor.M[1][2] = inertiaTensor.M[2][1] = 0.f;

    inertiaTensor.M[2][2] = 0.3f * mass * (squares.x + squares.y);

    inertiaTensor.M[3][3] = 1.f;

    sailboat.SetInertiaTensor(inertiaTensor);

    sailboat.SetDamping(0.8f, 0.8f);

    sailboat.SetAcceleration(cyclone::Vector3::Gravity);

    sailboat.CalculateDerivedData();

    sailboat.SetAwake();

    sailboat.SetCanSleep(false);

    registry.Add(&sailboat, &sail);

    registry.Add(&sailboat, &buoyancy);
}

SailboatApplication::~SailboatApplication()
{
}

const char* SailboatApplication::GetTitle()
{
    return "Sailboat";
}

void SailboatApplication::Display()
{
    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    auto position = sailboat.GetPosition();

    cyclone::Vector3 offset(4.f, 0.f, 0.f);

    offset = sailboat.GetTransform().TransformVector(offset);

    gluLookAt(position.x + offset.x, position.y + 5.f, position.z + offset.z, position.x, position.y, position.z, 0.f,
              1.f, 0.f);

    glColor3f(0.6f, 0.6f, 0.6f);

    auto bx = static_cast<int>(position.x);

    auto bz = static_cast<int>(position.z);

    glBegin(GL_QUADS);

    for (auto x = -20; x <= 20; x++)
    {
        for (auto z = -20; z <= 20; z++)
        {
            glVertex3f(bx + x - 0.1f, 0.f, bz + z - 0.1f);

            glVertex3f(bx + x - 0.1f, 0.f, bz + z + 0.1f);

            glVertex3f(bx + x + 0.1f, 0.f, bz + z + 0.1f);

            glVertex3f(bx + x + 0.1f, 0.f, bz + z - 0.1f);
        }
    }

    glEnd();

    // Set the transform matrix for the sailboat
    GLfloat gl_transform[16];

    sailboat.GetGLTransform(gl_transform);

    glPushMatrix();

    glMultMatrixf(gl_transform);

    // Draw the boat
    glColor3f(0.f, 0.f, 0.f);

    DrawBoat();

    glPopMatrix();

    char buffer[256];

    sprintf_s(buffer, "Speed %.1f", sailboat.GetVelocity().Size());

    glColor3f(0.f, 0.f, 0.f);

    RenderText(10.f, 24.f, buffer);

    sprintf_s(buffer, "Sail Control: %.1f", sail_control);

    RenderText(10.f, 10.f, buffer);
}

void SailboatApplication::Update()
{
    // Find the duration of the last frame in seconds
    const auto duration = static_cast<cyclone::real>(Timing::Get().lastFrameDuration * 0.001f);

    if (duration <= 0.f)
    {
        return;
    }

    // Start with no forces or acceleration.
    sailboat.ClearAccumulators();

    // Add the forces acting on the boat.
    registry.UpdateForces(duration);

    // Update the boat's physics.
    sailboat.Integrate(duration);

    // Change the wind speed.
    windspeed = windspeed * 0.9f + cyclone::Vector3(r.RandomBinomial(1.f), 0.f, r.RandomBinomial(1.f));

    Application::Update();
}

void SailboatApplication::Key(const unsigned char key)
{
    switch (key)
    {
    case 'q':
    case 'Q':
        {
            sail_control -= 0.1f;
            break;
        }

    case 'e':
    case 'E':
        {
            sail_control += 0.1f;
            break;
        }

    case 'w':
    case 'W':
        {
            sail_control = 0.0f;
            break;
        }

    default:
        Application::Key(key);
    }

    // Make sure the controls are in range
    if (sail_control < -1.f)
    {
        sail_control = -1.f;
    }
    else if (sail_control > 1.f)
    {
        sail_control = 1.f;
    }
}
