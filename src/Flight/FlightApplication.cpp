#include "FlightApplication.h"
#include "Timing.h"
#include "gl/glut.h"

/**
* Called by the common demo framework to create an application
* object (with new) and return a pointer.
*/
Application* GetApplication()
{
    return new FlightApplication();
}

static void DrawAircraft()
{
    // Fuselage
    glPushMatrix();

    glTranslatef(-0.5f, 0.f, 0.f);

    glScalef(2.f, 0.8f, 1.f);

    glutSolidCube(1.f);

    glPopMatrix();

    // Rear Fuselage
    glPushMatrix();

    glTranslatef(1.f, 0.15f, 0.f);

    glScalef(2.75f, 0.5f, 0.5f);

    glutSolidCube(1.f);

    glPopMatrix();

    // Wings
    glPushMatrix();

    glTranslatef(0.f, 0.3f, 0.f);

    glScalef(0.8f, 0.1f, 6.f);

    glutSolidCube(1.f);

    glPopMatrix();

    // Rudder
    glPushMatrix();

    glTranslatef(2.f, 0.775f, 0.f);

    glScalef(0.75f, 1.15f, 0.1f);

    glutSolidCube(1.f);

    glPopMatrix();

    // Tail-plane
    glPushMatrix();
    
    glTranslatef(1.9f, 0.f, 0.f);

    glScalef(0.85f, 0.1f, 2.f);

    glutSolidCube(1.f);

    glPopMatrix();
}

FlightApplication::FlightApplication(): Application(),
                                        left_wing(cyclone::Matrix(0.f, 0.f, 0.f, 0.f,
                                                                  -1.f, -0.5f, 0.f, 0.f,
                                                                  0.f, 0.f, 0.f, 0.f,
                                                                  0.f, 0.f, 0.f, 1.f),
                                                  cyclone::Matrix(0.f, 0.f, 0.f, 0.f,
                                                                  -0.995f, -0.5f, 0.f, 0.f,
                                                                  0.f, 0.f, 0.f, 0.f,
                                                                  0.f, 0.f, 0.f, 1.f),
                                                  cyclone::Matrix(0.f, 0.f, 0.f, 0.f,
                                                                  -1.005f, -0.5f, 0.f, 0.f,
                                                                  0.f, 0.f, 0.f, 0.f,
                                                                  0.f, 0.f, 0.f, 1.f),
                                                  cyclone::Vector3(-1.f, 0.f, -2.f),
                                                  &windspeed),
                                        right_wing(cyclone::Matrix(0.f, 0.f, 0.f, 0.f,
                                                                   -1.f, -0.5f, 0.f, 0.f,
                                                                   0.f, 0.f, 0.f, 0.f,
                                                                   0.f, 0.f, 0.f, 1.f),
                                                   cyclone::Matrix(0.f, 0.f, 0.f, 0.f,
                                                                   -0.995f, -0.5f, 0.f, 0.f,
                                                                   0.f, 0.f, 0.f, 0.f,
                                                                   0.f, 0.f, 0.f, 1.f),
                                                   cyclone::Matrix(0.f, 0.f, 0.f, 0.f,
                                                                   -1.005f, -0.5f, 0.f, 0.f,
                                                                   0.f, 0.f, 0.f, 0.f,
                                                                   0.f, 0.f, 0.f, 1.f),
                                                   cyclone::Vector3(-1.f, 0.f, 2.f),
                                                   &windspeed),
                                        rudder(cyclone::Matrix(0.f, 0.f, 0.f, 0.f,
                                                               0.f, 0.f, 0.f, 0.f,
                                                               0.f, 0.f, 0.f, 0.f,
                                                               0.f, 0.f, 0.f, 1.f),
                                               cyclone::Matrix(0.f, 0.f, 0.f, 0.f,
                                                               0.f, 0.f, 0.f, 0.f,
                                                               0.f, 0.f, 0.01f, 0.f,
                                                               0.f, 0.f, 0.f, 1.f),
                                               cyclone::Matrix(0.f, 0.f, 0.f, 0.f,
                                                               0.f, 0.f, 0.f, 0.f,
                                                               0.f, 0.f, -0.01f, 0.f,
                                                               0.f, 0.f, 0.f, 1.f),
                                               cyclone::Vector3(2.f, 0.5f, 0.f),
                                               &windspeed),
                                        tail(cyclone::Matrix(0.f, 0.f, 0.f, 0.f,
                                                             -1.f, -0.5f, 0.f, 0.f,
                                                             0.f, 0.f, -0.1f, 0.f,
                                                             0.f, 0.f, 0.f, 1.f),
                                             cyclone::Vector3(2.f, 0.f, 0.f),
                                             &windspeed),
                                        left_wing_control(0.f),
                                        right_wing_control(0.f),
                                        rudder_control(0.f),
                                        windspeed(0.f, 0.f, 0.f)

{
    // Set up the aircraft rigid body.
    ResetPlane();

    aircraft.SetMass(2.5f);

    cyclone::Matrix inertiaTensor;

    const auto halfSizes = cyclone::Vector3(2.f, 1.f, 1.f);

    const auto mass = 1.f;

    const auto squares = halfSizes * halfSizes;

    inertiaTensor.M[0][0] = 0.3f * mass * (squares.y + squares.z);

    inertiaTensor.M[0][1] = inertiaTensor.M[1][0] = 0.f;

    inertiaTensor.M[0][2] = inertiaTensor.M[2][0] = 0.f;

    inertiaTensor.M[1][1] = 0.3f * mass * (squares.x + squares.z);

    inertiaTensor.M[1][2] = inertiaTensor.M[2][1] = 0.f;

    inertiaTensor.M[2][2] = 0.3f * mass * (squares.x + squares.y);

    inertiaTensor.M[3][3] = 1.f;

    aircraft.SetInertiaTensor(inertiaTensor);
    
    aircraft.SetDamping(0.8f, 0.8f);
    
    aircraft.SetAcceleration(cyclone::Vector3::Gravity);
    
    aircraft.CalculateDerivedData();
    
    aircraft.SetAwake();
    
    aircraft.SetCanSleep(false);

    registry.Add(&aircraft, &left_wing);

    registry.Add(&aircraft, &right_wing);

    registry.Add(&aircraft, &rudder);

    registry.Add(&aircraft, &tail);
}

FlightApplication::~FlightApplication()
{
}

const char* FlightApplication::GetTitle()
{
    return "Platform";
}

void FlightApplication::Display()
{
    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    const auto position = aircraft.GetPosition();

    auto offset = cyclone::Vector3(4.f + aircraft.GetVelocity().Size(), 0.f, 0.f);

    offset = aircraft.GetTransform().TransformVector(offset);

    gluLookAt(position.x + offset.x, position.y + 5.f, position.z + offset.z,
              position.x, position.y, position.z,
              0.f, 1.f, 0.f);

    glColor3f(0.6f, 0.6f, 0.6f);

    const auto bx = static_cast<int>(position.x);

    const auto bz = static_cast<int>(position.z);

    glBegin(GL_QUADS);

    for (auto x = -20; x <= 20; ++x)
    {
        for (auto z = -20; z <= 20; ++z)
        {
            glVertex3f(bx + x - 0.1f, 0, bz + z - 0.1f);

            glVertex3f(bx + x - 0.1f, 0, bz + z + 0.1f);

            glVertex3f(bx + x + 0.1f, 0, bz + z + 0.1f);

            glVertex3f(bx + x + 0.1f, 0, bz + z - 0.1f);
        }
    }

    glEnd();

    // Set the transform matrix for the aircraft
    GLfloat gl_transform[16];

    aircraft.GetGLTransform(gl_transform);

    glPushMatrix();

    glMultMatrixf(gl_transform);

    // Draw the aircraft
    glColor3f(0.f, 0.f, 0.f);

    DrawAircraft();

    glPopMatrix();

    glColor3f(0.8f, 0.8f, 0.8f);

    glPushMatrix();

    glTranslatef(0.f, -1.f - position.y, 0.f);

    glScalef(1.f, 0.001f, 1.f);

    glMultMatrixf(gl_transform);

    DrawAircraft();

    glPopMatrix();

    char buffer[256];

    sprintf_s(buffer, "Altitude: %.1f | Speed %.1f", aircraft.GetPosition().y, aircraft.GetVelocity().Size());

    glColor3f(0.f, 0.f, 0.f);

    RenderText(10.f, 24.f, buffer);

    sprintf_s(buffer, "Left Wing: %.1f | Right Wing: %.1f | Rudder %.1f", left_wing_control, right_wing_control,
              rudder_control);

    RenderText(10.f, 10.f, buffer);
}

void FlightApplication::Update()
{
    // Find the duration of the last frame in seconds
    const auto duration = Timing::Get().lastFrameDuration * 0.001f;
    
    if (duration <= 0.f)
    {
        return;
    }
    
    // Start with no forces or acceleration.
    aircraft.ClearAccumulators();
    
    // Add the propeller force
    cyclone::Vector3 propulsion(-10.f, 0.f, 0.f);
    
    propulsion = aircraft.GetTransform().TransformVector(propulsion);
    
    aircraft.AddForce(propulsion);
    
    // Add the forces acting on the aircraft.
    registry.UpdateForces(duration);
    
    // Update the aircraft's physics.
    aircraft.Integrate(duration);
    
    // Do a very basic collision detection and response with the ground.
    auto position = aircraft.GetPosition();
    
    if (position.y < 0.f)
    {
        position.y = 0.f;
    
        aircraft.SetPosition(position);
    
        if (aircraft.GetVelocity().y < -10.f)
        {
            ResetPlane();
        }
    }
    
    Application::Update();
}

void FlightApplication::Key(const unsigned char key)
{
    switch (key)
    {
    case 'q':
    case 'Q':
        {
            rudder_control += 0.1f;

            break;
        }
    case 'e':
    case 'E':
        {
            rudder_control -= 0.1f;

            break;
        }
    case 'w':
    case 'W':
        {
            left_wing_control -= 0.1f;

            right_wing_control -= 0.1f;

            break;
        }
    case 's':
    case 'S':
        {
            left_wing_control += 0.1f;

            right_wing_control += 0.1f;

            break;
        }
    case 'd':
    case 'D':
        {
            left_wing_control -= 0.1f;

            right_wing_control += 0.1f;

            break;
        }
    case 'a':
    case 'A':
        {
            left_wing_control += 0.1f;

            right_wing_control -= 0.1f;

            break;
        }
    case 'x':
    case 'X':
        {
            left_wing_control = 0.f;

            right_wing_control = 0.f;

            rudder_control = 0.f;

            break;
        }
    case 'r':
    case 'R':
        {
            ResetPlane();

            break;
        }
    default:
        Application::Key(key);
    }

    // Make sure the controls are in range
    if (left_wing_control < -1.f)
    {
        left_wing_control = -1.f;
    }
    else if (left_wing_control > 1.f)
    {
        left_wing_control = 1.f;
    }

    if (right_wing_control < -1.f)
    {
        right_wing_control = -1.f;
    }
    else if (right_wing_control > 1.f)
    {
        right_wing_control = 1.f;
    }

    if (rudder_control < -1.f)
    {
        rudder_control = -1.f;
    }
    else if (rudder_control > 1.f)
    {
        rudder_control = 1.f;
    }

    // Update the control surfaces
    left_wing.SetControl(left_wing_control);

    right_wing.SetControl(right_wing_control);

    rudder.SetControl(rudder_control);
}

void FlightApplication::ResetPlane()
{
    aircraft.SetPosition(0.f, 0.f, 0.f);

    aircraft.SetOrientation(0.f, 0.f, 0.f, 1.f);

    aircraft.SetVelocity(0.f, 0.f, 0.f);

    aircraft.SetRotation(0.f, 0.f, 0.f);
}
