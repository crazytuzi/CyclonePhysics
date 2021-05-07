#include "RigidBodyApplication.h"
#include "Timing.h"
#include "gl/glut.h"

RigidBodyApplication::RigidBodyApplication(): resolver(maxContacts * 8), theta(0.f), phi(15.f), last_x(0), last_y(0),
                                              renderDebugInfo(false), pauseSimulation(true), autoPauseSimulation(false)
{
    collisionData.contactHead = contacts;
}

void RigidBodyApplication::Display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    gluLookAt(18.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f);

    glRotatef(-phi, 0.f, 0.f, 1.f);

    glRotatef(theta, 0.f, 1.f, 0.f);

    glTranslatef(0.f, -5.f, 0.f);
}

void RigidBodyApplication::Update()
{
    // Find the duration of the last frame in seconds
    auto duration = Timing::Get().lastFrameDuration * 0.001f;

    if (duration <= 0.f)
    {
        return;
    }

    if (duration > 0.05f)
    {
        duration = 0.05f;
    }

    // Exit immediately if we aren't running the simulation
    if (pauseSimulation)
    {
        Application::Update();

        return;
    }

    if (autoPauseSimulation)
    {
        pauseSimulation = true;

        autoPauseSimulation = false;
    }

    // Update the objects
    UpdateObjects(duration);

    // Perform the contact generation
    GenerateContacts();

    // Resolve detected contacts
    resolver.ResolveContacts(collisionData.contacts, collisionData.contactCount, duration);

    Application::Update();
}

void RigidBodyApplication::Mouse(int button, int state, const int x, const int y)
{
    // Set the position
    last_x = x;

    last_y = y;
}

void RigidBodyApplication::MouseDrag(const int x, const int y)
{
    // Update the camera
    theta += (x - last_x) * 0.25f;

    phi += (y - last_y) * 0.25f;

    // Keep it in bounds
    if (phi < -20.f)
    {
        phi = -20.f;
    }
    else if (phi > 80.f)
    {
        phi = 80.f;
    }

    // Remember the position
    last_x = x;

    last_y = y;
}

void RigidBodyApplication::Key(const unsigned char key)
{
    switch (key)
    {
    case 'R':
    case 'r':
        {
            // Reset the simulation
            Reset();

            return;
        }
    case 'C':
    case 'c':
        {
            // Toggle rendering of contacts
            renderDebugInfo = !renderDebugInfo;

            return;
        }
    case 'P':
    case 'p':
        {
            // Toggle running the simulation
            pauseSimulation = !pauseSimulation;

            return;
        }
    case ' ':
        {
            // Advance one frame
            autoPauseSimulation = true;

            pauseSimulation = false;
        }

    default:
        Application::Key(key);
    }
}

void RigidBodyApplication::DrawDebug()
{
    if (!renderDebugInfo)
    {
        return;
    }

    // Recalculate the contacts, so they are current (in case we're
    // paused, for example).
    GenerateContacts();

    // Render the contacts, if required
    glBegin(GL_LINES);

    for (auto i = 0u; i < collisionData.contactCount; ++i)
    {
        // Interbody contacts are in green, floor contacts are red.
        if (contacts[i].body[1])
        {
            glColor3f(0.f, 1.f, 0.f);
        }
        else
        {
            glColor3f(1.f, 0.f, 0.f);
        }

        auto vec = contacts[i].contactPoint;

        glVertex3f(vec.x, vec.y, vec.z);

        vec += contacts[i].contactNormal;

        glVertex3f(vec.x, vec.y, vec.z);
    }

    glEnd();
}
