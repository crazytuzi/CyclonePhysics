#include "FireworksApplication.h"
#include "Timing.h"
#include "gl/glut.h"

/**
* Called by the common demo framework to create an application
* object (with new) and return a pointer.
*/
Application* GetApplication()
{
    return new FireworksApplication();
}

FireworksApplication::FireworksApplication(): nextFirework(0)
{
    // Make all shots unused
    for (auto firework = fireworks; firework < fireworks + maxFireworks; ++firework)
    {
        firework->type = 0;
    }

    // Create the firework types
    InitFireworkRules();
}

FireworksApplication::~FireworksApplication()
{
}

void FireworksApplication::StartUp()
{
    // Call the superclass
    Application::StartUp();

    // But override the clear color
    glClearColor(0.f, 0.f, 0.1f, 1.f);
}

const char* FireworksApplication::GetTitle()
{
    return "Fireworks";
}

void FireworksApplication::Update()
{
    // Find the duration of the last frame in seconds
    const auto duration = static_cast<float>(Timing::Get().lastFrameDuration * 0.001f);

    if (duration <= 0.f)
    {
        return;
    }

    for (auto firework = fireworks; firework < fireworks + maxFireworks; ++firework)
    {
        // Check if we need to process this firework.
        if (firework->type > 0)
        {
            // Does it need removing?
            if (firework->Update(duration))
            {
                // Find the appropriate rule
                const auto rule = rules + (firework->type - 1);

                // Delete the current firework (this doesn't affect its
                // position and velocity for passing to the create function,
                // just whether or not it is processed for rendering or
                // physics.
                firework->type = 0;

                // Add the payload
                for (auto i = 0u; i < rule->payloadCount; ++i)
                {
                    const auto payload = rule->payloads + i;

                    Create(payload->type, payload->count, firework);
                }
            }
        }
    }

    Application::Update();
}

void FireworksApplication::Display()
{
    const static cyclone::real size = 0.1f;

    // Clear the viewport and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    gluLookAt(0.f, 4.f, 10.f, 0.f, 4.f, 0.f, 0.f, 1.f, 0.f);

    // Render each firework in turn
    glBegin(GL_QUADS);

    for (auto firework = fireworks; firework < fireworks + maxFireworks; ++firework)
    {
        // Check if we need to process this firework.
        if (firework->type > 0)
        {
            switch (firework->type)
            {
            case 1: glColor3f(1.f, 0.f, 0.f);
                break;
            case 2: glColor3f(1.f, 0.5f, 0.f);
                break;
            case 3: glColor3f(1.f, 1.f, 0.f);
                break;
            case 4: glColor3f(0.f, 1.f, 0.f);
                break;
            case 5: glColor3f(0.f, 1.f, 1.f);
                break;
            case 6: glColor3f(0.4f, 0.4f, 1.f);
                break;
            case 7: glColor3f(1.f, 0.f, 1.f);
                break;
            case 8: glColor3f(1.f, 1.f, 1.f);
                break;
            case 9: glColor3f(1.f, 0.5f, 0.5f);
                break;
            default:
                break;
            }

            const auto& pos = firework->GetPosition();

            glVertex3f(pos.x - size, pos.y - size, pos.z);

            glVertex3f(pos.x + size, pos.y - size, pos.z);

            glVertex3f(pos.x + size, pos.y + size, pos.z);

            glVertex3f(pos.x - size, pos.y + size, pos.z);

            // Render the firework's reflection
            glVertex3f(pos.x - size, -pos.y - size, pos.z);

            glVertex3f(pos.x + size, -pos.y - size, pos.z);

            glVertex3f(pos.x + size, -pos.y + size, pos.z);

            glVertex3f(pos.x - size, -pos.y + size, pos.z);
        }
    }

    glEnd();
}

void FireworksApplication::Key(const unsigned char key)
{
    switch (key)
    {
    case '1': Create(1, 1, nullptr);
        break;
    case '2': Create(2, 1, nullptr);
        break;
    case '3': Create(3, 1, nullptr);
        break;
    case '4': Create(4, 1, nullptr);
        break;
    case '5': Create(5, 1, nullptr);
        break;
    case '6': Create(6, 1, nullptr);
        break;
    case '7': Create(7, 1, nullptr);
        break;
    case '8': Create(8, 1, nullptr);
        break;
    case '9': Create(9, 1, nullptr);
        break;
    default:
        break;
    }
}

void FireworksApplication::Create(const unsigned type, const Firework* parent)
{
    // Get the rule needed to create this firework
    const auto rule = rules + (type - 1);

    // Create the firework
    rule->Create(fireworks + nextFirework, parent);

    nextFirework = (nextFirework + 1) % maxFireworks;
}

void FireworksApplication::Create(const unsigned type, const unsigned number, const Firework* parent)
{
    for (auto i = 0u; i < number; ++i)
    {
        Create(type, parent);
    }
}

void FireworksApplication::InitFireworkRules()
{
    // Go through the firework types and create their rules.
    rules[0].Init(2);
    rules[0].SetParameters(
        1, // type
        0.5f, 1.4f, // age range
        cyclone::Vector3(-5, 25, -5), // min velocity
        cyclone::Vector3(5, 28, 5), // max velocity
        0.1 // damping
    );
    rules[0].payloads[0].Set(3, 5);
    rules[0].payloads[1].Set(5, 5);

    rules[1].Init(1);
    rules[1].SetParameters(
        2, // type
        0.5f, 1.0f, // age range
        cyclone::Vector3(-5, 10, -5), // min velocity
        cyclone::Vector3(5, 20, 5), // max velocity
        0.8 // damping
    );
    rules[1].payloads[0].Set(4, 2);

    rules[2].Init(0);
    rules[2].SetParameters(
        3, // type
        0.5f, 1.5f, // age range
        cyclone::Vector3(-5, -5, -5), // min velocity
        cyclone::Vector3(5, 5, 5), // max velocity
        0.1 // damping
    );

    rules[3].Init(0);
    rules[3].SetParameters(
        4, // type
        0.25f, 0.5f, // age range
        cyclone::Vector3(-20, 5, -5), // min velocity
        cyclone::Vector3(20, 5, 5), // max velocity
        0.2 // damping
    );

    rules[4].Init(1);
    rules[4].SetParameters(
        5, // type
        0.5f, 1.0f, // age range
        cyclone::Vector3(-20, 2, -5), // min velocity
        cyclone::Vector3(20, 18, 5), // max velocity
        0.01 // damping
    );
    rules[4].payloads[0].Set(3, 5);

    rules[5].Init(0);
    rules[5].SetParameters(
        6, // type
        3, 5, // age range
        cyclone::Vector3(-5, 5, -5), // min velocity
        cyclone::Vector3(5, 10, 5), // max velocity
        0.95 // damping
    );

    rules[6].Init(1);
    rules[6].SetParameters(
        7, // type
        4, 5, // age range
        cyclone::Vector3(-5, 50, -5), // min velocity
        cyclone::Vector3(5, 60, 5), // max velocity
        0.01 // damping
    );
    rules[6].payloads[0].Set(8, 10);

    rules[7].Init(0);
    rules[7].SetParameters(
        8, // type
        0.25f, 0.5f, // age range
        cyclone::Vector3(-1, -1, -1), // min velocity
        cyclone::Vector3(1, 1, 1), // max velocity
        0.01 // damping
    );

    rules[8].Init(0);
    rules[8].SetParameters(
        9, // type
        3, 5, // age range
        cyclone::Vector3(-15, 10, -5), // min velocity
        cyclone::Vector3(15, 15, 5), // max velocity
        0.95 // damping
    );
    // ... and so on for other firework types ...
}
