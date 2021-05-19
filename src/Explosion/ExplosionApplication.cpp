#include "ExplosionApplication.h"
#include "gl/glut.h"
#include <cmath>
#include "cyclone/RigidBody/FineCollision/IntersectionTests.h"


/**
* Called by the common demo framework to create an application
* object (with new) and return a pointer.
*/
Application* GetApplication()
{
    return new ExplosionApplication();
}

ExplosionApplication::ExplosionApplication(): RigidBodyApplication(), editMode(false), upMode(false)
{
    ExplosionApplication::Reset();
}

void ExplosionApplication::StartUp()
{
    GLfloat lightAmbient[] = {0.8f, 0.8f, 0.8f, 1.f};

    GLfloat lightDiffuse[] = {0.9f, 0.95f, 1.f, 1.f};

    glLightfv(GL_LIGHT0,GL_AMBIENT, lightAmbient);

    glLightfv(GL_LIGHT0,GL_DIFFUSE, lightDiffuse);

    glEnable(GL_LIGHT0);

    Application::StartUp();
}

const char* ExplosionApplication::GetTitle()
{
    return "Explosion";
}

void ExplosionApplication::Key(const unsigned char key)
{
    switch (key)
    {
    case 'e':
    case 'E':
        {
            editMode = !editMode;

            upMode = false;

            return;
        }
    case 't':
    case 'T':
        {
            upMode = !upMode;

            editMode = false;

            return;
        }
    case 'w':
    case 'W':
        {
            for (auto box = boxData; box < boxData + boxes; ++box)
            {
                box->body->SetAwake();
            }

            for (auto ball = ballData; ball < ballData + balls; ++ball)
            {
                ball->body->SetAwake();
            }

            return;
        }
    default: ;
    }

    RigidBodyApplication::Key(key);
}

void ExplosionApplication::Display()
{
    const static GLfloat lightPosition[] = {1.f, -1.f, 0.f, 0.f};

    const static GLfloat lightPositionMirror[] = {1.f, 1.f, 0.f, 0.f};

    // Holds a transform matrix for rendering objects
    // reflected in the floor.
    const static GLfloat floorMirror[16] =
    {
        1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };

    // Update the transform matrices of each box in turn
    for (auto box = boxData; box < boxData + boxes; ++box)
    {
        box->CalculateInternals();

        box->bIsOverlapping = false;
    }

    // Update the transform matrices of each ball in turn
    for (auto ball = ballData; ball < ballData + balls; ++ball)
    {
        // Run the physics
        ball->CalculateInternals();
    }

    // Clear the viewport and set the camera direction
    RigidBodyApplication::Display();

    // Render each element in turn as a shadow
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_LIGHTING);

    glLightfv(GL_LIGHT0,GL_POSITION, lightPosition);

    glPushMatrix();

    glMultMatrixf(floorMirror);

    glColorMaterial(GL_FRONT_AND_BACK,GL_DIFFUSE);

    glEnable(GL_COLOR_MATERIAL);

    for (auto box = boxData; box < boxData + boxes; ++box)
    {
        box->Render();
    }

    for (auto ball = ballData; ball < ballData + balls; ++ball)
    {
        ball->Render();
    }

    glPopMatrix();

    glDisable(GL_LIGHTING);

    glDisable(GL_COLOR_MATERIAL);

    // Draw some scale circles
    glColor3f(0.75f, 0.75f, 0.75f);

    for (auto i = 1u; i < 20; ++i)
    {
        glBegin(GL_LINE_LOOP);

        for (auto j = 0u; j < 32; ++j)
        {
            const auto theta = R_PI * j / 16.f;

            glVertex3f(i * real_cos(theta), 0.f, i * real_sin(theta));
        }

        glEnd();
    }

    glBegin(GL_LINES);

    glVertex3f(-20.f, 0.f, 0.f);

    glVertex3f(20.f, 0.f, 0.f);

    glVertex3f(0.f, 0.f, -20.f);

    glVertex3f(0.0, 0.0, 20.f);

    glEnd();

    // Render each shadow in turn
    glEnable(GL_BLEND);

    glColor4f(0.f, 0.f, 0.f, 0.1f);

    glDisable(GL_DEPTH_TEST);

    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    for (auto box = boxData; box < boxData + boxes; ++box)
    {
        box->RenderShadow();
    }

    for (auto ball = ballData; ball < ballData + balls; ++ball)
    {
        ball->RenderShadow();
    }

    glDisable(GL_BLEND);

    // Render the boxes themselves
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_LIGHTING);

    glLightfv(GL_LIGHT0,GL_POSITION, lightPositionMirror);

    glColorMaterial(GL_FRONT_AND_BACK,GL_DIFFUSE);

    glEnable(GL_COLOR_MATERIAL);

    for (auto box = boxData; box < boxData + boxes; ++box)
    {
        box->Render();
    }

    for (auto ball = ballData; ball < ballData + balls; ++ball)
    {
        ball->Render();
    }

    glDisable(GL_COLOR_MATERIAL);

    glDisable(GL_LIGHTING);

    glDisable(GL_DEPTH_TEST);

    // Finish the frame, rendering any additional information
    DrawDebug();
}

void ExplosionApplication::MouseDrag(const int x, const int y)
{
    if (editMode)
    {
        boxData[0].body->SetPosition(
            boxData[0].body->GetPosition() + cyclone::Vector3((x - last_x) * 0.125f, 0.f, (y - last_y) * 0.125f));

        boxData->body->CalculateDerivedData();
    }
    else if (upMode)
    {
        boxData[0].body->
                   SetPosition(boxData[0].body->GetPosition() + cyclone::Vector3(0.f, (y - last_y) * 0.125f, 0.f));

        boxData[0].body->CalculateDerivedData();
    }
    else
    {
        RigidBodyApplication::MouseDrag(x, y);
    }

    // Remember the position
    last_x = x;

    last_y = y;
}

void ExplosionApplication::Fire() const
{
    auto position = ballData[0].body->GetPosition();

    position.Normalize();

    ballData[0].body->AddForce(position * -1000.f);
}

void ExplosionApplication::GenerateContacts()
{
    // Note that this method makes a lot of use of early returns to avoid
    // processing lots of potential contacts that it hasn't got room to
    // store.

    // Create the ground plane data
    cyclone::CollisionPlane plane;

    plane.direction = cyclone::Vector3(0.f, 1.f, 0.f);

    plane.offset = 0.f;

    // Set up the collision data structure
    collisionData.Reset(maxContacts);

    collisionData.friction = 0.9f;

    collisionData.restitution = 0.6f;

    collisionData.tolerance = 0.1f;

    for (auto box = boxData; box < boxData + boxes; ++box)
    {
        // Check for collisions with the ground plane
        if (!collisionData.HasMoreContacts())
        {
            return;
        }

        cyclone::CollisionDetector::BoxAndHalfSpace(*box, plane, &collisionData);

        // Check for collisions with each other box
        for (auto other = box + 1; other < boxData + boxes; ++other)
        {
            if (!collisionData.HasMoreContacts())
            {
                return;
            }

            cyclone::CollisionDetector::BoxAndBox(*box, *other, &collisionData);

            if (cyclone::IntersectionTests::BoxAndBox(*box, *other))
            {
                box->bIsOverlapping = other->bIsOverlapping = true;
            }
        }

        // Check for collisions with each ball
        for (auto other = ballData; other < ballData + balls; ++other)
        {
            if (!collisionData.HasMoreContacts())
            {
                return;
            }

            cyclone::CollisionDetector::BoxAndSphere(*box, *other, &collisionData);
        }
    }

    for (auto ball = ballData; ball < ballData + balls; ++ball)
    {
        // Check for collisions with the ground plane
        if (!collisionData.HasMoreContacts())
        {
            return;
        }

        cyclone::CollisionDetector::SphereAndHalfSpace(*ball, plane, &collisionData);

        for (auto other = ball + 1; other < ballData + balls; ++other)
        {
            // Check for collisions with the ground plane
            if (!collisionData.HasMoreContacts())
            {
                return;
            }

            cyclone::CollisionDetector::SphereAndSphere(*ball, *other, &collisionData);
        }
    }
}

void ExplosionApplication::UpdateObjects(const cyclone::real deltaTime)
{
    // Update the physics of each box in turn
    for (auto box = boxData; box < boxData + boxes; ++box)
    {
        // Run the physics
        box->body->Integrate(deltaTime);

        box->CalculateInternals();

        box->bIsOverlapping = false;
    }

    // Update the physics of each ball in turn
    for (auto ball = ballData; ball < ballData + balls; ++ball)
    {
        // Run the physics
        ball->body->Integrate(deltaTime);

        ball->CalculateInternals();
    }
}

void ExplosionApplication::Reset()
{
    auto box = boxData;

    box++->SetState(cyclone::Vector3(0.f, 3.f, 0.f), cyclone::Quaternion(), cyclone::Vector3(4.f, 1.f, 1.f),
                    cyclone::Vector3(0.f, 1.f, 0.f));

    if (boxes > 1)
    {
        box++->SetState(cyclone::Vector3(0.f, 4.75f, 2.f), cyclone::Quaternion(1.f, 0.1f, 0.05f, 0.01f),
                        cyclone::Vector3(1.f, 1.f, 4.f), cyclone::Vector3(0.f, 1.f, 0.f));
    }

    // Create the random objects
    cyclone::Random random;

    for (; box < boxData + boxes; ++box)
    {
        box->Random(&random);
    }

    for (auto ball = ballData; ball < ballData + balls; ++ball)
    {
        ball->Random(&random);
    }

    // Reset the contacts
    collisionData.contactCount = 0;
}
