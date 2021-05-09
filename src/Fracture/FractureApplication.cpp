#include "FractureApplication.h"
#include "Ragdoll/Bone.h"
#include "gl/glut.h"
#include <cmath>

/**
* Called by the common demo framework to create an application
* object (with new) and return a pointer.
*/
Application* GetApplication()
{
    return new FractureApplication();
}

FractureApplication::FractureApplication(): RigidBodyApplication(), hit(false), ball_active(false), fracture_contact(0)
{
    // Create the ball.
    ball.body = new cyclone::RigidBody();

    ball.radius = 0.25f;

    ball.body->SetMass(5.f);

    ball.body->SetDamping(0.9f, 0.9f);

    cyclone::Matrix inertiaTensor;

    inertiaTensor.M[0][0] = 5.f;

    inertiaTensor.M[0][1] = inertiaTensor.M[1][0] = 0.f;

    inertiaTensor.M[0][2] = inertiaTensor.M[2][0] = 0.f;

    inertiaTensor.M[1][1] = 5.f;

    inertiaTensor.M[1][2] = inertiaTensor.M[2][1] = 0.f;

    inertiaTensor.M[2][2] = 5.f;

    inertiaTensor.M[3][3] = 1.f;

    ball.body->SetInertiaTensor(inertiaTensor);

    ball.body->SetAcceleration(cyclone::Vector3::Gravity);

    ball.body->SetCanSleep(false);

    ball.body->SetAwake(true);

    // Set up the initial block
    FractureApplication::Reset();
}

const char* FractureApplication::GetTitle()
{
    return "Fracture";
}

void FractureApplication::Display()
{
    const static GLfloat lightPosition[] = {0.7f, 1.f, 0.4f, 0.f};

    RigidBodyApplication::Display();

    glEnable(GL_DEPTH_TEST);

    glEnable(GL_LIGHTING);

    glEnable(GL_LIGHT0);

    glLightfv(GL_LIGHT0,GL_POSITION, lightPosition);

    glColorMaterial(GL_FRONT_AND_BACK,GL_DIFFUSE);

    glEnable(GL_COLOR_MATERIAL);

    glEnable(GL_NORMALIZE);

    for (auto block = blocks; block < blocks + MAX_BLOCKS; ++block)
    {
        if (block->exists)
        {
            block->Render();
        }
    }

    glDisable(GL_NORMALIZE);

    if (ball_active)
    {
        glColor3f(0.4f, 0.7f, 0.4f);

        glPushMatrix();

        const auto position = ball.body->GetPosition();

        glTranslatef(position.x, position.y, position.z);

        glutSolidSphere(0.25f, 16.f, 8.f);

        glPopMatrix();
    }

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

    DrawDebug();
}

void FractureApplication::GenerateContacts()
{
    hit = false;

    // Create the ground plane data
    cyclone::CollisionPlane plane;

    plane.direction = cyclone::Vector3(0.f, 1.f, 0.f);

    plane.offset = 0.f;

    // Set up the collision data structure
    collisionData.Reset(maxContacts);

    collisionData.friction = 0.9f;

    collisionData.restitution = 0.2f;

    collisionData.tolerance = 0.1f;

    for (auto block = blocks; block < blocks + MAX_BLOCKS; ++block)
    {
        if (!block->exists)
        {
            continue;
        }

        // Check for collisions with the ground plane
        if (!collisionData.HasMoreContacts())
        {
            return;
        }

        cyclone::CollisionDetector::BoxAndHalfSpace(*block, plane, &collisionData);

        if (ball_active)
        {
            // And with the sphere
            if (!collisionData.HasMoreContacts())
            {
                return;
            }

            if (cyclone::CollisionDetector::BoxAndSphere(*block, ball, &collisionData))
            {
                hit = true;

                fracture_contact = collisionData.contactCount - 1;
            }
        }

        // Check for collisions with each other box
        for (auto other = block + 1; other < blocks + MAX_BLOCKS; ++other)
        {
            if (!other->exists)
            {
                continue;
            }

            if (!collisionData.HasMoreContacts())
            {
                return;
            }

            cyclone::CollisionDetector::BoxAndBox(*block, *other, &collisionData);
        }
    }

    // Check for sphere ground collisions
    if (ball_active)
    {
        if (!collisionData.HasMoreContacts())
        {
            return;
        }

        cyclone::CollisionDetector::SphereAndHalfSpace(ball, plane, &collisionData);
    }
}

void FractureApplication::UpdateObjects(const cyclone::real deltaTime)
{
    for (auto block = blocks; block < blocks + MAX_BLOCKS; ++block)
    {
        if (block->exists)
        {
            block->body->Integrate(deltaTime);

            block->CalculateInternals();
        }
    }

    if (ball_active)
    {
        ball.body->Integrate(deltaTime);

        ball.CalculateInternals();
    }
}

void FractureApplication::Reset()
{
    // Only the first block exists
    blocks[0].exists = true;

    for (auto block = blocks + 1; block < blocks + MAX_BLOCKS; ++block)
    {
        block->exists = false;
    }

    // Set the first block
    blocks[0].halfSize = cyclone::Vector3(4.f, 4.f, 4.f);

    blocks[0].body->SetPosition(0.f, 7.f, 0.f);

    blocks[0].body->SetOrientation(0.f, 0.f, 0.f, 1.f);

    blocks[0].body->SetVelocity(0.f, 0.f, 0.f);

    blocks[0].body->SetRotation(0.f, 0.f, 0.f);

    const auto mass = 100.f;

    blocks[0].body->SetMass(mass);

    const auto squares = blocks[0].halfSize * blocks[0].halfSize;

    cyclone::Matrix inertiaTensor;

    inertiaTensor.M[0][0] = 0.3f * mass * (squares.y + squares.z);

    inertiaTensor.M[0][1] = inertiaTensor.M[1][0] = 0.f;

    inertiaTensor.M[0][2] = inertiaTensor.M[2][0] = 0.f;

    inertiaTensor.M[1][1] = 0.3f * mass * (squares.x + squares.z);

    inertiaTensor.M[1][2] = inertiaTensor.M[2][1] = 0.f;

    inertiaTensor.M[2][2] = 0.3f * mass * (squares.x + squares.y);

    inertiaTensor.M[3][3] = 1.f;

    blocks[0].body->SetInertiaTensor(inertiaTensor);

    blocks[0].body->SetDamping(0.9f, 0.9f);

    blocks[0].body->CalculateDerivedData();

    blocks[0].CalculateInternals();

    blocks[0].body->SetAcceleration(cyclone::Vector3::Gravity);

    blocks[0].body->SetAwake(true);

    blocks[0].body->SetCanSleep(true);

    ball_active = true;

    // Set up the ball
    ball.body->SetPosition(0.f, 5.f, 20.f);

    ball.body->SetOrientation(0.f, 0.f, 0.f, 1.f);

    ball.body->SetVelocity(random.RandomBinomial(4.f), random.RandomReal(1.f, 6.f), -20.f);

    ball.body->SetRotation(0.f, 0.f, 0.f);

    ball.body->CalculateDerivedData();

    ball.body->SetAwake(true);

    ball.CalculateInternals();

    hit = false;

    // Reset the contacts
    collisionData.contactCount = 0;
}

void FractureApplication::Update()
{
    RigidBodyApplication::Update();

    // Handle fractures.
    if (hit)
    {
        blocks[0].DivideBlock(collisionData.contactHead[fracture_contact], blocks, blocks + 1);

        ball_active = false;
    }
}
