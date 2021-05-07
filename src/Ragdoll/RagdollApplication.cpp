#include "RagdollApplication.h"
#include "gl/glut.h"
#include <cmath>

/**
* Called by the common demo framework to create an application
* object (with new) and return a pointer.
*/
Application* GetApplication()
{
    return new RagdollApplication();
}

RagdollApplication::RagdollApplication(): RigidBodyApplication()
{
    // Set up the bone hierarchy.
    // Right Knee
    joints[0].Set(
        bones[0].body, cyclone::Vector3(0.f, 1.07f, 0.f),
        bones[1].body, cyclone::Vector3(0.f, -1.07f, 0.f),
        0.15f
    );

    // Left Knee
    joints[1].Set(
        bones[2].body, cyclone::Vector3(0.f, 1.07f, 0.f),
        bones[3].body, cyclone::Vector3(0.f, -1.07f, 0.f),
        0.15f
    );

    // Right elbow
    joints[2].Set(
        bones[9].body, cyclone::Vector3(0.f, 0.96f, 0.f),
        bones[8].body, cyclone::Vector3(0.f, -0.96f, 0.f),
        0.15f
    );

    // Left elbow
    joints[3].Set(
        bones[11].body, cyclone::Vector3(0.f, 0.96f, 0.f),
        bones[10].body, cyclone::Vector3(0.f, -0.96f, 0.f),
        0.15f
    );

    // Stomach to Waist
    joints[4].Set(
        bones[4].body, cyclone::Vector3(0.054f, 0.5f, 0.f),
        bones[5].body, cyclone::Vector3(-0.043f, -0.45f, 0.f),
        0.15f
    );

    joints[5].Set(
        bones[5].body, cyclone::Vector3(-0.043f, 0.411f, 0.f),
        bones[6].body, cyclone::Vector3(0.f, -0.411f, 0.f),
        0.15f
    );

    joints[6].Set(
        bones[6].body, cyclone::Vector3(0.f, 0.521f, 0.f),
        bones[7].body, cyclone::Vector3(0.f, -0.752f, 0.f),
        0.15f
    );

    // Right hip
    joints[7].Set(
        bones[1].body, cyclone::Vector3(0.f, 1.066f, 0.f),
        bones[4].body, cyclone::Vector3(0.f, -0.458f, -0.5f),
        0.15f
    );

    // Left Hip
    joints[8].Set(
        bones[3].body, cyclone::Vector3(0.f, 1.066f, 0.f),
        bones[4].body, cyclone::Vector3(0.f, -0.458f, 0.5f),
        0.105f
    );

    // Right shoulder
    joints[9].Set(
        bones[6].body, cyclone::Vector3(0.f, 0.367f, -0.8f),
        bones[8].body, cyclone::Vector3(0.f, 0.888f, 0.32f),
        0.15f
    );

    // Left shoulder
    joints[10].Set(
        bones[6].body, cyclone::Vector3(0.f, 0.367f, 0.8f),
        bones[10].body, cyclone::Vector3(0.f, 0.888f, -0.32f),
        0.15f
    );

    // Set up the initial positions
    RagdollApplication::Reset();
}

void RagdollApplication::StartUp()
{
    GLfloat lightAmbient[] = {0.8f, 0.8f, 0.8f, 1.f};

    GLfloat lightDiffuse[] = {0.9f, 0.95f, 1.f, 1.f};

    glLightfv(GL_LIGHT0,GL_AMBIENT, lightAmbient);

    glLightfv(GL_LIGHT0,GL_DIFFUSE, lightDiffuse);

    glEnable(GL_LIGHT0);

    Application::StartUp();
}

const char* RagdollApplication::GetTitle()
{
    return "Ragdoll";
}

void RagdollApplication::Display()
{
    const static GLfloat lightPosition[] = {0.7f, -1.f, 0.4f, 0.f};

    RigidBodyApplication::Display();

    // Render the bones
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_LIGHTING);

    glEnable(GL_LIGHT0);

    glLightfv(GL_LIGHT0,GL_POSITION, lightPosition);

    glColorMaterial(GL_FRONT_AND_BACK,GL_DIFFUSE);

    glEnable(GL_COLOR_MATERIAL);

    glEnable(GL_NORMALIZE);

    glColor3f(1.f, 0.f, 0.f);

    for (auto i = 0u; i < NUM_BONES; ++i)
    {
        bones[i].Render();
    }

    glDisable(GL_NORMALIZE);

    glDisable(GL_LIGHTING);

    glDisable(GL_COLOR_MATERIAL);

    glDisable(GL_DEPTH_TEST);

    glBegin(GL_LINES);

    for (auto i = 0u; i < NUM_JOINTS; ++i)
    {
        auto joint = joints + i;

        auto a_pos = joint->body[0]->GetPointInWorldSpace(joint->position[0]);

        auto b_pos = joint->body[1]->GetPointInWorldSpace(joint->position[1]);

        const auto length = (b_pos - a_pos).Size();

        if (length > joint->error)
        {
            glColor3f(1.f, 0.f, 0.f);
        }
        else
        {
            glColor3f(0.f, 1.f, 0.f);
        }

        glVertex3f(a_pos.x, a_pos.y, a_pos.z);

        glVertex3f(b_pos.x, b_pos.y, b_pos.z);
    }

    glEnd();

    glEnable(GL_DEPTH_TEST);

    // Draw some scale circles
    glColor3f(0.75f, 0.75f, 0.75f);

    for (auto i = 1; i < 20; ++i)
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

void RagdollApplication::GenerateContacts()
{
    // Create the ground plane data
    cyclone::CollisionPlane plane;

    plane.direction = cyclone::Vector3(0.f, 1.f, 0.f);

    plane.offset = 0.f;

    // Set up the collision data structure
    collisionData.Reset(maxContacts);

    collisionData.friction = 0.9f;

    collisionData.restitution = 0.6f;

    collisionData.tolerance = 0.1f;

    for (auto bone = bones; bone < bones + NUM_BONES; ++bone)
    {
        // Check for collisions with the ground plane
        if (!collisionData.HasMoreContacts())
        {
            return;
        }

        cyclone::CollisionDetector::BoxAndHalfSpace(*bone, plane, &collisionData);

        auto boneSphere = bone->GetCollisionSphere();

        // Check for collisions with each other box
        for (auto other = bone + 1; other < bones + NUM_BONES; ++other)
        {
            if (!collisionData.HasMoreContacts())
            {
                return;
            }

            auto otherSphere = other->GetCollisionSphere();

            cyclone::CollisionDetector::SphereAndSphere(boneSphere, otherSphere, &collisionData);
        }
    }

    // Check for joint violation
    for (auto joint = joints; joint < joints + NUM_JOINTS; ++joint)
    {
        if (!collisionData.HasMoreContacts())
        {
            return;
        }

        const auto added = joint->AddContact(collisionData.contacts, collisionData.contactsLeft);

        collisionData.AddContacts(added);
    }
}

void RagdollApplication::UpdateObjects(const cyclone::real deltaTime)
{
    for (auto bone = bones; bone < bones + NUM_BONES; ++bone)
    {
        bone->body->Integrate(deltaTime);

        bone->CalculateInternals();
    }
}

void RagdollApplication::Reset()
{
    bones[0].SetState(
        cyclone::Vector3(0.f, 0.993f, -0.5f),
        cyclone::Vector3(0.301f, 1.f, 0.234f));

    bones[1].SetState(
        cyclone::Vector3(0.f, 3.159f, -0.56f),
        cyclone::Vector3(0.301f, 1.f, 0.234f));

    bones[2].SetState(
        cyclone::Vector3(0.f, 0.993f, 0.5f),
        cyclone::Vector3(0.301f, 1.f, 0.234f));

    bones[3].SetState(
        cyclone::Vector3(0.f, 3.15f, 0.56f),
        cyclone::Vector3(0.301f, 1.f, 0.234f));

    bones[4].SetState(
        cyclone::Vector3(-0.054f, 4.683f, 0.013f),
        cyclone::Vector3(0.415f, 0.392f, 0.69f));

    bones[5].SetState(
        cyclone::Vector3(0.043f, 5.603f, 0.013f),
        cyclone::Vector3(0.301f, 0.367f, 0.693f));

    bones[6].SetState(
        cyclone::Vector3(0.f, 6.485f, 0.013f),
        cyclone::Vector3(0.435f, 0.367f, 0.786f));

    bones[7].SetState(
        cyclone::Vector3(0.f, 7.759f, 0.013f),
        cyclone::Vector3(0.45f, 0.598f, 0.421f));

    bones[8].SetState(
        cyclone::Vector3(0.f, 5.946f, -1.066f),
        cyclone::Vector3(0.267f, 0.888f, 0.207f));

    bones[9].SetState(
        cyclone::Vector3(0.f, 4.024f, -1.066f),
        cyclone::Vector3(0.267f, 0.888f, 0.207f));

    bones[10].SetState(
        cyclone::Vector3(0.f, 5.946f, 1.066f),
        cyclone::Vector3(0.267f, 0.888f, 0.207f));

    bones[11].SetState(
        cyclone::Vector3(0.f, 4.024f, 1.066f),
        cyclone::Vector3(0.267f, 0.888f, 0.207f));

    const auto strength = -random.RandomReal(500.f, 1000.f);

    for (auto i = 0u; i < NUM_BONES; ++i)
    {
        bones[i].body->AddForceAtBodyPoint(cyclone::Vector3(strength, 0.f, 0.f), cyclone::Vector3());
    }

    bones[6].body->AddForceAtBodyPoint(cyclone::Vector3(strength, 0.f, random.RandomBinomial(1000.f)),
                                       cyclone::Vector3(random.RandomBinomial(4.f), random.RandomBinomial(3.f), 0.f));

    // Reset the contacts
    collisionData.contactCount = 0;
}
