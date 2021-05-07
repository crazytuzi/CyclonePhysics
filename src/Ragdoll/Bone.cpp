#include "Bone.h"
#include "gl/glut.h"

Bone::Bone()
{
    body = new cyclone::RigidBody();
}

Bone::~Bone()
{
    delete body;
}

cyclone::CollisionSphere Bone::GetCollisionSphere() const
{
    cyclone::CollisionSphere sphere;

    sphere.body = body;

    sphere.radius = halfSize.x;

    sphere.offset = cyclone::Matrix();

    if (halfSize.y < sphere.radius)
    {
        sphere.radius = halfSize.y;
    }

    if (halfSize.z < sphere.radius)
    {
        sphere.radius = halfSize.z;
    }

    sphere.CalculateInternals();

    return sphere;
}

void Bone::Render() const
{
    // Get the OpenGL transformation
    GLfloat mat[16];

    body->GetGLTransform(mat);

    if (body->GetAwake())
    {
        glColor3f(0.5f, 0.3f, 0.3f);
    }
    else
    {
        glColor3f(0.3f, 0.3f, 0.5f);
    }

    glPushMatrix();

    glMultMatrixf(mat);

    glScalef(halfSize.x * 2, halfSize.y * 2, halfSize.z * 2);

    glutSolidCube(1.f);

    glPopMatrix();
}

void Bone::SetState(const cyclone::Vector3& position, const cyclone::Vector3& extents)
{
    body->SetPosition(position);

    body->SetOrientation(cyclone::Quaternion());

    body->SetVelocity(cyclone::Vector3());

    body->SetRotation(cyclone::Vector3());

    halfSize = extents;

    const auto mass = halfSize.x * halfSize.y * halfSize.z * 8.f;

    body->SetMass(mass);

    cyclone::Matrix tensor;

    const auto squares = halfSize * halfSize;

    tensor.M[0][0] = 0.3f * mass * (squares.y + squares.z);

    tensor.M[0][1] = tensor.M[1][0] = 0.f;

    tensor.M[0][2] = tensor.M[2][0] = 0.f;

    tensor.M[1][1] = 0.3f * mass * (squares.x + squares.z);

    tensor.M[1][2] = tensor.M[2][1] = 0.f;

    tensor.M[2][2] = 0.3f * mass * (squares.x + squares.y);

    tensor.M[3][3] = 1.f;

    body->SetInertiaTensor(tensor);

    body->SetLinearDamping(0.95f);

    body->SetAngularDamping(0.8f);

    body->ClearAccumulators();

    body->SetAcceleration(cyclone::Vector3::Gravity);

    body->SetCanSleep(false);

    body->SetAwake();

    body->CalculateDerivedData();

    CalculateInternals();
}
