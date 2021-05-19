#include "Box.h"
#include "gl/glut.h"

Box::Box(): bIsOverlapping(false)
{
    body = new cyclone::RigidBody();
}

Box::~Box()
{
    delete body;
}

void Box::Render() const
{
    // Get the OpenGL transformation
    GLfloat mat[16];

    body->GetGLTransform(mat);

    if (bIsOverlapping)
    {
        glColor3f(0.7f, 1.f, 0.7f);
    }
    else if (body->GetAwake())
    {
        glColor3f(1.f, 0.7f, 0.7f);
    }
    else
    {
        glColor3f(0.7f, 0.7f, 1.f);
    }

    glPushMatrix();

    glMultMatrixf(mat);

    glScalef(halfSize.x * 2, halfSize.y * 2, halfSize.z * 2);

    glutSolidCube(1.f);

    glPopMatrix();
}

void Box::RenderShadow() const
{
    // Get the OpenGL transformation
    GLfloat matrix[16];

    body->GetGLTransform(matrix);

    glPushMatrix();

    glScalef(1.f, 0.f, 1.f);

    glMultMatrixf(matrix);

    glScalef(halfSize.x * 2, halfSize.y * 2, halfSize.z * 2);

    glutSolidCube(1.f);

    glPopMatrix();
}

void Box::SetState(const cyclone::Vector3& position, const cyclone::Quaternion& orientation,
                   const cyclone::Vector3& extents, const cyclone::Vector3& velocity)
{
    body->SetPosition(position);

    body->SetOrientation(orientation);

    body->SetVelocity(velocity);

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

    body->SetAcceleration(0.f, -10.f, 0.f);

    body->SetAwake();

    body->CalculateDerivedData();
}

void Box::Random(cyclone::Random* random)
{
    if (random != nullptr)
    {
        const static cyclone::Vector3 minPosition(-5.f, 5.f, -5.f);

        const static cyclone::Vector3 maxPosition(5.f, 10.0, 5.f);

        const static cyclone::Vector3 minSize(0.5f, 0.5f, 0.5f);

        const static cyclone::Vector3 maxSize(4.5f, 1.5f, 1.5f);

        SetState(random->RandomVector(minPosition, maxPosition), random->RandomQuaternion(),
                 random->RandomVector(minSize, maxSize), cyclone::Vector3());
    }
}
