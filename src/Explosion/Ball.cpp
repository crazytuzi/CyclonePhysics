#include "Ball.h"
#include "gl/glut.h"

Ball::Ball()
{
    body = new cyclone::RigidBody();
}

Ball::~Ball()
{
    delete body;
}

void Ball::Render() const
{
    // Get the OpenGL transformation
    GLfloat matrix[16];

    body->GetGLTransform(matrix);

    if (body->GetAwake())
    {
        glColor3f(1.f, 0.7f, 0.7f);
    }
    else
    {
        glColor3f(0.7f, 0.7f, 1.f);
    }

    glPushMatrix();

    glMultMatrixf(matrix);

    glutSolidSphere(radius, 20.f, 20.f);

    glPopMatrix();
}

void Ball::RenderShadow() const
{
    // Get the OpenGL transformation
    GLfloat matrix[16];

    body->GetGLTransform(matrix);

    glPushMatrix();

    glScalef(1.f, 0.f, 1.f);

    glMultMatrixf(matrix);

    glutSolidSphere(radius, 20.f, 20.f);

    glPopMatrix();
}

void Ball::SetState(const cyclone::Vector3& position, const cyclone::Quaternion& orientation,
                    const cyclone::real radius, const cyclone::Vector3& velocity)
{
    body->SetPosition(position);

    body->SetOrientation(orientation);

    body->SetVelocity(velocity);

    body->SetRotation(cyclone::Vector3());

    Ball::radius = radius;

    const auto mass = 4.f * 0.3333f * R_PI * radius * radius * radius;

    body->SetMass(mass);

    cyclone::Matrix tensor;

    const auto coefficient = 0.4f * mass * radius * radius;

    tensor.M[0][0] = coefficient;

    tensor.M[0][1] = tensor.M[1][0] = 0.f;

    tensor.M[0][2] = tensor.M[2][0] = 0.f;

    tensor.M[1][1] = coefficient;

    tensor.M[1][2] = tensor.M[2][1] = 0.f;

    tensor.M[2][2] = coefficient;

    tensor.M[3][3] = 1.f;

    body->SetInertiaTensor(tensor);

    body->SetLinearDamping(0.95f);

    body->SetAngularDamping(0.8f);

    body->ClearAccumulators();

    body->SetAcceleration(0.f, -10.f, 0.f);

    body->SetAwake();

    body->CalculateDerivedData();
}

void Ball::Random(cyclone::Random* random)
{
    if (random != nullptr)
    {
        const static cyclone::Vector3 minPosition(-5.f, 5.f, -5.f);

        const static cyclone::Vector3 maxPosition(5.f, 10.0, 5.f);

        SetState(random->RandomVector(minPosition, maxPosition), random->RandomQuaternion(),
                 random->RandomReal(0.5f, 1.5f), cyclone::Vector3());
    }
}
