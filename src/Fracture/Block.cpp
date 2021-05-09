#include "Block.h"
#include "gl/glut.h"

Block::Block(): exists(false)
{
    body = new cyclone::RigidBody();
}

Block::~Block()
{
    delete body;
}

void Block::Render() const
{
    // Get the OpenGL transformation
    GLfloat mat[16];

    body->GetGLTransform(mat);

    if (body->GetAwake())
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

void Block::SetState(const cyclone::Vector3& position, const cyclone::Quaternion& orientation,
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

void Block::CalculateMassProperties(const cyclone::real inverseDensity) const
{
    // Check for infinite mass
    if (inverseDensity <= 0)
    {
        // Just set zeros for both mass and inertia tensor
        body->SetInverseMass(0.f);

        body->SetInverseInertiaTensor(cyclone::Matrix());
    }
    else
    {
        // Otherwise we need to calculate the mass
        const auto volume = halfSize.Size() * 2.f;

        const auto mass = volume / inverseDensity;

        body->SetMass(mass);

        // And calculate the inertia tensor from the mass and size
        cyclone::Matrix tensor;

        tensor.M[0][0] = 0.333f * mass * halfSize.y * halfSize.y + halfSize.z * halfSize.z;

        tensor.M[0][1] = tensor.M[1][0] = 0.f;

        tensor.M[0][2] = tensor.M[2][0] = 0.f;

        tensor.M[1][1] = 0.333f * mass * halfSize.y * halfSize.x + halfSize.z * halfSize.z;

        tensor.M[1][2] = tensor.M[2][1] = 0.f;

        tensor.M[2][2] = 0.333f * mass * halfSize.y * halfSize.x + halfSize.z * halfSize.y;

        tensor.M[3][3] = 1.f;

        body->SetInertiaTensor(tensor);
    }
}

void Block::DivideBlock(const cyclone::Contact& contact, Block* target, Block* blocks) const
{
    // Find out if we're block one or two in the contact structure, and
    // therefore what the contact normal is.
    auto normal = contact.contactNormal;

    auto body = contact.body[0];

    if (body != target->body)
    {
        normal *= -1;

        body = contact.body[1];
    }

    // Work out where on the body (in body coordinates) the contact is
    // and its direction.
    auto point = body->GetPointInLocalSpace(contact.contactPoint);

    normal = body->GetDirectionInLocalSpace(normal);

    // Work out the centre of the split: this is the point coordinates
    // for each of the axes perpendicular to the normal, and 0 for the
    // axis along the normal.
    point = point - normal * (point | normal);

    // Take a copy of the half size, so we can create the new blocks.
    const auto& size = target->halfSize;

    // Take a copy also of the body's other data.
    cyclone::RigidBody tempBody;

    tempBody.SetPosition(body->GetPosition());

    tempBody.SetOrientation(body->GetOrientation());

    tempBody.SetVelocity(body->GetVelocity());

    tempBody.SetRotation(body->GetRotation());

    tempBody.SetLinearDamping(body->GetLinearDamping());

    tempBody.SetAngularDamping(body->GetAngularDamping());

    tempBody.SetInverseInertiaTensor(body->GetInverseInertiaTensor());

    tempBody.CalculateDerivedData();

    // Remove the old block
    target->exists = false;

    // Work out the inverse density of the old block
    const auto inverseDensity = halfSize.Size() * 8.f * body->GetInverseMass();

    // Now split the block into eight.
    for (auto i = 0u; i < 8; ++i)
    {
        // Find the minimum and maximum extents of the new block
        // in old-block coordinates
        cyclone::Vector3 min, max;

        if ((i & 1) == 0)
        {
            min.x = -size.x;

            max.x = point.x;
        }
        else
        {
            min.x = point.x;

            max.x = size.x;
        }

        if ((i & 2) == 0)
        {
            min.y = -size.y;

            max.y = point.y;
        }
        else
        {
            min.y = point.y;

            max.y = size.y;
        }

        if ((i & 4) == 0)
        {
            min.z = -size.z;

            max.z = point.z;
        }
        else
        {
            min.z = point.z;

            max.z = size.z;
        }

        // Get the origin and half size of the block, in old-body
        // local coordinates.
        const auto halfSize = (max - min) * 0.5f;

        auto newPosition = halfSize + min;

        // Convert the origin to world coordinates.
        newPosition = tempBody.GetPointInWorldSpace(newPosition);

        // Work out the direction to the impact.
        auto direction = newPosition - contact.contactPoint;

        direction.Normalize();

        // Set the body's properties (we assume the block has a body
        // already that we're going to overwrite).
        blocks[i].body->SetPosition(newPosition);

        blocks[i].body->SetVelocity(tempBody.GetVelocity() + direction * 10.f);

        blocks[i].body->SetOrientation(tempBody.GetOrientation());

        blocks[i].body->SetOrientation(tempBody.GetOrientation());

        blocks[i].body->SetRotation(tempBody.GetRotation());

        blocks[i].body->SetLinearDamping(tempBody.GetLinearDamping());

        blocks[i].body->SetAngularDamping(tempBody.GetAngularDamping());

        blocks[i].body->SetAwake(true);

        blocks[i].body->SetAcceleration(cyclone::Vector3::Gravity);

        blocks[i].body->ClearAccumulators();

        blocks[i].body->CalculateDerivedData();

        blocks[i].offset = cyclone::Matrix();

        blocks[i].exists = true;

        blocks[i].halfSize = halfSize;

        // Finally calculate the mass and inertia tensor of the new block
        blocks[i].CalculateMassProperties(inverseDensity);
    }
}
