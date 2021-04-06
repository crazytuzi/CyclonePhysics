#include "RigidBody/Contact/Contact.h"
#include <cassert>
#include <cmath>

using namespace cyclone;

void Contact::SetBodyData(RigidBody* one, RigidBody* two, const real friction, const real restitution)
{
    body[0] = one;

    body[1] = two;

    Contact::friction = friction;

    Contact::restitution = restitution;
}

void Contact::CalculateInternals(const real deltaTime)
{
    // Check if the first object is NULL, and swap if it is.
    if (body[0] == nullptr)
    {
        SwapBodies();
    }

    assert(body[0]);

    // Calculate an set of axis at the contact point.
    CalculateContactBasis();

    // Store the relative position of the contact relative to each body
    relativeContactPosition[0] = contactPoint - body[0]->GetPosition();

    if (body[1] != nullptr)
    {
        relativeContactPosition[1] = contactPoint - body[1]->GetPosition();
    }

    // Find the relative velocity of the bodies at the contact point.
    contactVelocity = CalculateLocalVelocity(0, deltaTime);

    if (body[1] != nullptr)
    {
        contactVelocity -= CalculateLocalVelocity(1, deltaTime);
    }

    // Calculate the desired change in velocity for resolution
    CalculateDesiredDeltaVelocity(deltaTime);
}

void Contact::SwapBodies()
{
    contactNormal *= -1.f;

    const auto temp = body[0];

    body[0] = body[1];

    body[1] = temp;
}

void Contact::MatchAwakeState()
{
    // Collisions with the world never cause a body to wake up.
    if (body[1] == nullptr)
    {
        return;
    }

    const auto body0Awake = body[0]->GetAwake();

    const auto body1Awake = body[1]->GetAwake();

    // Wake up only the sleeping one
    if (body0Awake ^ body1Awake)
    {
        if (body0Awake)
        {
            body[1]->SetAwake();
        }
        else
        {
            body[0]->SetAwake();
        }
    }
}

void Contact::CalculateDesiredDeltaVelocity(const real deltaTime)
{
    if (body[0] == nullptr)
    {
        return;
    }

    const static auto velocityLimit = 0.25f;

    // Calculate the acceleration induced velocity accumulated this frame
    real velocityFromAcceleration = 0.f;

    if (body[0]->GetAwake())
    {
        velocityFromAcceleration += body[0]->GetLastFrameLinearAcceleration() * deltaTime | contactNormal;
    }

    if (body[1] != nullptr && body[1]->GetAwake())
    {
        velocityFromAcceleration -= body[1]->GetLastFrameLinearAcceleration() * deltaTime | contactNormal;
    }

    // If the velocity is very slow, limit the restitution
    auto thisRestitution = restitution;

    if (real_abs(contactVelocity.x) < velocityLimit)
    {
        thisRestitution = 0.f;
    }

    // Combine the bounce velocity with the removed
    // acceleration velocity.
    desiredDeltaVelocity = -contactVelocity.x - thisRestitution * (contactVelocity.x - velocityFromAcceleration);
}

Vector3 Contact::CalculateLocalVelocity(const unsigned bodyIndex, const real deltaTime)
{
    const auto thisBody = body[bodyIndex];

    if (thisBody == nullptr)
    {
        return Vector3::Zero;
    }

    // Work out the velocity of the contact point.
    auto velocity = thisBody->GetRotation() ^ relativeContactPosition[bodyIndex];

    velocity += thisBody->GetVelocity();

    // Turn the velocity into contact-coordinates.
    auto contactVelocity = contactToWorld.InverseTransformVector(velocity);

    // Calculate the amount of velocity that is due to forces without
    // reactions.
    auto accelerationVelocity = thisBody->GetLastFrameLinearAcceleration() * deltaTime;

    // Calculate the velocity in contact-coordinates.
    accelerationVelocity = contactToWorld.InverseTransformVector(accelerationVelocity);

    // We ignore any component of acceleration in the contact normal
    // direction, we are only interested in planar acceleration
    accelerationVelocity.x = 0.f;

    // Add the planar velocities - if there's enough friction they will
    // be removed during velocity resolution
    contactVelocity += accelerationVelocity;

    return contactVelocity;
}

void Contact::CalculateContactBasis()
{
    Vector3 contactTangent[2];

    // Check whether the Z-axis is nearer to the X or Y axis
    if (real_abs(contactNormal.x) > real_abs(contactNormal.y))
    {
        // Scaling factor to ensure the results are normalised
        const auto scale = 1.f / real_sqrt(contactNormal.z * contactNormal.z + contactNormal.x * contactNormal.x);

        // The new X-axis is at right angles to the world Y-axis
        contactTangent[0].x = contactNormal.z * scale;

        contactTangent[0].y = 0.f;

        contactTangent[0].z = -contactNormal.x * scale;

        // The new Y-axis is at right angles to the new X- and Z- axes
        contactTangent[1].x = contactNormal.y * contactTangent[0].x;

        contactTangent[1].y = contactNormal.z * contactTangent[0].x - contactNormal.x * contactTangent[0].z;

        contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
    }
    else
    {
        // Scaling factor to ensure the results are normalised
        const auto scale = 1.f / real_sqrt(contactNormal.z * contactNormal.z + contactNormal.y * contactNormal.y);

        // The new X-axis is at right angles to the world X-axis
        contactTangent[0].x = 0.f;

        contactTangent[0].y = -contactNormal.z * scale;

        contactTangent[0].z = contactNormal.y * scale;

        // The new Y-axis is at right angles to the new X- and Z- axes
        contactTangent[1].x = contactNormal.y * contactTangent[0].z - contactNormal.z * contactTangent[0].y;

        contactTangent[1].y = -contactNormal.x * contactTangent[0].z;

        contactTangent[1].z = contactNormal.x * contactTangent[0].y;
    }

    contactToWorld = Matrix(contactNormal, contactTangent[0], contactTangent[1]);
}

void Contact::ApplyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2])
{
    if (body[0] == nullptr)
    {
        return;
    }

    // Get hold of the inverse mass and inverse inertia tensor, both in
    // world coordinates.
    Matrix inverseInertiaTensor[2];

    body[0]->GetInverseInertiaTensorWorld(&inverseInertiaTensor[0]);

    if (body[1] != nullptr)
    {
        body[1]->GetInverseInertiaTensorWorld(&inverseInertiaTensor[1]);
    }

    // We will calculate the impulse for each contact axis
    Vector3 impulseContact;

    if (friction == 0.f)
    {
        // Use the short format for frictionless contacts
        impulseContact = CalculateFrictionlessImpulse(inverseInertiaTensor);
    }
    else
    {
        // Otherwise we may have impulses that aren't in the direction of the
        // contact, so we need the more complex version.
        impulseContact = CalculateFrictionImpulse(inverseInertiaTensor);
    }

    // Convert impulse to world coordinates
    const auto impulse = contactToWorld.TransformVector(impulseContact);

    // Split in the impulse into linear and rotational components
    auto impulsiveTorque = relativeContactPosition[0] ^ impulse;

    rotationChange[0] = inverseInertiaTensor[0].TransformVector(impulsiveTorque);

    velocityChange[0] = impulse * body[0]->GetInverseMass();

    // Apply the changes
    body[0]->AddVelocity(velocityChange[0]);

    body[0]->AddRotation(rotationChange[0]);

    if (body[1] != nullptr)
    {
        // Work out body one's linear and angular changes
        impulsiveTorque = impulse ^ relativeContactPosition[1];

        rotationChange[1] = inverseInertiaTensor[1].TransformVector(impulsiveTorque);

        velocityChange[1] = impulse * -body[1]->GetInverseMass();

        // And apply them.
        body[1]->AddVelocity(velocityChange[1]);

        body[1]->AddRotation(rotationChange[1]);
    }
}

void Contact::ApplyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], const real penetration)
{
    const static auto angularLimit = 0.2f;

    real angularMove[2];

    real linearMove[2];

    real totalInertia = 0.f;

    real linearInertia[2];

    real angularInertia[2];

    // We need to work out the inertia of each object in the direction
    // of the contact normal, due to angular inertia only.
    for (auto i = 0u; i < 2; ++i)
    {
        if (body[i] != nullptr)
        {
            Matrix inverseInertiaTensor;

            body[i]->GetInverseInertiaTensorWorld(&inverseInertiaTensor);

            // Use the same procedure as for calculating frictionless
            // velocity change to work out the angular inertia.
            auto angularInertiaWorld = relativeContactPosition[i] ^ contactNormal;

            angularInertiaWorld = inverseInertiaTensor.TransformVector(angularInertiaWorld);

            angularInertiaWorld = angularInertiaWorld ^ relativeContactPosition[i];

            angularInertia[i] = angularInertiaWorld | contactNormal;

            // The linear component is simply the inverse mass
            linearInertia[i] = body[i]->GetInverseMass();

            // Keep track of the total inertia from all components
            totalInertia += linearInertia[i] + angularInertia[i];
        }
    }

    // Loop through again calculating and applying the changes
    for (auto i = 0u; i < 2; ++i)
    {
        if (body[i] != nullptr)
        {
            // The linear and angular movements required are in proportion to
            // the two inverse inertia.
            const auto sign = i == 0 ? 1.f : -1.f;

            angularMove[i] = sign * penetration * (angularInertia[i] / totalInertia);

            linearMove[i] = sign * penetration * (linearInertia[i] / totalInertia);

            // To avoid angular projections that are too great (when mass is large
            // but inertia tensor is small) limit the angular move.
            auto projection = relativeContactPosition[i];

            projection += contactNormal * (-relativeContactPosition[i] | contactNormal);

            // Use the small angle approximation for the sine of the angle (i.e.
            // the magnitude would be sine(angularLimit) * projection.magnitude
            // but we approximate sine(angularLimit) to angularLimit).
            const auto maxMagnitude = angularLimit * projection.Size();

            if (angularMove[i] < -maxMagnitude)
            {
                const auto totalMove = angularMove[i] + linearMove[i];

                angularMove[i] = -maxMagnitude;

                linearMove[i] = totalMove - angularMove[i];
            }
            else if (angularMove[i] > maxMagnitude)
            {
                const auto totalMove = angularMove[i] + linearMove[i];

                angularMove[i] = maxMagnitude;

                linearMove[i] = totalMove - angularMove[i];
            }

            // We have the linear amount of movement required by turning
            // the rigid body (in angularMove[i]). We now need to
            // calculate the desired rotation to achieve that.
            if (angularMove[i] == 0.f)
            {
                // Easy case - no angular movement means no rotation.
                angularChange[i].Reset();
            }
            else
            {
                // Work out the direction we'd like to rotate in.
                auto targetAngularDirection = relativeContactPosition[i] ^ contactNormal;

                Matrix inverseInertiaTensor;

                body[i]->GetInverseInertiaTensorWorld(&inverseInertiaTensor);

                // Work out the direction we'd need to rotate to achieve that
                angularChange[i] = inverseInertiaTensor.TransformVector(targetAngularDirection) * (angularMove[i] /
                    angularInertia[i]);
            }

            // Velocity change is easier - it is just the linear movement
            // along the contact normal.
            linearChange[i] = contactNormal * linearMove[i];

            // Now we can start to apply the values we've calculated.
            // Apply the linear movement
            auto position = body[i]->GetPosition();

            position += linearChange[i];

            body[i]->SetPosition(position);

            // And the change in orientation
            Quaternion quaternion = body[i]->GetOrientation();

            quaternion += angularChange[i];

            body[i]->SetOrientation(quaternion);

            // We need to calculate the derived data for any body that is
            // asleep, so that the changes are reflected in the object's
            // data. Otherwise the resolution will not change the position
            // of the object, and the next collision detection round will
            // have the same penetration.
            if (!body[i]->GetAwake())
            {
                body[i]->CalculateDerivedData();
            }
        }
    }
}

Vector3 Contact::CalculateFrictionlessImpulse(Matrix* inverseInertiaTensor)
{
    if (body[0] == nullptr)
    {
        return Vector3::Zero;
    }

    Vector3 impulseContact;

    // Build a vector that shows the change in velocity in
    // world space for a unit impulse in the direction of the contact
    // normal.
    auto deltaVelWorld = relativeContactPosition[0] ^ contactNormal;

    deltaVelWorld = inverseInertiaTensor[0].TransformVector(deltaVelWorld);

    deltaVelWorld = deltaVelWorld ^ relativeContactPosition[0];

    // Work out the change in velocity in contact coordinates.
    auto deltaVelocity = deltaVelWorld | contactNormal;

    // Add the linear component of velocity change
    deltaVelocity += body[0]->GetInverseMass();

    // Check if we need to the second body's data
    if (body[1] != nullptr)
    {
        // Go through the same transformation sequence again
        deltaVelWorld = relativeContactPosition[1] ^ contactNormal;

        deltaVelWorld = inverseInertiaTensor[1].TransformVector(deltaVelWorld);

        deltaVelWorld = deltaVelWorld ^ relativeContactPosition[1];

        // Add the change in velocity due to rotation
        deltaVelocity += deltaVelWorld | contactNormal;

        // Add the change in velocity due to linear motion
        deltaVelocity += body[1]->GetInverseMass();
    }

    // Calculate the required size of the impulse
    impulseContact.x = desiredDeltaVelocity / deltaVelocity;

    impulseContact.y = 0.f;

    impulseContact.z = 0.f;

    return impulseContact;
}

Vector3 Contact::CalculateFrictionImpulse(Matrix* inverseInertiaTensor)
{
    auto inverseMass = body[0]->GetInverseMass();

    // The equivalent of a cross product in matrices is multiplication
    // by a skew symmetric matrix - we build the matrix for converting
    // between linear and angular quantities.
    Matrix impulseToTorque(0.f, -relativeContactPosition[0].z, relativeContactPosition[0].y, 0.f,
                           relativeContactPosition[0].z, 0.f, -relativeContactPosition[0].x, 0.f,
                           -relativeContactPosition[0].y, relativeContactPosition[0].x, 0.f, 0.f);

    // Build the matrix to convert contact impulse to change in velocity
    // in world coordinates.
    Matrix deltaVelWorld = impulseToTorque;

    deltaVelWorld *= inverseInertiaTensor[0];

    deltaVelWorld *= impulseToTorque;

    deltaVelWorld *= -1.f;

    // Check if we need to add body two's data
    if (body[1] != nullptr)
    {
        // Set the cross product matrix
        impulseToTorque = Matrix(0.f, -relativeContactPosition[1].z, relativeContactPosition[1].y, 0.f,
                                 relativeContactPosition[1].z, 0.f, -relativeContactPosition[1].x, 0.f,
                                 -relativeContactPosition[1].y, relativeContactPosition[1].x, 0.f, 0.f);

        // Calculate the velocity change matrix
        auto deltaVelWorld2 = impulseToTorque;

        deltaVelWorld2 *= inverseInertiaTensor[1];

        deltaVelWorld2 *= impulseToTorque;

        deltaVelWorld2 *= -1.f;

        // Add to the total delta velocity.
        deltaVelWorld += deltaVelWorld2;

        // Add to the inverse mass
        inverseMass += body[1]->GetInverseMass();
    }

    // Do a change of basis to convert into contact coordinates.
    auto deltaVelocity = contactToWorld.Transposed();

    deltaVelocity *= deltaVelWorld;

    deltaVelocity *= contactToWorld;

    // Add in the linear velocity change
    deltaVelocity.M[0][0] += inverseMass;

    deltaVelocity.M[1][1] += inverseMass;

    deltaVelocity.M[2][2] += inverseMass;

    // Invert to get the impulse needed per unit velocity
    const auto impulseMatrix = deltaVelocity.Inverse();

    // Find the target velocities to kill0
    const Vector3 velocityKill(desiredDeltaVelocity, -contactVelocity.y, -contactVelocity.z);

    // Find the impulse to kill target velocities
    auto impulseContact = impulseMatrix.TransformVector(velocityKill);

    // Check for exceeding friction
    const auto planarImpulse = real_sqrt(impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);

    if (planarImpulse > impulseContact.x * friction)
    {
        // We need to use dynamic friction
        impulseContact.y /= planarImpulse;

        impulseContact.z /= planarImpulse;

        impulseContact.x = deltaVelocity.M[0][0] + deltaVelocity.M[0][1] * friction * impulseContact.y + deltaVelocity.M
            [0][2] * friction * impulseContact.z;

        impulseContact.x = desiredDeltaVelocity / impulseContact.x;

        impulseContact.y *= friction * impulseContact.x;

        impulseContact.z *= friction * impulseContact.x;
    }

    return impulseContact;
}
