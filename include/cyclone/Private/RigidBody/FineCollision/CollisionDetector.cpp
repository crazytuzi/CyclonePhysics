#include "RigidBody/FineCollision/CollisionDetector.h"
#include "RigidBody/FineCollision/IntersectionTests.h"
#include <cfloat>
#include <cmath>
#include <cassert>

using namespace cyclone;

static real TransformToAxis(const CollisionBox& box, const Vector3& axis)
{
    return box.halfSize.x * real_abs(axis | box.GetAxis(0)) + box.halfSize.y * real_abs(axis | box.GetAxis(1)) + box.
        halfSize.z * real_abs(axis | box.GetAxis(2));
}

/*
* This function checks if the two boxes overlap
* along the given axis, returning the amount of overlap.
* The final parameter toCentre
* is used to pass in the vector between the boxes centre
* points, to avoid having to recalculate it each time.
*/
static real PenetrationOnAxis(const CollisionBox& one, const CollisionBox& two, const Vector3& axis,
                              const Vector3& toCentre)
{
    // Project the half-size of one onto axis
    const auto oneProject = TransformToAxis(one, axis);

    const auto twoProject = TransformToAxis(two, axis);

    // Project this onto the axis
    const auto distance = real_abs(toCentre | axis);

    // Return the overlap (i.e. positive indicates
    // overlap, negative indicates separation).
    return oneProject + twoProject - distance;
}

static bool TryAxis(const CollisionBox& one, const CollisionBox& two, Vector3 axis, const Vector3& toCenter,
                    const unsigned index, real& smallestPenetration, unsigned& smallestCase)
{
    // Make sure we have a normalized axis, and don't check almost parallel axes
    if (axis.SizeSquared() < 0.0001f)
    {
        return true;
    }

    axis.Normalize();

    const auto penetration = PenetrationOnAxis(one, two, axis, toCenter);

    if (penetration < 0.f)
    {
        return false;
    }

    if (penetration < smallestPenetration)
    {
        smallestPenetration = penetration;

        smallestCase = index;
    }

    return true;
}

static void FillPointFaceBoxBox(const CollisionBox& one, const CollisionBox& two, const Vector3& toCentre,
                                CollisionData* data, const unsigned best, const real pen)
{
    // This method is called when we know that a vertex from
    // box two is in contact with box one.
    if (data == nullptr)
    {
        return;
    }

    auto contact = data->contacts;

    if (contact == nullptr)
    {
        return;
    }

    // We know which axis the collision is on (i.e. best),
    // but we need to work out which of the two faces on
    // this axis.
    auto normal = one.GetAxis(best);

    if ((one.GetAxis(best) | toCentre) > 0)
    {
        normal *= -1.f;
    }

    // Work out which vertex of box two we're colliding with.
    // Using toCentre doesn't work!
    auto vertex = two.halfSize;

    if ((two.GetAxis(0) | normal) < 0)
    {
        vertex.x = -vertex.x;
    }

    if ((two.GetAxis(1) | normal) < 0)
    {
        vertex.y = -vertex.y;
    }

    if ((two.GetAxis(2) | normal) < 0)
    {
        vertex.z = -vertex.z;
    }

    // Create the contact data
    contact->contactNormal = normal;

    contact->penetration = pen;

    contact->contactPoint = two.GetTransform() * vertex;

    contact->SetBodyData(one.body, two.body, data->friction, data->restitution);
}

static Vector3 ContactPoint(const Vector3& pointOne, const Vector3& axisOne, const real oneSize,
                            const Vector3& pointTwo, const Vector3& axisTwo, const real twoSize, const bool useOne)
{
    const auto smOne = axisOne.SizeSquared();

    const auto smTwo = axisTwo.SizeSquared();

    const auto dpOneTwo = axisTwo | axisOne;

    const auto toSt = pointOne - pointTwo;

    const auto dpStaOne = axisOne | toSt;

    const auto dpStaTwo = axisTwo | toSt;

    const auto denominator = smOne * smTwo - dpOneTwo * dpOneTwo;

    // Zero denominator indicates parallel lines
    if (real_abs(denominator) < 0.0001f)
    {
        return useOne ? pointOne : pointTwo;
    }

    const auto mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denominator;

    const auto mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denominator;

    // If either of the edges has the nearest point out
    // of bounds, then the edges aren't crossed, we have
    // an edge-face contact. Our point is on the edge, which
    // we know from the useOne parameter.
    if (mua > oneSize || mua < -oneSize || mub > twoSize || mub < -twoSize)
    {
        return useOne ? pointOne : pointTwo;
    }

    return (pointOne + axisOne * mua) * 0.5f + (pointTwo + axisTwo * mub) * 0.5f;
}

unsigned CollisionDetector::SphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane,
                                               CollisionData* data)
{
    // Make sure we have contacts
    if (data == nullptr || !data->HasMoreContacts())
    {
        return 0;
    }

    // Cache the sphere position
    const auto position = sphere.GetAxis(3);

    // Find the distance from the plane
    const auto ballDistance = plane.direction | position - sphere.radius - plane.offset;

    if (ballDistance >= 0)
    {
        return 0;
    }

    // Create the contact - it has a normal in the plane direction.
    auto contact = data->contacts;

    if (contact == nullptr)
    {
        return 0;
    }

    contact->contactNormal = plane.direction;

    contact->penetration = -ballDistance;

    contact->contactPoint = position - plane.direction * (ballDistance + sphere.radius);

    contact->SetBodyData(sphere.body, nullptr, data->friction, data->restitution);

    data->AddContacts(1);

    return 1;
}

unsigned CollisionDetector::SphereAndTruePlane(const CollisionSphere& sphere, const CollisionPlane& plane,
                                               CollisionData* data)
{
    // Make sure we have contacts
    if (data == nullptr || !data->HasMoreContacts())
    {
        return 0;
    }

    // Cache the sphere position
    const auto position = sphere.GetAxis(3);

    // Find the distance from the plane
    const auto centreDistance = plane.direction | position - plane.offset;

    // Check if we're within radius
    if (centreDistance * centreDistance > sphere.radius * sphere.radius)
    {
        return 0;
    }

    // Check which side of the plane we're on
    auto normal = plane.direction;

    auto penetration = -centreDistance;

    if (centreDistance < 0)
    {
        normal *= -1;

        penetration = -penetration;
    }

    penetration += sphere.radius;

    // Create the contact - it has a normal in the plane direction.
    auto contact = data->contacts;

    if (contact == nullptr)
    {
        return 0;
    }

    contact->contactNormal = normal;

    contact->penetration = penetration;

    contact->contactPoint = position - plane.direction * centreDistance;

    contact->SetBodyData(sphere.body, nullptr, data->friction, data->restitution);

    data->AddContacts(1);

    return 1;
}

unsigned CollisionDetector::SphereAndSphere(const CollisionSphere& one, const CollisionSphere& two, CollisionData* data)
{
    // Make sure we have contacts
    if (data == nullptr || !data->HasMoreContacts())
    {
        return 0;
    }

    // Cache the sphere positions
    const auto positionOne = one.GetAxis(3);

    const auto positionTwo = two.GetAxis(3);

    // Find the vector between the objects
    const auto midLine = positionOne - positionTwo;

    const auto size = midLine.Size();

    // See if it is large enough.
    if (size <= 0.f || size >= one.radius + two.radius)
    {
        return 0;
    }

    // We manually create the normal, because we have the
    // size to hand.
    const auto normal = midLine / size;

    auto contact = data->contacts;

    if (contact == nullptr)
    {
        return 0;
    }

    contact->contactNormal = normal;

    contact->penetration = one.radius + two.radius - size;

    contact->contactPoint = positionOne + midLine * 0.5f;

    contact->SetBodyData(one.body, two.body, data->friction, data->restitution);

    data->AddContacts(1);

    return 1;
}

unsigned CollisionDetector::BoxAndHalfSpace(const CollisionBox& box, const CollisionPlane& plane, CollisionData* data)
{
    // Make sure we have contacts
    if (data == nullptr || !data->HasMoreContacts())
    {
        return 0;
    }

    // Check for intersection
    if (!IntersectionTests::BoxAndHalfSpace(box, plane))
    {
        return 0;
    }

    // We have an intersection, so find the intersection points. We can make
    // do with only checking vertices. If the box is resting on a plane
    // or on an edge, it will be reported as four or two contact points.

    // Go through each combination of + and - for each half-size
    static real multiple[8][3] = {
        {1.f, 1.f, 1.f}, {-1.f, 1.f, 1.f}, {1.f, -1.f, 1.f}, {-1.f, -1.f, 1.f},
        {1.f, 1.f, -1.f}, {-1.f, 1.f, -1.f}, {1.f, -1.f, -1.f}, {-1.f, -1.f, -1.f}
    };

    auto contact = data->contacts;

    if (contact == nullptr)
    {
        return 0;
    }

    auto contactsUsed = 0u;

    for (auto i = 0u; i < 8; ++i)
    {
        // Calculate the position of each vertex
        auto vertexPosition = Vector3(multiple[i][0], multiple[i][1], multiple[i][2]);

        vertexPosition *= box.halfSize;

        vertexPosition = box.transform.TransformPosition(vertexPosition);

        // Calculate the distance from the plane
        const auto vertexDistance = vertexPosition | plane.direction;

        // Compare this to the plane's distance
        if (vertexDistance <= plane.offset)
        {
            // Create the contact data.

            // The contact point is halfway between the vertex and the
            // plane - we multiply the direction by half the separation
            // distance and add the vertex location.
            contact->contactPoint = plane.direction;

            contact->contactPoint *= vertexDistance - plane.offset;

            contact->contactPoint += vertexPosition;

            contact->contactNormal = plane.direction;

            contact->penetration = plane.offset - vertexDistance;

            // Write the appropriate data
            contact->SetBodyData(box.body, nullptr, data->friction, data->restitution);

            // Move onto the next contact
            ++contact;

            ++contactsUsed;

            if (contactsUsed == static_cast<unsigned>(data->contactsLeft))
            {
                return contactsUsed;
            }
        }
    }

    data->AddContacts(contactsUsed);

    return contactsUsed;
}

// This preprocessor definition is only used as a convenience
// in the boxAndBox contact generation method.
#define CHECK_OVERLAP(axis, index) \
if (!TryAxis(one, two, axis, toCentre, index, pen, best)) return 0;

unsigned CollisionDetector::BoxAndBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data)
{
    // Make sure we have contacts
    if (data == nullptr || !data->HasMoreContacts())
    {
        return 0;
    }

    // Find the vector between the two centres
    const auto toCentre = two.GetAxis(3) - one.GetAxis(3);

    // We start assuming there is no contact
    auto pen = REAL_MAX;

    auto best = 0xffffffu;

    // Now we check each axes, returning if it gives us
    // a separating axis, and keeping track of the axis with
    // the smallest penetration otherwise.
    CHECK_OVERLAP(one.GetAxis(0), 0);

    CHECK_OVERLAP(one.GetAxis(1), 1);

    CHECK_OVERLAP(one.GetAxis(2), 2);

    CHECK_OVERLAP(two.GetAxis(0), 3);

    CHECK_OVERLAP(two.GetAxis(1), 4);

    CHECK_OVERLAP(two.GetAxis(2), 5);

    // Store the best axis-major, in case we run into almost
    // parallel edge collisions later
    const auto bestSingleAxis = best;

    CHECK_OVERLAP(one.GetAxis(0) ^ two.GetAxis(0), 6);

    CHECK_OVERLAP(one.GetAxis(0) ^ two.GetAxis(1), 7);

    CHECK_OVERLAP(one.GetAxis(0) ^ two.GetAxis(2), 8);

    CHECK_OVERLAP(one.GetAxis(1) ^ two.GetAxis(0), 9);

    CHECK_OVERLAP(one.GetAxis(1) ^ two.GetAxis(1), 10);

    CHECK_OVERLAP(one.GetAxis(1) ^ two.GetAxis(2), 11);

    CHECK_OVERLAP(one.GetAxis(2) ^ two.GetAxis(0), 12);

    CHECK_OVERLAP(one.GetAxis(2) ^ two.GetAxis(1), 13);

    CHECK_OVERLAP(one.GetAxis(2) ^ two.GetAxis(2), 14);

    // Make sure we've got a result.
    assert(best != 0xffffffu);

    // We now know there's a collision, and we know which
    // of the axes gave the smallest penetration. We now
    // can deal with it in different ways depending on
    // the case.
    if (best < 3)
    {
        // We've got a vertex of box two on a face of box one.
        FillPointFaceBoxBox(one, two, toCentre, data, best, pen);

        data->AddContacts(1);

        return 1;
    }

    if (best < 6)
    {
        // We've got a vertex of box one on a face of box two.
        // We use the same algorithm as above, but swap around
        // one and two (and therefore also the vector between their
        // centres).
        FillPointFaceBoxBox(two, one, toCentre * -1.f, data, best - 3, pen);

        data->AddContacts(1);

        return 1;
    }

    // We've got an edge-edge contact. Find out which axes
    best -= 6;

    const auto oneAxisIndex = best / 3;

    const auto twoAxisIndex = best % 3;

    const auto oneAxis = one.GetAxis(oneAxisIndex);

    const auto twoAxis = two.GetAxis(twoAxisIndex);

    auto axis = oneAxis ^ twoAxis;

    axis.Normalize();

    // The axis should point from box one to box two.
    if ((axis | toCentre) > 0)
    {
        axis *= -1.f;
    }

    // We have the axes, but not the edges: each axis has 4 edges parallel
    // to it, we need to find which of the 4 for each object. We do
    // that by finding the point in the centre of the edge. We know
    // its component in the direction of the box's collision axis is zero
    // (its a mid-point) and we determine which of the extremes in each
    // of the other axes is closest.
    auto pointOnOneEdge = one.halfSize;

    auto pointOnTwoEdge = two.halfSize;

    for (auto i = 0u; i < 3; ++i)
    {
        if (i == oneAxisIndex)
        {
            pointOnOneEdge[i] = 0;
        }
        else if ((one.GetAxis(i) | axis) > 0)
        {
            pointOnOneEdge[i] = -pointOnOneEdge[i];
        }

        if (i == twoAxisIndex)
        {
            pointOnTwoEdge[i] = 0;
        }
        else if ((two.GetAxis(i) | axis) < 0)
        {
            pointOnTwoEdge[i] = -pointOnTwoEdge[i];
        }
    }

    // Move them into world coordinates (they are already oriented
    // correctly, since they have been derived from the axes).
    pointOnOneEdge = one.GetTransform() * pointOnOneEdge;

    pointOnTwoEdge = two.GetTransform() * pointOnTwoEdge;

    // So we have a point and a direction for the colliding edges.
    // We need to find out point of closest approach of the two
    // line-segments.
    const auto vertex = ContactPoint(pointOnOneEdge, oneAxis, one.halfSize[oneAxisIndex], pointOnTwoEdge, twoAxis,
                                     two.halfSize[twoAxisIndex], bestSingleAxis > 2);

    // We can fill the contact.
    auto contact = data->contacts;

    if (contact == nullptr)
    {
        return 0;
    }

    contact->penetration = pen;

    contact->contactNormal = axis;

    contact->contactPoint = vertex;

    contact->SetBodyData(one.body, two.body, data->friction, data->restitution);

    data->AddContacts(1);

    return 1;
}

#undef CHECK_OVERLAP

unsigned CollisionDetector::BoxAndPoint(const CollisionBox& box, const Vector3& point, CollisionData* data)
{
    // Make sure we have contacts
    if (data == nullptr || !data->HasMoreContacts())
    {
        return 0;
    }

    // Transform the point into box coordinates
    const auto relPoint = box.GetTransform().InverseTransformVector(point);

    // Check each axis, looking for the axis on which the
    // penetration is least deep.
    auto min_depth = box.halfSize.x - real_abs(relPoint.x);

    if (min_depth < 0)
    {
        return 0;
    }

    auto normal = box.GetAxis(0) * (relPoint.x < 0 ? -1.f : 1.f);

    auto depth = box.halfSize.y - real_abs(relPoint.y);

    if (depth < 0)
    {
        return 0;
    }

    if (depth < min_depth)
    {
        min_depth = depth;

        normal = box.GetAxis(1) * (relPoint.y < 0 ? -1.f : 1.f);
    }

    depth = box.halfSize.z - real_abs(relPoint.z);

    if (depth < 0)
    {
        return 0;
    }

    if (depth < min_depth)
    {
        min_depth = depth;

        normal = box.GetAxis(2) * (relPoint.z < 0 ? -1.f : 1.f);
    }

    // Compile the contact
    auto contact = data->contacts;

    if (contact == nullptr)
    {
        return 0;
    }

    contact->contactNormal = normal;

    contact->contactPoint = point;

    contact->penetration = min_depth;

    // Note that we don't know what rigid body the point
    // belongs to, so we just use NULL. Where this is called
    // this value can be left, or filled in.
    contact->SetBodyData(box.body, nullptr, data->friction, data->restitution);

    data->AddContacts(1);

    return 1;
}

unsigned CollisionDetector::BoxAndSphere(const CollisionBox& box, const CollisionSphere& sphere, CollisionData* data)
{
    // Make sure we have contacts
    if (data == nullptr || !data->HasMoreContacts())
    {
        return 0;
    }

    // Transform the centre of the sphere into box coordinates
    const auto centre = sphere.GetAxis(3);

    auto relCentre = box.transform.InverseTransformVector(centre);

    // Early out check to see if we can exclude the contact
    if (real_abs(relCentre.x) - sphere.radius > box.halfSize.x ||
        real_abs(relCentre.y) - sphere.radius > box.halfSize.y ||
        real_abs(relCentre.z) - sphere.radius > box.halfSize.z)
    {
        return 0;
    }

    Vector3 closestPoint;

    // Clamp each coordinate to the box.
    auto distance = relCentre.x;

    if (distance > box.halfSize.x)
    {
        distance = box.halfSize.x;
    }

    if (distance < -box.halfSize.x)
    {
        distance = -box.halfSize.x;
    }

    closestPoint.x = distance;

    distance = relCentre.y;

    if (distance > box.halfSize.y)
    {
        distance = box.halfSize.y;
    }

    if (distance < -box.halfSize.y)
    {
        distance = -box.halfSize.y;
    }

    closestPoint.y = distance;

    distance = relCentre.z;

    if (distance > box.halfSize.z)
    {
        distance = box.halfSize.z;
    }

    if (distance < -box.halfSize.z)
    {
        distance = -box.halfSize.z;
    }

    closestPoint.z = distance;

    // Check we're in contact
    distance = (closestPoint - relCentre).SizeSquared();

    if (distance > sphere.radius * sphere.radius)
    {
        return 0;
    }

    // Compile the contact
    const auto closestPointWorld = box.GetTransform().TransformPosition(closestPoint);

    auto contact = data->contacts;

    if (contact == nullptr)
    {
        return 0;
    }

    contact->contactNormal = closestPointWorld - centre;

    contact->contactNormal.Normalize();

    contact->contactPoint = closestPointWorld;

    contact->penetration = sphere.radius - real_sqrt(distance);

    contact->SetBodyData(box.body, sphere.body, data->friction, data->restitution);

    data->AddContacts(1);

    return 1;
}
