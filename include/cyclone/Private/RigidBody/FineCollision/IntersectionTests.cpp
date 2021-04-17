#include "RigidBody/FineCollision/IntersectionTests.h"
#include <cmath>

using namespace cyclone;

static real TransformToAxis(const CollisionBox& box, const Vector3& axis)
{
    return box.halfSize.x * real_abs(axis | box.GetAxis(0)) + box.halfSize.y * real_abs(axis | box.GetAxis(1)) + box.
        halfSize.z * real_abs(axis | box.GetAxis(2));
}

/**
* This function checks if the two boxes overlap
* along the given axis. The final parameter toCentre
* is used to pass in the vector between the boxes centre
* points, to avoid having to recalculate it each time.
*/
static bool OverlapOnAxis(const CollisionBox& one, const CollisionBox& two, const Vector3& axis,
                          const Vector3& toCentre)
{
    // Project the half-size of one onto axis
    const auto oneProject = TransformToAxis(one, axis);

    const auto twoProject = TransformToAxis(two, axis);

    // Project this onto the axis
    const auto distance = real_abs(toCentre | axis);

    // Check for overlap
    return distance < oneProject + twoProject;
}

// This preprocessor definition is only used as a convenience
// in the boxAndBox intersection  method.
#define TEST_OVERLAP(axis) OverlapOnAxis(one, two, axis, toCentre)

bool IntersectionTests::SphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane)
{
    // Find the distance from the origin
    const auto ballDistance = plane.direction | sphere.GetAxis(3) - sphere.radius;

    // Check for the intersection
    return ballDistance <= plane.offset;
}

bool IntersectionTests::SphereAndSphere(const CollisionSphere& one, const CollisionSphere& two)
{
    // Find the vector between the objects
    const auto midLine = one.GetAxis(3) - two.GetAxis(3);

    // See if it is large enough.
    return midLine.SizeSquared() < (one.radius + two.radius) * (one.radius + two.radius);
}

bool IntersectionTests::BoxAndBox(const CollisionBox& one, const CollisionBox& two)
{
    // Find the vector between the two centres
    const auto toCentre = two.GetAxis(3) - one.GetAxis(3);

    return
        // Check on box one's axes first
        TEST_OVERLAP(one.GetAxis(0)) &&

        TEST_OVERLAP(one.GetAxis(1)) &&

        TEST_OVERLAP(one.GetAxis(2)) &&

        // And on two's
        TEST_OVERLAP(two.GetAxis(0)) &&

        TEST_OVERLAP(two.GetAxis(1)) &&

        TEST_OVERLAP(two.GetAxis(2)) &&

        // Now on the cross products
        TEST_OVERLAP(one.GetAxis(0) ^ two.GetAxis(0)) &&

        TEST_OVERLAP(one.GetAxis(0) ^ two.GetAxis(1)) &&

        TEST_OVERLAP(one.GetAxis(0) ^ two.GetAxis(2)) &&

        TEST_OVERLAP(one.GetAxis(1) ^ two.GetAxis(0)) &&

        TEST_OVERLAP(one.GetAxis(1) ^ two.GetAxis(1)) &&

        TEST_OVERLAP(one.GetAxis(1) ^ two.GetAxis(2)) &&

        TEST_OVERLAP(one.GetAxis(2) ^ two.GetAxis(0)) &&

        TEST_OVERLAP(one.GetAxis(2) ^ two.GetAxis(1)) &&

        TEST_OVERLAP(one.GetAxis(2) ^ two.GetAxis(2));
}

#undef TEST_OVERLAP

bool IntersectionTests::BoxAndHalfSpace(const CollisionBox& box, const CollisionPlane& plane)
{
    // Work out the projected radius of the box onto the plane direction
    const auto projectedRadius = TransformToAxis(box, plane.direction);

    // Work out how far the box is from the origin
    const auto boxDistance = plane.direction | box.GetAxis(3) - projectedRadius;

    // Check for the intersection
    return boxDistance <= plane.offset;
}
