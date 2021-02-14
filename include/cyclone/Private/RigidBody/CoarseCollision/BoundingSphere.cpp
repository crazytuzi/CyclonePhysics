#include "RigidBody/CoarseCollision/BoundingSphere.h"
#include <cmath>

using namespace cyclone;

BoundingSphere::BoundingSphere(const Vector3& centre, const real radius): centre(centre), radius(radius)
{
}

BoundingSphere::BoundingSphere(const BoundingSphere& one, const BoundingSphere& another)
{
    const auto centerOffset = another.centre - one.centre;

    auto distance = centerOffset.SizeSquared();

    const auto radiusDiff = another.radius - one.radius;

    // Check if the larger sphere encloses the small one
    if (radiusDiff * radiusDiff >= distance)
    {
        if (one.radius > another.radius)
        {
            centre = one.centre;

            radius = one.radius;
        }
        else
        {
            centre = another.centre;

            radius = another.radius;
        }
    }
    else // Otherwise we need to work with partially overlapping spheres
    {
        distance = real_sqrt(distance);

        radius = (distance + one.radius + another.radius) * 0.5f;

        // The new centre is based on one's centre, moved towards two's centre
        // by an amount proportional to the spheres' radius.
        centre = one.centre;

        if (distance > 0)
        {
            centre += centerOffset * ((radius - one.radius) / distance);
        }
    }
}

bool BoundingSphere::Overlaps(const BoundingSphere& other) const
{
    const auto distanceSquared = (centre - other.centre).SizeSquared();

    return distanceSquared < (radius + other.radius) * (radius + other.radius);
}

real BoundingSphere::GetGrowth(const BoundingSphere& other) const
{
    const BoundingSphere sphere(*this, other);

    // We return a value proportional to the change in surface area of the sphere.
    return sphere.radius * sphere.radius - radius * radius;
}

real BoundingSphere::Size() const
{
    return 1.333333f * R_PI * radius * radius * radius;
}
