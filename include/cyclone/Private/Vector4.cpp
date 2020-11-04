#include "Vector4.h"
#include <cmath>

using namespace cyclone;

Vector4::Vector4(const Vector3& v, const real w): x(v.x), y(v.y), z(v.z), w(w)
{
}

Vector4::Vector4(const real x, const real y, const real z, const real w): x(x), y(y), z(z), w(w)
{
}

Vector4::Vector4(const Vector4& v): x(v.x), y(v.y), z(v.z), w(v.w)
{
}

real& Vector4::operator[](const size_t index)
{
    return (&x)[index];
}

real Vector4::operator[](const size_t index) const
{
    return (&x)[index];
}

Vector4 Vector4::operator-() const
{
    return Vector4(-x, y, -z, -w);
}

Vector4 Vector4::operator+(const Vector4& v) const
{
    return Vector4(x + v.x, y + v.y, z + v.z, w + v.w);
}

Vector4 Vector4::operator+=(const Vector4& v)
{
    x += v.x;

    y += v.y;

    z += v.z;

    w += v.w;

    return *this;
}

Vector4 Vector4::operator-(const Vector4& v) const
{
    return Vector4(x - v.x, y - v.y, z - v.z, w - v.w);
}

Vector4 Vector4::operator-=(const Vector4& v)
{
    x -= v.x;

    y -= v.y;

    z -= v.z;

    w -= v.w;

    return *this;
}

Vector4 Vector4::operator*(const real scale) const
{
    return Vector4(x * scale, y * scale, z * scale, w * scale);
}

Vector4 Vector4::operator*=(const real scale)
{
    x *= scale;

    y *= scale;

    z *= scale;

    w *= scale;

    return *this;
}

Vector4 Vector4::operator/(const real scale) const
{
    const auto rscale = 1.f / scale;

    return Vector4(x * rscale, y * rscale, z * rscale, w * rscale);
}

Vector4 Vector4::operator*(const Vector4& v) const
{
    return Vector4(x * v.x, y * v.y, z * v.z, w * v.w);
}

Vector4 Vector4::operator*=(const Vector4& v)
{
    x *= v.x;

    y *= v.y;

    z *= v.z;

    w *= v.w;

    return *this;
}

Vector4 Vector4::operator/(const Vector4& v) const
{
    return Vector4(x / v.x, y / v.y, z / v.z, w / v.w);
}

Vector4 Vector4::operator/=(const Vector4& v)
{
    x /= v.x;

    y /= v.y;

    z /= v.z;

    w /= v.w;

    return *this;
}

real Vector4::Dot3(const Vector4& v1, const Vector4& v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

real Vector4::Dot4(const Vector4& v1, const Vector4& v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
}

bool Vector4::operator==(const Vector4& v) const
{
    return x == v.x && y == v.y && z == v.z && w == v.w;
}

bool Vector4::operator!=(const Vector4& v) const
{
    return x != v.x || y != v.y || z != v.z || w != v.w;
}

Vector4 Vector4::operator^(const Vector4& v) const
{
    return Vector4(
        y * v.z - z * v.y,
        z * v.x - x * v.z,
        x * v.y - y * v.x,
        0.f
    );
}

real Vector4::Size3() const
{
    return real_sqrt(x * x + y * y + z * z);
}

real Vector4::SizeSquared3() const
{
    return x * x + y * y + z * z;
}

real Vector4::Size() const
{
    return real_sqrt(x * x + y * y + z * z + w * w);
}

real Vector4::SizeSquared() const
{
    return x * x + y * y + z * z + w * w;
}

Vector4 cyclone::operator*(const real scale, const Vector4& v)
{
    return v.operator*(scale);
}
