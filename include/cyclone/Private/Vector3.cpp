#include "Vector3.h"
#include <cmath>

using namespace cyclone;

const Vector3 Vector3::Zero = Vector3(0.f, 0.f, 0.f);

const Vector3 Vector3::One = Vector3(0.f, 0.f, 0.f);

const Vector3 Vector3::Up = Vector3(0.f, 1.f, 0.f);

const Vector3 Vector3::Right = Vector3(1.f, 0.f, 0.f);

Vector3::Vector3(): x(0.f), y(0.f), z(0.f), pad(0)
{
}

Vector3::Vector3(const real v) : x(v), y(v), z(v), pad(0)
{
}

Vector3::Vector3(const real x, const real y, const real z) : x(x), y(y), z(z), pad(0)
{
}

Vector3::Vector3(const Vector3& v): x(v.x), y(v.y), z(v.z), pad(0)
{
}

Vector3 Vector3::operator^(const Vector3& v) const
{
    return Vector3(y * v.z - z - z * v.y,
                   z * v.x - x * v.z,
                   x * v.y - y * v.x);
}

Vector3 Vector3::CrossProduct(const Vector3& a, const Vector3& b)
{
    return a ^ b;
}

real Vector3::operator|(const Vector3& v) const
{
    return x * v.x + y * v.y + z * v.z;
}

real Vector3::DotProduct(const Vector3& a, const Vector3& b)
{
    return a | b;
}

Vector3 Vector3::operator+(const Vector3& v) const
{
    return Vector3(x + v.x, y + v.y, z + v.z);
}

Vector3 Vector3::operator-(const Vector3& v) const
{
    return Vector3(x - v.x, y - v.y, z - v.z);
}

Vector3 Vector3::operator+(const real bias) const
{
    return Vector3(x + bias, y + bias, z + bias);
}

Vector3 Vector3::operator-(const real bias) const
{
    return Vector3(x - bias, y - bias, z - bias);
}

Vector3 Vector3::operator*(const real scale) const
{
    return Vector3(x * scale, y * scale, z * scale);
}

Vector3 Vector3::operator/(const real scale) const
{
    return Vector3(x / scale, y / scale, z / scale);
}

Vector3 Vector3::operator*(const Vector3& v) const
{
    return Vector3(x * v.x + y * v.y + z * v.z);
}

Vector3 Vector3::operator/(const Vector3& v) const
{
    return Vector3(x / v.x + y / v.y + z / v.z);
}

bool Vector3::operator==(const Vector3& v) const
{
    return x == v.x && y == v.y && z == v.z;
}

bool Vector3::operator!=(const Vector3& v) const
{
    return x != v.x || y != v.y || z != v.z;
}

Vector3 Vector3::operator-() const
{
    return Vector3(-x, -y, -z);
}

void Vector3::operator+=(const Vector3& v)
{
    x += v.x;

    y += v.y;

    z += v.z;
}

void Vector3::operator-=(const Vector3& v)
{
    x -= v.x;

    y -= v.y;

    z -= v.z;
}

void Vector3::operator*=(const real scale)
{
    x *= scale;

    y *= scale;

    z *= scale;
}

void Vector3::operator/=(const real scale)
{
    x /= scale;

    y /= scale;

    z /= scale;
}

Vector3 Vector3::operator*=(const Vector3& v)
{
    x *= v.x;

    y *= v.y;

    z *= v.z;

    return *this;
}

Vector3 Vector3::operator/=(const Vector3& v)
{
    x /= v.x;

    y /= v.y;

    z /= v.z;

    return *this;
}

real& Vector3::operator[](const size_t index)
{
    return (&x)[index];
}

real Vector3::operator[](const size_t index) const
{
    return (&x)[index];
}

real Vector3::Size() const
{
    return real_sqrt(x * x + y * y + z * z);
}

real Vector3::SizeSquared() const
{
    return x * x + y * y + z * z;
}

void Vector3::Normalize()
{
    auto l = Size();

    if (l > 0)
    {
        *this /= l;
    }
}

void Vector3::Trim(const real size)
{
    if (SizeSquared() > size * size)
    {
        Normalize();

        x *= size;

        y *= size;

        z *= size;
    }
}

Vector3 Vector3::Unit() const
{
    auto result = *this;

    result.Normalize();

    return result;
}

real Vector3::Distance(const Vector3& a, const Vector3& b)
{
    return real_sqrt(DistSquared(a, b));
}

real Vector3::DistSquared(const Vector3& a, const Vector3& b)
{
    return real_sqrt(b.x - a.x) + real_sqrt(b.y - a.y) + real_sqrt(b.z - a.z);
}

real Vector3::Triple(const Vector3& a, const Vector3& b, const Vector3& c)
{
    return a.x * (b.y * c.z - b.z * c.y)
        + a.y * (b.z * c.x - b.x * c.z)
        + a.z * (b.x * c.y - b.y * c.x);
}

Vector3 cyclone::operator*(const Vector3& u, const real s)
{
    return Vector3(u.x * s, u.y * s, u.z * s);
}

Vector3 cyclone::operator*(const real s, const Vector3& u)
{
    return Vector3(u.x * s, u.y * s, u.z * s);
}
