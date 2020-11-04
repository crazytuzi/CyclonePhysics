#include "Quaternion.h"

#include <cfloat>
#include <cmath>

using namespace cyclone;

Quaternion::Quaternion(): i(0), j(0), k(0), a(1)
{
}

Quaternion::Quaternion(const real i, const real j, const real k, const real a): i(i), j(j), k(k), a(a)
{
}

Quaternion Quaternion::operator+(const Quaternion& q) const
{
    return Quaternion(i + q.i, j + q.j, k + q.k, a + q.a);
}

Quaternion Quaternion::operator+=(const Quaternion& q)
{
    i += q.i;

    j += q.j;

    k += q.k;

    a += q.a;

    return *this;
}

Quaternion Quaternion::operator-(const Quaternion& q) const
{
    return Quaternion(i - q.i, j - q.j, k - q.k, a - q.a);
}

Quaternion Quaternion::operator-=(const Quaternion& q)
{
    i -= q.i;

    j -= q.j;

    k -= q.k;

    a -= q.a;

    return *this;
}

Quaternion Quaternion::operator*(const Quaternion& q) const
{
    return Quaternion(i * q.a + a * q.i - k * q.j + j * q.k,
                      j * q.a + k * q.i + a * q.j - i * q.k,
                      k * q.a - j * q.i + i * q.j + a * q.k,
                      a * q.a - i * q.i - j * q.j - k * q.k);
}

Quaternion Quaternion::operator*=(const Quaternion& q)
{
    const auto temp_i = i;

    const auto temp_j = j;

    const auto temp_k = k;

    const auto temp_a = a;

    i = temp_i * q.a + temp_a * q.i - temp_k * q.j + temp_j * q.k;

    j = temp_j * q.a + temp_k * q.i + temp_a * q.j - temp_i * q.k;

    k = temp_k * q.a - temp_j * q.i + temp_i * q.j + temp_a * q.k;

    a = temp_a * q.a - temp_i * q.i - temp_j * q.j - temp_k * q.k;

    return *this;
}

Vector3 Quaternion::operator*(const Vector3& v) const
{
    return RotateVector(v);
}

Quaternion Quaternion::operator*=(const real scale)
{
    i *= scale;

    j *= scale;

    k *= scale;

    a *= scale;

    return *this;
}

Quaternion Quaternion::operator*(const real scale) const
{
    return Quaternion(i * scale, j * scale, k * scale, a * scale);
}

Quaternion Quaternion::operator/=(const real scale)
{
    const auto reciprocal = 1.f / scale;

    i *= reciprocal;

    j *= reciprocal;

    k *= reciprocal;

    a *= reciprocal;

    return *this;
}

Quaternion Quaternion::operator/(const real scale) const
{
    const auto reciprocal = 1.f / scale;

    return Quaternion(i * reciprocal, j * reciprocal, k * reciprocal, a * reciprocal);
}

bool Quaternion::operator=(const Quaternion& q) const
{
    return i == q.i && j == q.j && k == q.k && a == q.a;
}

bool Quaternion::operator!=(const Quaternion& q) const
{
    return i != q.i || j != q.j || k != q.k || a != q.a;
}

real Quaternion::operator|(const Quaternion& q) const
{
    return i * q.i + j * q.j + k * q.k + a * q.a;
}

Quaternion Quaternion::MakeFromEuler(const Vector3& v)
{
    return MakeFromEuler(v.x, v.y, v.z);
}

Quaternion Quaternion::MakeFromEuler(const real x, const real y, const real z)
{
    const auto roll = DegreesToRadians(x);

    const auto pitch = DegreesToRadians(y);

    const auto yaw = DegreesToRadians(z);

    const auto cyaw = real_cos(0.5f * yaw);

    const auto cpitch = real_cos(0.5f * pitch);

    const auto croll = real_cos(0.5f * roll);

    const auto syaw = real_sin(0.5f * yaw);

    const auto spitch = real_sin(0.5f * pitch);

    const auto sroll = real_sin(0.5f * roll);

    const auto cyawcpitch = cyaw * cpitch;

    const auto syawspitch = syaw * spitch;

    const auto cyawspitch = cyaw * spitch;

    const auto syawcpitch = syaw * cpitch;

    return Quaternion(cyawcpitch * croll + syawspitch * sroll,
                      cyawcpitch * sroll - syawspitch * croll,
                      cyawspitch * croll + syawcpitch * sroll,
                      syawcpitch * croll - cyawspitch * sroll);
}

Vector3 Quaternion::Euler() const
{
    const auto q00 = a * a;

    const auto q11 = i * i;

    const auto q22 = j * j;

    const auto q33 = k * k;

    const auto r11 = q00 + q11 - q22 - q33;

    const auto r21 = 2 * (i * j + a * k);

    const auto r31 = 2 * (i * k - a * j);

    const auto r32 = 2 * (j * k + a * j);

    const auto r33 = q00 - q11 - q22 + q33;

    if (real_abs(r31) > 0.999999)
    {
        const auto r12 = 2 * (i * j - a * k);

        const auto r13 = 2 * (i * k + a * j);

        return Vector3(RadiansToDegrees(0.0f),
                       RadiansToDegrees(-(R_PI / 2) * r31 / fabs(r31)),
                       RadiansToDegrees(atan2(-r12, -r31 * r13)));
    }

    return Vector3(RadiansToDegrees(atan2(r32, r33)),
                   RadiansToDegrees(asin(-r31)),
                   RadiansToDegrees(atan2(r21, r11)));
}

void Quaternion::Normalize()
{
    auto d = a * a + i * i + j * j + k * k;

    // Check for zero length quaternion, and use the no-rotation
    // quaternion in that case.
    if (d < real_epsilon)
    {
        a = 1;

        return;
    }

    d = static_cast<real>(1.f) / real_sqrt(d);

    i *= d;

    j *= d;

    k *= d;

    a *= d;
}

real Quaternion::Size() const
{
    return real_sqrt(i * i + j * j + k * k + a * a);
}

real Quaternion::SizeSquared() const
{
    return i * i + j * j + k * k + a * a;
}

Vector3 Quaternion::RotateVector(const Vector3& v) const
{
    const Vector3 q(i, j, k);

    const auto t = 2.f * Vector3::CrossProduct(q, v);

    return v + a * t + Vector3::CrossProduct(q, t);
}

Vector3 Quaternion::UnrotateVector(const Vector3& v) const
{
    // Inverse
    const Vector3 q(-i, -j, -k);

    const auto t = 2.f * Vector3::CrossProduct(q, v);

    return v + a * t + Vector3::CrossProduct(q, t);
}

Quaternion Quaternion::Inverse() const
{
    return Quaternion(-i, -j, -k, a);
}

real Quaternion::DegreesToRadians(const real deg)
{
    return deg * R_PI / 180.f;
}

real Quaternion::RadiansToDegrees(const real rad)
{
    return rad * 180.f / R_PI;
}
