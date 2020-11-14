#pragma once

#include "Precision.h"
#include "Vector3.h"

namespace cyclone
{
    /**
    * Holds a three degree of freedom orientation.
    *
    * Quaternions have
    * several mathematical properties that make them useful for
    * representing orientations, but require four items of data to
    * hold the three degrees of freedom. These four items of data can
    * be viewed as the coefficients of a complex number with three
    * imaginary parts. The mathematics of the quaternion is then
    * defined and is roughly correspondent to the math of 3D
    * rotations. A quaternion is only a valid rotation if it is
    * normalised: i.e. it has a length of 1.
    *
    * @note Angular velocity and acceleration can be correctly
    * represented as vectors. Quaternions are only needed for
    * orientation.
    */
    class Quaternion
    {
    public:
        /**
        * Holds the real component of the quaternion.
        */
        real i;

        /**
        * Holds the second complex component of the
        * quaternion.
        */
        real j;
        /**
        * Holds the third complex component of the
        * quaternion.
        */

        real k;

        /**
        * Holds the real component of the quaternion.
        */
        real a;

    public:
        /**
        * Default constructor (no initialization).
        */
        Quaternion();

        /**
        * Constructor.
        */
        explicit Quaternion(real i, real j, real k, real a);

        /**
        * Gets the result of adding a Quaternion to this.
        * This is a component-wise addition;
        * composing quaternions should be done via multiplication.
        */
        Quaternion operator+(const Quaternion& q) const;

        /**
        * Adds to this quaternion.
        * This is a component-wise addition;
        * composing quaternions should be done via multiplication.
        */
        Quaternion operator+=(const Quaternion& q);

        /**
        * Gets the result of subtracting a Quaternion to this.
        * This is a component-wise subtraction;
        * composing quaternions should be done via multiplication.
        */
        Quaternion operator-(const Quaternion& q) const;

        /**
        * Subtracts another quaternion from this.
        * This is a component-wise subtraction;
        * composing quaternions should be done via multiplication.
        */
        Quaternion operator-=(const Quaternion& q);

        /**
        * Gets the result of multiplying this by another quaternion (this * Q).
        * Order matters when composing quaternions: C = A * B
        * will yield a quaternion C that logically
        * first applies B then A to any subsequent
        * transformation (right first, then left).
        */
        Quaternion operator*(const Quaternion& q) const;

        /**
        * Multiply this by a quaternion (this = this * Q).
        * Order matters when composing quaternions: C = A * B
        * will yield a quaternion C that logically
        * first applies B then A to any subsequent
        * transformation (right first, then left).
        */
        Quaternion operator*=(const Quaternion& q);

        /**
        * Rotate a vector by this quaternion.
        */
        Vector3 operator*(const Vector3& v) const;

        /**
        * Multiply this quaternion by a scaling factor.
        */
        Quaternion operator*=(real scale);

        /**
        * Get the result of scaling this quaternion.
        */
        Quaternion operator*(real scale) const;

        /**
        * Divide this quaternion by scale.
        */
        Quaternion operator/=(real scale);

        /**
        * Divide this quaternion by scale.
        */
        Quaternion operator/(real scale) const;

        /**
        * Checks whether two quaternions are identical.
        */
        bool operator=(const Quaternion& q) const;

        /**
        * Checks whether two quaternions are not identical.
        */
        bool operator!=(const Quaternion& q) const;

        /**
        * Calculates dot product of two quaternions.
        */
        real operator|(const Quaternion& q) const;

        /**
        * Convert a vector of floating-point Euler angles
        * (in degrees) into a Quaternion.
        */
        static Quaternion MakeFromEuler(const Vector3& v);

        /**
        * Convert a point Euler angles
        * (in degrees) into a Quaternion.
        */
        static Quaternion MakeFromEuler(real x, real y, real z);

        /**
        * Convert a Quaternion into floating-point Euler angles (in degrees).
        */
        Vector3 Euler() const;

        /**
        * Normalises the quaternion to unit length, making it a valid
        * orientation quaternion.
        */
        void Normalize();

        /**
        * Get the length of this quaternion.
        */
        real Size() const;

        /**
        * Get the length squared of this quaternion.
        */
        real SizeSquared() const;

        /**
        * Rotate a vector by this quaternion.
        */
        Vector3 RotateVector(const Vector3& v) const;

        /**
        * Rotate a vector by the inverse of this quaternion.
        */
        Vector3 UnrotateVector(const Vector3& v) const;

        /**
        * Inverse of this quaternion.
        */
        Quaternion Inverse() const;

    private:
        static real DegreesToRadians(real deg);

        static real RadiansToDegrees(real rad);
    };
}
