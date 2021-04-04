#pragma once

#include "Precision.h"
#include "Vector3.h"

namespace cyclone
{
    class Vector4
    {
        typedef unsigned size_t;

    public:
        /** Holds the value along the x axis. */
        real x;

        /** Holds the value along the y axis. */
        real y;

        /** Holds the value along the z axis. */
        real z;

        /** Holds the value along the w axis. */
        real w;

    public:
        /**
        * Constructor.
        */
        explicit Vector4(const Vector3& v, real w = 0.f);

        /**
        * Creates and initializes a new vector
        * from the specified components.
        */
        explicit Vector4(real x = 0.f, real y = 0.f, real z = 0.f, real w = 0.f);

        /**
        * Creates and initializes a new vector
        * from an int vector value.
        */
        Vector4(const Vector4& v);

        /**
        * Access a specific component of the vector.
        */
        real& operator[](size_t index);

        /**
        * Access a specific component of the vector.
        */
        real operator[](size_t index) const;

        /**
        * Gets a negated copy of the vector.
        */
        Vector4 operator-() const;

        /**
        * Gets the result of adding a vector to this.
        */
        Vector4 operator+(const Vector4& v) const;

        /**
        * Adds another vector to this one.
        */
        Vector4 operator+=(const Vector4& v);

        /**
        * Gets the result of subtracting a vector from this.
        */
        Vector4 operator-(const Vector4& v) const;

        /**
        * Subtracts another vector to this one.
        */
        Vector4 operator-=(const Vector4& v);

        /**
        * Gets the result of scaling this vector.
        */
        Vector4 operator*(real scale) const;

        /**
        * Gets the result of scaling this vector.
        */
        Vector4 operator*=(real scale);

        /**
        * Gets the result of dividing this vector.
        */
        Vector4 operator/(real scale) const;

        /**
        * Gets the result of multiplying a vector with this.
        */
        Vector4 operator*(const Vector4& v) const;

        /**
        * Gets the result of multiplying a vector
        * with another Vector (component wise).
        */
        Vector4 operator*=(const Vector4& v);

        /**
        * Gets the result of dividing this vector.
        */
        Vector4 operator/(const Vector4& v) const;

        /**
        * Gets the result of dividing a vector
        * with another Vector (component wise).
        */
        Vector4 operator/=(const Vector4& v);

        /**
        * Calculates 3D Dot product of two 4D vectors.
        */
        static real Dot3(const Vector4& v1, const Vector4& v2);

        /**
        * Calculates 4D Dot product.
        */
        static real Dot4(const Vector4& v1, const Vector4& v2);

        /**
        * Checks for equality against another vector.
        */
        bool operator==(const Vector4& v) const;

        /**
        * Checks for inequality against another vector.
        */
        bool operator!=(const Vector4& v) const;

        /**
        * Calculate Cross product between this and another vector.
        */
        Vector4 operator^(const Vector4& v) const;

        /**
        * Get the length of this vector
        * not taking W component into account.
        */
        real Size3() const;

        /**
        * Get the squared length of this vector
        * not taking W component into account.
        */
        real SizeSquared3() const;

        /**
        * Get the length (magnitude) of this vector,
        * taking the W component into account
        */
        real Size() const;

        /**
        * Get the squared length of this vector,
        * taking the W component into account
        */
        real SizeSquared() const;

        /**
        * Scales a vector.
        */
        friend Vector4 operator*(real scale, const Vector4& v);
    };
}
