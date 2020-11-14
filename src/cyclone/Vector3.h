#pragma once

#include "Precision.h"

namespace cyclone
{
    /**
    * Holds a vector in 3 dimensions. Four data members are allocated
    * to ensure alignment in an array.
    */
    class Vector3
    {
        typedef unsigned size_t;

    public:
        /** Holds the value along the x axis. */
        real x;

        /** Holds the value along the y axis. */
        real y;

        /** Holds the value along the z axis. */
        real z;

    private:
        /** Padding to ensure 4 word alignment. */
        real pad;

    public:
        const static Vector3 Zero;

        const static Vector3 One;

        const static Vector3 Up;

        const static Vector3 Right;

        const static Vector3 Gravity;

    public:
        /**
        * The default constructor creates a zero vector.
        */
        Vector3();

        /**
        * Constructor initializing all components to
        * a single float value.
        */
        explicit Vector3(real v);

        /**
        * Constructor using initial values for each component.
        */
        Vector3(real x, real y, real z);

        /**
        * Constructor using the XYZ components from a 4D vector.
        */
        Vector3(const class Vector4& v);

        /**
        * Constructs a vector from a vector.
        */
        Vector3(const Vector3& v);

        /**
        * Calculate cross product between this and another vector.
        */
        Vector3 operator^(const Vector3& v) const;

        /**
        * Calculate the cross product of two vectors.
        */
        static Vector3 CrossProduct(const Vector3& a, const Vector3& b);

        /**
        * Calculate the dot product between this and another vector.
        */
        real operator|(const Vector3& v) const;

        /**
        * Calculate the dot product of two vectors.
        */
        static real DotProduct(const Vector3& a, const Vector3& b);

        /**
        * Gets the result of component-wise addition of this and another vector.
        */
        Vector3 operator+(const Vector3& v) const;

        /**
        * Gets the result of component-wise subtraction of this by another vector.
        */
        Vector3 operator-(const Vector3& v) const;

        /**
        * Gets the result of adding to each component of the vector.
        */
        Vector3 operator+(real bias) const;

        /**
        * Gets the result of subtracting from each component of the vector.
        */
        Vector3 operator-(real bias) const;

        /**
        * Gets the result of scaling the vector (multiplying each
        * component by a value).
        */
        Vector3 operator*(real scale) const;

        /**
        * Gets the result of dividing each component of the vector
        * by a value.
        */
        Vector3 operator/(real scale) const;

        /**
        * Gets the result of component-wise multiplication
        * of this vector by another.
        */
        Vector3 operator*(const Vector3& v) const;

        /**
        * Gets the result of component-wise division
        * of this vector by another.
        */
        Vector3 operator/(const Vector3& v) const;

        /**
        * Check against another vector for equality.
        */
        bool operator==(const Vector3& v) const;

        /**
        * Check against another vector for inequality.
        */
        bool operator!=(const Vector3& v) const;

        /**
        * Get a negated copy of the vector.
        */
        Vector3 operator-() const;

        /**
        * Adds another vector to this.
        * Uses component-wise addition.
        */
        void operator+=(const Vector3& v);

        /**
        * Subtracts another vector from this.
        * Uses component-wise subtraction.
        */
        void operator-=(const Vector3& v);

        /**
        * Scales the vector.
        */
        void operator*=(real scale);

        /**
        * Divides the vector by a number.
        */
        void operator/=(real scale);

        /**
        * Multiplies the vector with another vector,
        * using component-wise multiplication.
        */
        Vector3 operator*=(const Vector3& v);

        /**
        * Divides the vector by another vector,
        * using component-wise division.
        */
        Vector3 operator/=(const Vector3& v);

        /**
        * Gets specific component of the vector.
        */
        real& operator[](size_t index);

        /**
        * Gets specific component of the vector.
        */
        real operator[](size_t index) const;

        /**
        * Get the length (magnitude) of this vector.
        */
        real Size() const;

        /**
        * Get the squared length of this vector.
        */
        real SizeSquared() const;

        /**
        * Turns a non-zero vector into a vector of unit length.
        */
        void Normalize();

        /**
        * Limits the size of the vector to the given maximum.
        */
        void Trim(real size);

        /**
        * Returns the normalised version of a vector.
        */
        Vector3 Unit() const;

        /**
        * Zero all the components of the vector.
        */
        void Reset();

        /**
        * Euclidean distance between two points.
        */
        static real Distance(const Vector3& a, const Vector3& b);

        /**
        * Squared distance between two points.
        */
        static real DistSquared(const Vector3& a, const Vector3& b);

        /**
        * Triple product of three vectors: X dot (Y cross Z).
        */
        static real Triple(const Vector3& a, const Vector3& b, const Vector3& c);

        /**
        * Gets the result of scaling the vector (multiplying each
        * component by a value).
        */
        friend Vector3 operator*(real s, const Vector3& u);
    };
};
