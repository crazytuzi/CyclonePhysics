#pragma once

#include "Precision.h"
#include "Vector3.h"

namespace cyclone
{
    /**
    * 4x4 matrix of floating point values.
    * Matrix-matrix multiplication happens with a pre-multiple of the transpose
    * in other words, Res = Mat1.operator*(Mat2) means Res = Mat2^T * Mat1, as
    * opposed to Res = Mat1 * Mat2.
    * Matrix elements are accessed with M[RowIndex][ColumnIndex].
    */
    class Matrix
    {
    public:
        /**
        * Holds the transform matrix data in array form.
        */
        real M[4][4];

    public:
        /**
        * Constructors.
        */
        Matrix(const Vector3& InX, const Vector3& InY, const Vector3& InZ, const Vector3& InW = Vector3::Zero);

        /**
        * Constructors.
        */
        Matrix(real x0 = 0.f, real x1 = 0.f, real x2 = 0.f, real x3 = 0.f,
               real y0 = 0.f, real y1 = 0.f, real y2 = 0.f, real y3 = 0.f,
               real z0 = 0.f, real z1 = 0.f, real z2 = 0.f, real z3 = 0.f,
               real w0 = 0.f, real w1 = 0.f, real w2 = 0.f, real w3 = 1.f);

        /**
        * Set this to the identity matrix
        */
        void SetIdentity();

        /**
        * Gets the result of multiplying a Matrix to this.
        */
        Matrix operator*(const Matrix& m) const;

        /**
        * Multiply this by a matrix.
        */
        void operator*=(const Matrix& m);

        /**
        * Gets the result of adding a matrix to this.
        */
        Matrix operator+(const Matrix& m) const;

        /**
        * Adds to this matrix.
        */
        void operator+=(const Matrix& m);

        /**
        * This isn't applying SCALE,
        * just multiplying the value to all members
        */
        Matrix operator*(float scale);

        /**
        * Multiply this matrix by a weighting factor.
        */
        void operator*=(float scale);

        /**
        * Checks whether two matrix are identical.
        */
        bool operator==(const Matrix& m) const;

        /**
        * Checks whether another Matrix is not equal to this,
        * within specified tolerance.
        */
        bool operator!=(const Matrix& m) const;

        /**
        * Transform the given vector by this matrix.
        */
        Vector3 operator*(const Vector3& v) const;

        /**
        * Transform a direction vector.
        */
        Vector3 TransformVector(const Vector3& v) const;

        /**
        * Transform a direction vector by the inverse of this matrix.
        */
        Vector3 InverseTransformVector(const Vector3& v) const;

        /**
        * Transform a location - will take into
        * account translation part of the Matrix.
        */
        Vector3 TransformPosition(const Vector3& v) const;

        /**
        * Inverts the matrix and then transforms V
        * - correctly handles scaling in this matrix.
        */
        Vector3 InverseTransformPosition(const Vector3& v) const;

        /**
        * Transpose.
        */
        Matrix Transposed() const;

        /**
        * Return determinant of this matrix.
        */
        real Determinant() const;

        /**
        * Returns a new matrix containing the inverse of this matrix.
        */
        Matrix Inverse() const;

    private:
        /**
        * Calculate the inverse of an Matrix.
        */
        static void VectorMatrixInverse(void* dstMatrix, const void* srcMatrix);
    };
}
