#include "Core/Matrix.h"
#include <cstring>

using namespace cyclone;

Matrix::Matrix(const Vector3& InX, const Vector3& InY, const Vector3& InZ, const Vector3& InW)
{
    M[0][0] = InX.x;

    M[0][1] = InX.y;

    M[0][2] = InX.z;

    M[0][3] = 0.f;

    M[1][0] = InY.x;

    M[1][1] = InY.y;

    M[1][2] = InY.z;

    M[1][3] = 0.f;

    M[2][0] = InZ.x;

    M[2][1] = InZ.y;

    M[2][2] = InZ.z;

    M[2][3] = 0.f;

    M[3][0] = InW.x;

    M[3][1] = InW.y;

    M[3][2] = InW.z;

    M[3][3] = 1.f;
}

Matrix::Matrix(const real x0, const real x1, const real x2, const real x3, const real y0, const real y1, const real y2,
               const real y3, const real z0, const real z1, const real z2, const real z3, const real w0, const real w1,
               const real w2, const real w3)
{
    M[0][0] = x0;

    M[0][1] = x1;

    M[0][2] = x2;

    M[0][3] = x3;

    M[1][0] = y0;

    M[1][1] = y1;

    M[1][2] = y2;

    M[1][3] = y3;

    M[2][0] = z0;

    M[2][1] = z1;

    M[2][2] = z2;

    M[2][3] = z3;

    M[3][0] = w0;

    M[3][1] = w1;

    M[3][2] = w2;

    M[3][3] = w3;
}

void Matrix::SetIdentity()
{
    for (auto i = 0; i < 4; ++i)
    {
        for (auto j = 0; j < 4; ++j)
        {
            i == j ? M[i][j] = 1.f : M[i][j] = 0.f;
        }
    }
}

Matrix Matrix::operator*(const Matrix& m) const
{
    Matrix Result;

    for (auto k = 0; k < 4; ++k)
    {
        for (auto i = 0; i < 4; ++i)
        {
            for (auto j = 0; j < 4; ++j)
            {
                Result.M[i][k] += M[i][j] * m.M[j][k];
            }
        }
    }

    return Result;
}

void Matrix::operator*=(const Matrix& m)
{
    *this = *this * m;
}

Matrix Matrix::operator+(const Matrix& m) const
{
    Matrix Result;

    for (auto i = 0; i < 4; ++i)
    {
        for (auto j = 0; j < 4; ++j)
        {
            Result.M[i][j] = M[i][j] + m.M[i][j];
        }
    }

    return Result;
}

void Matrix::operator+=(const Matrix& m)
{
    *this = *this + m;
}

Matrix Matrix::operator*(const float scale)
{
    Matrix Result;

    for (auto i = 0; i < 4; ++i)
    {
        for (auto j = 0; j < 4; ++j)
        {
            Result.M[i][j] = M[i][j] * scale;
        }
    }

    return Result;
}

void Matrix::operator*=(const float scale)
{
    *this = *this * scale;
}

bool Matrix::operator==(const Matrix& m) const
{
    for (auto i = 0; i < 4; ++i)
    {
        for (auto j = 0; j < 4; ++j)
        {
            if (M[i][j] != m.M[i][j])
            {
                return false;
            }
        }
    }

    return true;
}

bool Matrix::operator!=(const Matrix& m) const
{
    return !(*this == m);
}

Vector3 Matrix::operator*(const Vector3& v) const
{
    Vector3 Result;

    for (auto k = 0; k < 3; ++k)
    {
        for (auto i = 0; i < 4; ++i)
        {
            for (auto j = 0; j < 4; ++j)
            {
                Result[k] += v[k] * M[i][j];
            }
        }
    }

    return Result;
}

Vector3 Matrix::TransformVector(const Vector3& v) const
{
    return Vector3(v.x * M[0][0] + v.y * M[0][1] + v.z * M[0][2],
                   v.x * M[1][0] + v.y * M[1][1] + v.z * M[1][2],
                   v.x * M[2][0] + v.y * M[2][1] + v.z * M[2][2]);
}

Vector3 Matrix::InverseTransformVector(const Vector3& v) const
{
    return Vector3(v.x * M[0][0] + v.y * M[1][0] + v.z * M[2][0],
                   v.x * M[0][1] + v.y * M[1][1] + v.z * M[2][1],
                   v.x * M[0][2] + v.y * M[1][2] + v.z * M[2][2]);
}

Vector3 Matrix::TransformPosition(const Vector3& v) const
{
    return Vector3(v.x * M[0][0] + v.y * M[0][1] + v.z * M[0][2] + M[0][3],
                   v.x * M[1][0] + v.y * M[1][1] + v.z * M[1][2] + M[1][3],
                   v.x * M[2][0] + v.y * M[2][1] + v.z * M[2][2] + M[2][3]);
}

Vector3 Matrix::InverseTransformPosition(const Vector3& v) const
{
    return Vector3(v.x * M[0][0] + v.y * M[1][0] + v.z * M[2][0] + M[3][0],
                   v.x * M[0][1] + v.y * M[1][1] + v.z * M[2][1] + M[3][1],
                   v.x * M[0][2] + v.y * M[1][2] + v.z * M[2][2] + M[3][2]);
}

Matrix Matrix::Transposed() const
{
    Matrix Result;

    Result.M[0][0] = M[0][0];

    Result.M[0][1] = M[1][0];

    Result.M[0][2] = M[2][0];

    Result.M[0][3] = M[3][0];

    Result.M[1][0] = M[0][1];

    Result.M[1][1] = M[1][1];

    Result.M[1][2] = M[2][1];

    Result.M[1][3] = M[3][1];

    Result.M[2][0] = M[0][2];

    Result.M[2][1] = M[1][2];

    Result.M[2][2] = M[2][2];

    Result.M[2][3] = M[3][2];

    Result.M[3][0] = M[0][3];

    Result.M[3][1] = M[1][3];

    Result.M[3][2] = M[2][3];

    Result.M[3][3] = M[3][3];

    return Result;
}

real Matrix::Determinant() const
{
    return
        M[0][0] * (
            M[1][1] * (M[2][2] * M[3][3] - M[2][3] * M[3][2]) -
            M[2][1] * (M[1][2] * M[3][3] - M[1][3] * M[3][2]) +
            M[3][1] * (M[1][2] * M[2][3] - M[1][3] * M[2][2])
        ) -
        M[1][0] * (
            M[0][1] * (M[2][2] * M[3][3] - M[2][3] * M[3][2]) -
            M[2][1] * (M[0][2] * M[3][3] - M[0][3] * M[3][2]) +
            M[3][1] * (M[0][2] * M[2][3] - M[0][3] * M[2][2])
        ) +
        M[2][0] * (
            M[0][1] * (M[1][2] * M[3][3] - M[1][3] * M[3][2]) -
            M[1][1] * (M[0][2] * M[3][3] - M[0][3] * M[3][2]) +
            M[3][1] * (M[0][2] * M[1][3] - M[0][3] * M[1][2])
        ) -
        M[3][0] * (
            M[0][1] * (M[1][2] * M[2][3] - M[1][3] * M[2][2]) -
            M[1][1] * (M[0][2] * M[2][3] - M[0][3] * M[2][2]) +
            M[2][1] * (M[0][2] * M[1][3] - M[0][3] * M[1][2])
        );
}

Matrix Matrix::Inverse() const
{
    Matrix Result;

    const auto Det = Determinant();

    if (Det == 0.f)
    {
        Result = Matrix(1.f, 0.f, 0.f, 0.f,
                        0.f, 1.f, 0.f, 0.f,
                        0.f, 0.f, 1.f, 0.f,
                        0.f, 0.f, 0.f, 1.f);
    }
    else
    {
        VectorMatrixInverse(&Result, this);
    }

    return Result;
}

void Matrix::VectorMatrixInverse(void* dstMatrix, const void* srcMatrix)
{
    typedef real Float4x4[4][4];

    const auto& M = *static_cast<const Float4x4*>(srcMatrix);

    Float4x4 Result;

    real Det[4];

    Float4x4 Tmp;

    Tmp[0][0] = M[2][2] * M[3][3] - M[2][3] * M[3][2];

    Tmp[0][1] = M[1][2] * M[3][3] - M[1][3] * M[3][2];

    Tmp[0][2] = M[1][2] * M[2][3] - M[1][3] * M[2][2];

    Tmp[1][0] = M[2][2] * M[3][3] - M[2][3] * M[3][2];

    Tmp[1][1] = M[0][2] * M[3][3] - M[0][3] * M[3][2];

    Tmp[1][2] = M[0][2] * M[2][3] - M[0][3] * M[2][2];

    Tmp[2][0] = M[1][2] * M[3][3] - M[1][3] * M[3][2];

    Tmp[2][1] = M[0][2] * M[3][3] - M[0][3] * M[3][2];

    Tmp[2][2] = M[0][2] * M[1][3] - M[0][3] * M[1][2];

    Tmp[3][0] = M[1][2] * M[2][3] - M[1][3] * M[2][2];

    Tmp[3][1] = M[0][2] * M[2][3] - M[0][3] * M[2][2];

    Tmp[3][2] = M[0][2] * M[1][3] - M[0][3] * M[1][2];

    Det[0] = M[1][1] * Tmp[0][0] - M[2][1] * Tmp[0][1] + M[3][1] * Tmp[0][2];

    Det[1] = M[0][1] * Tmp[1][0] - M[2][1] * Tmp[1][1] + M[3][1] * Tmp[1][2];

    Det[2] = M[0][1] * Tmp[2][0] - M[1][1] * Tmp[2][1] + M[3][1] * Tmp[2][2];

    Det[3] = M[0][1] * Tmp[3][0] - M[1][1] * Tmp[3][1] + M[2][1] * Tmp[3][2];

    const auto Determinant = M[0][0] * Det[0] - M[1][0] * Det[1] + M[2][0] * Det[2] - M[3][0] * Det[3];

    const auto RDet = 1.f / Determinant;

    Result[0][0] = RDet * Det[0];

    Result[0][1] = -RDet * Det[1];

    Result[0][2] = RDet * Det[2];

    Result[0][3] = -RDet * Det[3];

    Result[1][0] = -RDet * (M[1][0] * Tmp[0][0] - M[2][0] * Tmp[0][1] + M[3][0] * Tmp[0][2]);

    Result[1][1] = RDet * (M[0][0] * Tmp[1][0] - M[2][0] * Tmp[1][1] + M[3][0] * Tmp[1][2]);

    Result[1][2] = -RDet * (M[0][0] * Tmp[2][0] - M[1][0] * Tmp[2][1] + M[3][0] * Tmp[2][2]);

    Result[1][3] = RDet * (M[0][0] * Tmp[3][0] - M[1][0] * Tmp[3][1] + M[2][0] * Tmp[3][2]);

    Result[2][0] = RDet * (
        M[1][0] * (M[2][1] * M[3][3] - M[2][3] * M[3][1]) -
        M[2][0] * (M[1][1] * M[3][3] - M[1][3] * M[3][1]) +
        M[3][0] * (M[1][1] * M[2][3] - M[1][3] * M[2][1])
    );

    Result[2][1] = -RDet * (
        M[0][0] * (M[2][1] * M[3][3] - M[2][3] * M[3][1]) -
        M[2][0] * (M[0][1] * M[3][3] - M[0][3] * M[3][1]) +
        M[3][0] * (M[0][1] * M[2][3] - M[0][3] * M[2][1])
    );

    Result[2][2] = RDet * (
        M[0][0] * (M[1][1] * M[3][3] - M[1][3] * M[3][1]) -
        M[1][0] * (M[0][1] * M[3][3] - M[0][3] * M[3][1]) +
        M[3][0] * (M[0][1] * M[1][3] - M[0][3] * M[1][1])
    );

    Result[2][3] = -RDet * (
        M[0][0] * (M[1][1] * M[2][3] - M[1][3] * M[2][1]) -
        M[1][0] * (M[0][1] * M[2][3] - M[0][3] * M[2][1]) +
        M[2][0] * (M[0][1] * M[1][3] - M[0][3] * M[1][1])
    );

    Result[3][0] = -RDet * (
        M[1][0] * (M[2][1] * M[3][2] - M[2][2] * M[3][1]) -
        M[2][0] * (M[1][1] * M[3][2] - M[1][2] * M[3][1]) +
        M[3][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
    );

    Result[3][1] = RDet * (
        M[0][0] * (M[2][1] * M[3][2] - M[2][2] * M[3][1]) -
        M[2][0] * (M[0][1] * M[3][2] - M[0][2] * M[3][1]) +
        M[3][0] * (M[0][1] * M[2][2] - M[0][2] * M[2][1])
    );

    Result[3][2] = -RDet * (
        M[0][0] * (M[1][1] * M[3][2] - M[1][2] * M[3][1]) -
        M[1][0] * (M[0][1] * M[3][2] - M[0][2] * M[3][1]) +
        M[3][0] * (M[0][1] * M[1][2] - M[0][2] * M[1][1])
    );

    Result[3][3] = RDet * (
        M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
        M[1][0] * (M[0][1] * M[2][2] - M[0][2] * M[2][1]) +
        M[2][0] * (M[0][1] * M[1][2] - M[0][2] * M[1][1])
    );

    memcpy(dstMatrix, &Result, 16 * sizeof(real));
}
