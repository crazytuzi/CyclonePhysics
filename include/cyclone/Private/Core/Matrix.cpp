#include "Core/Matrix.h"

using namespace cyclone;

Matrix::Matrix()
{
    for (auto i = 0; i < 4; ++i)
    {
        for (auto j = 0; j < 4; ++j)
        {
            M[i][j] = 0;
        }
    }
}

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

Vector4 Matrix::operator*(const Vector4& v) const
{
    Vector4 Result;

    for (auto k = 0; k < 4; ++k)
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

Vector4 Matrix::TransformVector(const Vector3& v) const
{
    return *this * Vector4(v, 0.f);
}


Vector3 Matrix::InverseTransformVector(const Vector3& v) const
{
    const auto InverseSelf = Inverse();

    return InverseSelf.TransformVector(v);
}

Vector4 Matrix::TransformPosition(const Vector3& v) const
{
    return *this * Vector4(v, 1.f);
}

Vector3 Matrix::InverseTransformPosition(const Vector3& v) const
{
    const auto InverseSelf = Inverse();

    return InverseSelf.TransformPosition(v);
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
    real inv[16] = {
        M[1][1] * M[2][2] * M[3][3] - M[1][1] * M[2][3] * M[3][2] - M[2][1] * M[1][2] * M[3][3] + M[2][1] * M[1][3] * M[
            3][2] + M[3][1] * M[1][2] * M[2][3] - M[3][1] * M[1][3] * M[2][2],
        -M[0][1] * M[2][2] * M[3][3] + M[0][1] * M[2][3] * M[3][2] + M[2][1] * M[0][2] * M[3][3] - M[2][1] * M[0][3] * M
        [3][2] - M[3][1] * M[0][2] * M[2][3] + M[3][1] * M[0][3] * M[2][2],
        M[0][1] * M[1][2] * M[3][3] - M[0][1] * M[1][3] * M[3][2] - M[1][1] * M[0][2] * M[3][3] + M[1][1] * M[0][3] * M[
            3][2] + M[3][1] * M[0][2] * M[1][3] - M[3][1] * M[0][3] * M[1][2],
        -M[0][1] * M[1][2] * M[2][3] + M[0][1] * M[1][3] * M[2][2] + M[1][1] * M[0][2] * M[2][3] - M[1][1] * M[0][3] * M
        [2][2] - M[2][1] * M[0][2] * M[1][3] + M[2][1] * M[0][3] * M[1][2],
        -M[1][0] * M[2][2] * M[3][3] + M[1][0] * M[2][3] * M[3][2] + M[2][0] * M[1][2] * M[3][3] - M[2][0] * M[1][3] * M
        [3][2] - M[3][0] * M[1][2] * M[2][3] + M[3][0] * M[1][3] * M[2][2],
        M[0][0] * M[2][2] * M[3][3] - M[0][0] * M[2][3] * M[3][2] - M[2][0] * M[0][2] * M[3][3] + M[2][0] * M[0][3] * M[
            3][2] + M[3][0] * M[0][2] * M[2][3] - M[3][0] * M[0][3] * M[2][2],
        -M[0][0] * M[1][2] * M[3][3] + M[0][0] * M[1][3] * M[3][2] + M[1][0] * M[0][2] * M[3][3] - M[1][0] * M[0][3] * M
        [3][2] - M[3][0] * M[0][2] * M[1][3] + M[3][0] * M[0][3] * M[1][2],
        M[0][0] * M[1][2] * M[2][3] - M[0][0] * M[1][3] * M[2][2] - M[1][0] * M[0][2] * M[2][3] + M[1][0] * M[0][3] * M[
            2][2] + M[2][0] * M[0][2] * M[1][3] - M[2][0] * M[0][3] * M[1][2],
        M[1][0] * M[2][1] * M[3][3] - M[1][0] * M[2][3] * M[3][1] - M[2][0] * M[1][1] * M[3][3] + M[2][0] * M[1][3] * M[
            3][1] + M[3][0] * M[1][1] * M[2][3] - M[3][0] * M[1][3] * M[2][1],
        -M[0][0] * M[2][1] * M[3][3] + M[0][0] * M[2][3] * M[3][1] + M[2][0] * M[0][1] * M[3][3] - M[2][0] * M[0][3] * M
        [3][1] - M[3][0] * M[0][1] * M[2][3] + M[3][0] * M[0][3] * M[2][1],
        M[0][0] * M[1][1] * M[3][3] - M[0][0] * M[1][3] * M[3][1] - M[1][0] * M[0][1] * M[3][3] + M[1][0] * M[0][3] * M[
            3][1] + M[3][0] * M[0][1] * M[1][3] - M[3][0] * M[0][3] * M[1][1],
        -M[0][0] * M[1][1] * M[2][3] + M[0][0] * M[1][3] * M[2][1] + M[1][0] * M[0][1] * M[2][3] - M[1][0] * M[0][3] * M
        [2][1] - M[2][0] * M[0][1] * M[1][3] + M[2][0] * M[0][3] * M[1][1],
        -M[1][0] * M[2][1] * M[3][2] + M[1][0] * M[2][2] * M[3][1] + M[2][0] * M[1][1] * M[3][2] - M[2][0] * M[1][2] * M
        [3][1] - M[3][0] * M[1][1] * M[2][2] + M[3][0] * M[1][2] * M[2][1],
        M[0][0] * M[2][1] * M[3][2] - M[0][0] * M[2][2] * M[3][1] - M[2][0] * M[0][1] * M[3][2] + M[2][0] * M[0][2] * M[
            3][1] + M[3][0] * M[0][1] * M[2][2] - M[3][0] * M[0][2] * M[2][1],
        -M[0][0] * M[1][1] * M[3][2] + M[0][0] * M[1][2] * M[3][1] + M[1][0] * M[0][1] * M[3][2] - M[1][0] * M[0][2] * M
        [3][1] - M[3][0] * M[0][1] * M[1][2] + M[3][0] * M[0][2] * M[1][1],
        M[0][0] * M[1][1] * M[2][2] - M[0][0] * M[1][2] * M[2][1] - M[1][0] * M[0][1] * M[2][2] + M[1][0] * M[0][2] * M[
            2][1] + M[2][0] * M[0][1] * M[1][2] - M[2][0] * M[0][2] * M[1][1]
    };

    auto det = M[0][0] * inv[0] + M[0][1] * inv[4] + M[0][2] * inv[8] + M[0][3] * inv[12];

    if (det != 0.f)
    {
        det = 1.f / det;
    }

    Matrix m;

    for (auto i = 0; i < 4; ++i)
    {
        for (auto j = 0; j < 4; ++j)
        {
            m.M[i][j] = inv[i * 4 + j] * det;
        }
    }

    return m;
}
