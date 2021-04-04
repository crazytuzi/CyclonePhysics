#include "RigidBody/Force/AeroControl.h"

using namespace cyclone;

AeroControl::AeroControl(const Matrix& base, const Matrix& min, const Matrix& max, const Vector3& position,
                         const Vector3* windspeed): Aero(base, position, windspeed), minTensor(min), maxTensor(max),
                                                    controlSetting(0.f)
{
}

void AeroControl::SetControl(const real value)
{
    controlSetting = value;
}

void AeroControl::UpdateForce(RigidBody* body, const real deltaTime)
{
    UpdateForceFromTensor(body, deltaTime, GetTensor());
}

Matrix AeroControl::GetTensor()
{
    if (controlSetting <= -1.f)
    {
        return minTensor;
    }

    if (controlSetting >= 1.f)
    {
        return maxTensor;
    }

    if (controlSetting < 0)
    {
        auto matrix = Matrix();

        for (auto i = 0; i < 4; ++i)
        {
            for (auto j = 0; j < 4; ++j)
            {
                matrix.M[i][j] = minTensor.M[i][j] * -controlSetting + tensor.M[i][j] * (1.f + controlSetting);
            }
        }

        return matrix;
    }

    if (controlSetting > 0)
    {
        auto matrix = Matrix();

        for (auto i = 0; i < 4; ++i)
        {
            for (auto j = 0; j < 4; ++j)
            {
                matrix.M[i][j] = tensor.M[i][j] * (1.f - controlSetting) + maxTensor.M[i][j] * controlSetting;
            }
        }

        return matrix;
    }

    return tensor;
}
