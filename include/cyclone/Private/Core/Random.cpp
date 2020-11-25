#include "Core/Random.h"
#include <ctime>

using namespace cyclone;

Random::Random()
{
    Seed(0);
}

Random::Random(const unsigned seed)
{
    Seed(seed);
}

void Random::Seed(unsigned seed)
{
    if (seed == 0)
    {
        seed = static_cast<unsigned>(clock());
    }

    // Fill the buffer with some basic random numbers
    for (auto i = 0u; i < 17; ++i)
    {
        // Simple linear congruential generator
        seed = seed * 2891336453 + 1;

        buffer[i] = seed;
    }

    // Initialize pointers into the buffer
    p1 = 0;

    p2 = 10;
}

unsigned Random::RandomBits()
{
    // Rotate the buffer and store it back to itself
    const auto result = buffer[p1] = RotLeft(buffer[p2], 13) + RotLeft(buffer[p1], 9);

    // Rotate pointers
    if (--p1 < 0)
    {
        p1 = 16;
    }

    if (--p2 < 0)
    {
        p2 = 16;
    }

    return result;
}

#if SINGLE_PRECISION
real Random::RandomReal()
{
    // Get the random number
    const auto bits = RandomBits();

    // Set up a reinterpret structure for manipulation
    union
    {
        real value;

        unsigned word;
    } convert;

    // Now assign the bits to the word. This works by fixing the ieee
// sign and exponent bits (so that the size of the result is 1-2)
    // and using the bits to create the fraction part of the float.
    convert.word = (bits >> 9) | 0x3f800000;

    return convert.value - 1.f;
}
#else
real Random::RandomReal()
{
    // Get the random number
    const auto bits = RandomBits();

    // Set up a reinterpret structure for manipulation
    union
    {
        real value = 0;

        unsigned words[2];
    } convert;

    // Now assign the bits to the words. This works by fixing the ieee
    // sign and exponent bits (so that the size of the result is 1-2)
    // and using the bits to create the fraction part of the float. Note
    // that bits are used more than once in this process.
    // Fill in the top 16 bits
    convert.words[0] = bits << 20;

    // And the bottom 20
    convert.words[1] = bits >> 12 | 0x3FF00000;

    return convert.value - 1.f;
}
#endif

real Random::RandomReal(const real scale)
{
    return RandomReal() * scale;
}

real Random::RandomReal(const real min, const real max)
{
    return RandomReal() * (max - min) + min;
}

unsigned Random::RandomInt(const unsigned max)
{
    return RandomBits() % max;
}

real Random::RandomBinomial(const real scale)
{
    return (RandomReal() - RandomReal()) * scale;
}

Vector3 Random::RandomVector(const real scale)
{
    return Vector3(
        RandomBinomial(scale),
        RandomBinomial(scale),
        RandomBinomial(scale)
    );
}

Vector3 Random::RandomVector(const Vector3& scale)
{
    return Vector3(
        RandomBinomial(scale.x),
        RandomBinomial(scale.y),
        RandomBinomial(scale.z)
    );
}

Vector3 Random::RandomVector(const Vector3& min, const Vector3& max)
{
    return Vector3(
        RandomReal(min.x, max.x),
        RandomReal(min.y, max.y),
        RandomReal(min.z, max.z)
    );
}

Quaternion Random::RandomQuaternion()
{
    Quaternion q(
        RandomReal(),
        RandomReal(),
        RandomReal(),
        RandomReal()
    );

    q.Normalize();

    return q;
}

unsigned Random::RotLeft(const unsigned n, const unsigned r)
{
    return (n << r) | (n >> (32 - r));
}

unsigned Random::RotRight(const unsigned n, const unsigned r)
{
    return (n >> r) | (n << (32 - r));
}
