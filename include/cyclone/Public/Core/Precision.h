#pragma once

/**
* @file
*
* Because Cyclone is designed to work at either single or double
* precision, mathematical functions such as sqrt cannot be used
* in the source code or headers. This file provides defines for
* the real number type and mathematical formulae that work on it.
*
* @note All the contents of this file need to be changed to compile
* Cyclone at a different precision.
*/

namespace cyclone
{
#if false
    /**
    * Defines we're in single precision mode, for any code
    * that needs to be conditionally compiled.
    */
    #define SINGLE_PRECISION true

    /**
    * Defines a real number precision. Cyclone can be compiled in
    * single or double precision versions. By default single precision is
    * provided.
    */
    typedef float real;

    /** Defines the highest value for the real number. */
    #define REAL_MAX FLT_MAX

    /** Defines the precision of the square root operator. */
    #define real_sqrt std::sqrtf

    /** Defines the precision of the absolute magnitude operator. */
    #define real_abs std::fabsf

    /** Defines the precision of the sine operator. */
    #define real_sin std::sinf

    /** Defines the precision of the cosine operator. */
    #define real_cos std::cosf
    
    /** Defines the precision of the exponent operator. */
    #define real_exp std::expf

    /** Defines the precision of the power operator. */
    #define real_pow std::powf

    /** Defines the precision of the floating point modulo operator. */
    #define real_fmod std::fmodf

    /** Defines the number e on which 1+e == 1 **/
    #define real_epsilon FLT_EPSILON

    #define R_PI 3.14159f
#else
    /**
    * Defines we're in double precision mode, for any code
    * that needs to be conditionally compiled.
    */
#define DOUBLE_PRECISION true

    /**
    * Defines a real number precision. Cyclone can be compiled in
    * single or double precision versions. By default single precision is
    * provided.
    */
    typedef double real;

    /** Defines the highest value for the real number. */
#define REAL_MAX DBL_MAX

    /** Defines the precision of the square root operator. */
#define real_sqrt std::sqrt

    /** Defines the precision of the absolute magnitude operator. */
#define real_abs std::fabs

    /** Defines the precision of the sine operator. */
#define real_sin std::sin

    /** Defines the precision of the cosine operator. */
#define real_cos std::cos

    /** Defines the precision of the exponent operator. */
#define real_exp std::exp

    /** Defines the precision of the power operator. */
#define real_pow std::pow

    /** Defines the precision of the floating point modulo operator. */
#define real_fmod std::fmod

    /** Defines the number e on which 1+e == 1 **/
#define real_epsilon DBL_EPSILON

#define R_PI 3.14159265358979
#endif
}
