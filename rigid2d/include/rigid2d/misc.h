#ifndef __MISC_H_
#define __MISC_H_

#include <limits>
#include <cmath>

namespace math
{

/// \brief PI.  Not in C++ standard until C++20.
constexpr double PI=3.14159265358979323846;

template <typename real>
inline bool
isZero(real x, real eps=std::numeric_limits<real>::min())
{
  return std::abs(x) <= eps;
}

template <typename real>
inline bool
isNull(real x, real y, real eps=std::numeric_limits<real>::min())
{
  return isZero(x, eps) && isZero(y, eps);
}

}

inline bool 
isNumerical(char c)
{
  return (c >= '0' && c <= '9') || (c == '.') || (c == '-');
}


#endif
