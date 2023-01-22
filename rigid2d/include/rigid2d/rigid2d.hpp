#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include <iosfwd> // contains forward definitions for iostream objects
#include <cstdlib> 
#include <iostream>
#include <string>
#include <cmath>
#include "rigid2d/misc.h"

namespace rigid2d
{
    
    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
      return fabs(d1 - d2) < epsilon;
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {
      return deg * math::PI / 180.0;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
      return rad * 180.0 / math::PI;
    }

    inline double
    normalize_angle(double rad)
    {
      int n = 0.5 * rad / math::PI;
      rad -= n * 2.0 * math::PI;

      if(rad > math::PI)
        return rad - 2*math::PI;
      else if (rad < -math::PI)
        return 2*math::PI + rad;

      return rad;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    struct Pose
    {
      double x, y, theta;
    };

    /// \brief A 2-Dimensional Vector
    class Vector2D
    {
    public:
      double x, y;

      Vector2D() : x{}, y{}{};
      Vector2D(double x_, double y_) : x{x_}, y{y_}{};
      
      Vector2D& normalize();
      double norm() const;
      double angle() const;
      inline bool equals(Vector2D const & v) const;
      
      double& operator[](int i)
      {
        return (&x)[i]; 
      }

      const double& operator[](int i) const
      {
        return (&x)[i];
      }
      
      Vector2D operator+(const Vector2D& v) const
      {
        return Vector2D{x + v.x, y + v.y};
      }

      Vector2D& operator+=(const Vector2D& v)
      {
        x += v.x;
        y += v.y;
        return *this;
      }

      Vector2D operator-(const Vector2D& v) const
      {
        return Vector2D{x - v.x, y - v.y};
      }

      Vector2D& operator-=(const Vector2D& v)
      {
        x -= v.x;
        y -= v.y;
        return *this;
      }

      Vector2D operator*(double a) const
      {
        return Vector2D{a * x, a * y};
      }

      Vector2D& operator *=(double a)
      {
        x *= a;
        y *= a;
        return *this;
      }
      
      bool operator ==(Vector2D const& v) const
      {
        return equals(v);
      }
      
    };
    
    inline bool 
    Vector2D::equals(Vector2D const & v) const
    {
      return math::isNull(x-v.x, y-v.y);
    }
    
    inline Vector2D 
    operator*(double a, Vector2D const& v)
    {
      return Vector2D{a * v.x, a * v.y};
    }


    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered 
    ///   as [xcomponent, ycomponent]
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    std::istream & operator>>(std::istream & is, Vector2D & v); 

    struct Twist
    {
      Twist() = default;
      Twist(double w_, double x, double y): w{w_}, v{x, y}
      {}
      double w;
      Vector2D v;
    };

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    private:
      Vector2D _u, _v, _o;
      double _theta{0.0};

    public:
        /// \brief Create an identity transformation
        Transform2D() = default;

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D & trans)
        {
          _o.x = trans.x;
          _o.y = trans.y;
        }

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians)
        {
          _u.x = std::cos(radians);
          _u.y = std::sin(radians);
          _v.x = -1.0 * _u.y;
          _v.y = _u.y;

          _theta = radians;
        }

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(const Vector2D & trans, double radians)
        {
          setTR(trans.x, trans.y, radians);
        }

        void setTR(double dx, double dy, double radians)
        {
          _o.x = dx;
          _o.y = dy;
          _u.x = std::cos(radians);
          _u.y = std::sin(radians);
          _v.x = -1.0 * _u.y;
          _v.y = _u.x;
          _theta = radians;
        }

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(const Vector2D& v) const;
        
        /// \brief apply a transformation to a Twist
        /// \param tw - the twist to transform
        /// \return a twist in the new coordinate system
        Twist operator()(const Twist& tw) const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        Transform2D integrateTwistForOneTimeUnit(Twist const & tw); 


        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \returns a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief get the x displacement of the  transformation
        /// \return the x displacement
        double x() const;
        
        /// \brief get the y displacement of the  transformation
        /// \return the y displacement
        double y() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement
        double theta() const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, 
                                         const Transform2D & tf);
        /// \brief multiply two transforms together, returning their composition
        /// \param lhs - the left hand operand
        /// \param rhs - the right hand operand
        /// \return the composition of the two transforms
        /// HINT: This function should be implemented in terms of *=
        friend Transform2D operator*(Transform2D const& lhs, 
                                     Transform2D const& rhs);
    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    

}

#endif
