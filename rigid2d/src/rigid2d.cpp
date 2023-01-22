#include "rigid2d/rigid2d.hpp"
#include "rigid2d/misc.h"

namespace rigid2d
{
  std::ostream & operator<<(std::ostream & os, const Vector2D & v)
  {
    os << '[' << v.x << ", " << v.y << ']';
    return os;
  }
  
  std::istream & operator>>(std::istream & is, Vector2D & v)
  {
    while(is.peek() == '\n' || is.peek() == ' ')
      is.get();

    if(is.peek() == '[')
      is.get();

    char num_str[32];
    int i = 0;
    while(is.peek() != '\n' && is.peek() != ' ')
    {
      if(isNumerical(is.peek()))
      {
        is.get(num_str[i++]); 
      }
      else
        is.get();
    }
    num_str[i] = '\0';
    v.x = std::stod(num_str);
    while(is.peek() == '\n' || is.peek() == ' ')
      is.get();

    i = 0;
    while(is.peek() != -1 && is.peek() != '\n' && 
          is.peek() != '\0' && is.peek() != ' ')
    {
      if(isNumerical(is.peek()))
      {
        is.get(num_str[i++]); 
      }
      else
        is.get();
    }
    num_str[i] = '\0';
    v.y = std::stod(num_str);

    if(is.peek() == ']')
      is.get(); 

    return is;
  }
  
  Vector2D&
  Vector2D::normalize()
  {
    auto inv_norm = 1.0 / this->norm();
    x *= inv_norm;
    y *= inv_norm;
    return *this;
  }

  double 
  Vector2D::norm() const
  {
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
  }

  double 
  Vector2D::angle() const
  {
    return std::atan2(y, x);
  }
  
  

  Vector2D
  Transform2D::operator()(const Vector2D& v) const
  {
    // Apply transform (rotation and translation) to a point
    Vector2D w;
    w.x = _u.x*v.x + _v.x*v.y + _o.x; 
    w.y = _u.y*v.x + _v.y*v.y + _o.y;

    w.x = almost_equal(w.x, 0) ? 0 : w.x;
    w.y = almost_equal(w.y, 0) ? 0 : w.y;

    return w;
  }

  Twist 
  Transform2D::operator()(const Twist& tw) const
  {
    // The angular velocity speed does not change with the change in
    // space coordinates, neither the angular velocity axis. 
    double vx = _u.x*tw.v.x + _v.x*tw.v.y;
    double vy = _u.y*tw.v.x + _v.y*tw.v.y;
    Twist twb{tw.w, vx, vy};
    return twb;
  }


  Transform2D
  Transform2D::inv() const
  {
    Transform2D t;
    
    auto ctheta = _u.x;
    auto stheta = _u.y;

    t._u.x = ctheta;
    t._u.y = -stheta;
    t._v.x = stheta;
    t._v.y = ctheta;

    t._theta = std::atan2(t._u.y, t._u.x);
    t._theta = almost_equal(t._theta, 0) ? 0 : t._theta;

    t._o.x = -(_o.x*ctheta + _o.y*stheta);
    t._o.y = _o.x*stheta - _o.y*ctheta; 
    
    t._o.x = almost_equal(t._o.x, 0) ? 0 : t._o.x;
    t._o.y = almost_equal(t._o.y, 0) ? 0 : t._o.y;

    return t;
  }

  Transform2D&
  Transform2D::operator*=(const Transform2D& rhs)
  {
    auto& p = rhs._o;
    _o.x = _u.x*p.x + _v.x*p.y + _o.x;
    _o.y = _u.y*p.x + _v.y*p.y + _o.y;
    _o.x = almost_equal(_o.x, 0) ? 0 : _o.x;
    _o.y = almost_equal(_o.y, 0) ? 0 : _o.y;
    
    auto& w = rhs._u;
    _u.x = _u.x*w.x + _v.x*w.y;
    _u.y = _u.y*w.x + _v.y*w.y;
    
    auto& q = rhs._v;
    _v.x = _u.x*q.x + _v.x*q.y;
    _v.y = _u.y*q.x + _v.y*q.y;
    
    _u.normalize();
    _v.normalize();

    _theta = std::atan2(_u.y, _u.x);
    _theta = almost_equal(_theta, 0) ? 0 : _theta;

    return *this;
  }

  double
  Transform2D::y() const
  {
    return _o.y;
  }

  double
  Transform2D::x() const
  {
    return _o.x;
  }
  
  double
  Transform2D::theta() const
  {
    return _theta;
  }

  std::ostream&
  operator<<(std::ostream& os, const Transform2D& tf)
  {
    os << "(degrees): " << rad2deg(tf.theta()) << " ";
    os << "dx: " << tf.x() << " dy: " << tf.y();
    return os;
  }

  Transform2D
  operator*(Transform2D const& lhs, Transform2D const& rhs)
  {
    Transform2D t(lhs._o, lhs.theta());
    t *= rhs;
    return t;
  }

  std::istream&
  operator>>(std::istream & is, Transform2D &tf)
  {
    double tr[3];
    int count = 0;
    while(count < 3)
    {
      while(!isNumerical(is.peek()))
        is.get();

      char buff[32]; int i = 0;
      while(isNumerical(is.peek()))
        is.get(buff[i++]);
      buff[i] = '\0';
      tr[count++] = std::stod(buff);
    }
    tf.setTR(tr[1], tr[2], deg2rad(tr[0]));
    return is;
  }

  Transform2D
  Transform2D::integrateTwistForOneTimeUnit(Twist const& tw)
  {
    double ox, oy;
    if(!almost_equal(tw.w, 0))
    {
      auto inv_theta = 1.0 / tw.w;
      auto sx = -tw.v.y * inv_theta;
      auto sy = tw.v.x * inv_theta;
      
      double ctheta = std::cos(tw.w);
      double stheta = std::sin(tw.w);
      ox = (-sx)*ctheta - (-sy)*stheta + sx;
      oy = (-sx)*stheta + (-sy)*ctheta + sy;

    }
    else
    {
      ox = tw.v.x;
      oy = tw.v.y;
    }
    
    Transform2D twTransform(Vector2D(ox, oy), tw.w);
    
    auto& thisTransform = *this;
    return thisTransform*twTransform;
  }
}

