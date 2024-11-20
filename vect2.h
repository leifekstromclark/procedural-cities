#ifndef _VECT2_H_
#define _VECT2_H_

#include <cmath>

class Vect2 {
    double x;
    double y;
    double norm;
  public:
    Vect2(double x, double y): x{x}, y{y}, norm{std::pow(std::pow(x, 2) + std::pow(y, 2), 0.5)} {}
    inline double dot(const Vect2 &other) const {
      return x * other.x + y * other.y;
    }
    inline double getX() const {
      return x;
    }
    inline double getY() const {
      return y;
    }
    inline double getNorm() const {
      return norm;
    }
    inline Vect2 operator+(const Vect2 &other) const {
      return Vect2(x + other.getX(), y + other.getY());
    }
    inline Vect2 operator-(const Vect2 &other) const {
      return Vect2(x - other.getX(), y - other.getY());
    }
    inline Vect2 scale(double c) const {
      return Vect2(x * c, y * c);
    }
    inline Vect2 normalize() const {
      return Vect2(x / norm, y / norm);
    }
    inline Vect2 rotate(const Vect2 &origin, double rot) const {
      double dx = x - origin.getX();
      double dy = y - origin.getY();
      double cos = std::cos(rot);
      double sin = std::sin(rot);
      return Vect2(origin.getX() + dx * cos + dy * -1 * sin, origin.getY() + dx * sin + dy * cos);
    }
    inline double getAngle(const Vect2 &other) const {
      return std::acos(dot(other) / getNorm() / other.getNorm());
    }
    inline double worldAngle() const {
      return getAngle(Vect2(1,0)) * (y < 0 ? -1 : 1);
    }
};

#endif

