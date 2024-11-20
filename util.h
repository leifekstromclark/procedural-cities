#ifndef _UTIL_H_
#define _UTIL_H_

#include <cmath>
#include <optional>
#include "vect2.h"

inline bool angleBetweenCCW(double lower, double theta, double upper) {
    lower = fmod(lower, M_PI * 2);
    upper = fmod(upper, M_PI * 2);
    theta = fmod(theta, M_PI * 2);
    if (lower < 0) lower += M_PI * 2;
    if (upper < 0) upper += M_PI * 2;
    if (theta < 0) theta += M_PI * 2;
    return (lower <= upper && lower <= theta && theta <= upper) || (lower > upper && (lower <= theta || theta <= upper));
}

inline double angleDiffCCW(double a, double b) {
    double diff = fmod(a - b, M_PI * 2);
    if (diff < 0) diff += M_PI * 2;
    return diff;
}

inline double angleDiff(double a, double b) {
    double diff = angleDiffCCW(a, b);
    if (diff > M_PI) diff = M_PI * 2 - diff;
    return diff;
}

struct Coeff {
    double a, b, c;
};

inline Coeff getCoeff(Vect2 p1, Vect2 p2) {
    double a = p1.getY() - p2.getY();
    double b = p2.getX() - p1.getX();
    double c = p1.getX() * p2.getY() - p2.getX() * p1.getY();
    return Coeff {a, b, -c};
}

std::optional<Vect2> intersect(Coeff co1, Coeff co2);

#endif

