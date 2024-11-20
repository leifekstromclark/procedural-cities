#include <optional>
#include "vect2.h"
#include "util.h"

std::optional<Vect2> intersect(Coeff co1, Coeff co2) {
    double det  = co1.a * co2.b - co1.b * co2.a;
    double det_x = co1.c * co2.b - co1.b * co2.c;
    double det_y = co1.a * co2.c - co1.c * co2.a;
    if (det != 0) {
        return std::optional<Vect2>{Vect2(det_x / det, det_y / det)};
    }
    return std::nullopt;
}

