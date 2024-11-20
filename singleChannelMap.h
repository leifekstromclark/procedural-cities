#ifndef _SINGLECHANNELMAP_H_
#define _SINGLECHANNELMAP_H_

#include <vector>
#include <cstddef>
#include <memory>
#include "vect2.h"

class OutOfBounds {};

// it would be really cool if this eventually was part of something like blender's node system;

class SingleChannelMap {
  public:
    virtual ~SingleChannelMap() {}
    virtual double sample(Vect2 p) const = 0;
};

typedef double (*Interp)(double a, double b, double w);

inline double lerp(double a, double b, double w) {return (b - a) * w + a; }
inline double smoothStep(double a, double b, double w) {return (b - a) * (3.0 - w * 2.0) * w * w + a; }
inline double smootherStep(double a, double b, double w) {return (b - a) * ((w * (w * 6.0 - 15.0) + 10.0) * w * w * w) + a; }

class PerlinNoiseMap : public SingleChannelMap {
    std::vector<std::vector<Vect2>> grid;
    Vect2 offset;
    double strength;
    double cell_size;
    Interp f;

  public:
    PerlinNoiseMap(Vect2 offset, double strength, size_t w, size_t h, double cell_size, Interp f);
    ~PerlinNoiseMap() {}
    double sample(Vect2 p) const override;
};

class ClampMap : public SingleChannelMap {
    std::unique_ptr<SingleChannelMap> map;
    double min;
    double max;
  public:
    ClampMap(std::unique_ptr<SingleChannelMap> map, double min, double max);
    ~ClampMap() {}
    double sample(Vect2 p) const override;
};

class WaterMap : public SingleChannelMap {
    std::unique_ptr<SingleChannelMap> map;
    double threshold;
  public:
    WaterMap(std::unique_ptr<SingleChannelMap> map, double threshold);
    ~WaterMap() {}
    double sample(Vect2 p) const override;
};

#endif

