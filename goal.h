#ifndef _GOAL_H_
#define _GOAL_H_

#include <vector>
#include <memory>
#include "random"
#include "vect2.h"
#include "cityLayout.h"
#include "singleChannelMap.h"

struct Extension {
    size_t base;
    Vect2 tip;
    bool highway;
};

class PrecomputeGoal { // could replace with more advanced observer goal in the future
  public:
    virtual void update(const CityLayout &city) = 0;
    virtual ~PrecomputeGoal() {}
};

class SampleGoal {
  public:
    double weight;
    SampleGoal(double weight): weight{weight} {}
    inline double getWeight() const {return weight; }
    virtual double weigh(size_t node, bool highway, const CityLayout& city) const = 0;
    virtual ~SampleGoal() {}
};
/*
class NearestGrowthCenter : public SampleGoal {
    std::vector<Vect2> growthCenters;
    double strength;
  public:
    double weigh(size_t node, const CityLayout& city) const override;
    ~NearestGrowthCenter() {};
};
*/

class IntersectionRatio : public SampleGoal, public PrecomputeGoal {
    double currentRatio;
    double targetRatio;
  public:
    IntersectionRatio(double weight, double targetRatio);
    double weigh(size_t node, bool highway, const CityLayout &city) const override;
    void update(const CityLayout &city) override;
    ~IntersectionRatio() {}
};

class ExtensionGoal {
  public:
    double weight;
    ExtensionGoal(double weight): weight{weight} {}
    inline double getWeight() const {return weight; }
    virtual std::pair<double, double> evaluate(size_t node, double angle, bool highway, const CityLayout& city) = 0;
    virtual ~ExtensionGoal() {}
};

/*
class MajorMinor : public ExtensionGoal {
    std::unique_ptr<ExtensionGoal> major;
    std::unique_ptr<ExtensionGoal> minor;
  public:
    MajorMinor(std::unique_ptr<ExtensionGoal> major, std::unique_ptr<ExtensionGoal> minor);
    std::pair<double, double> evaluate(size_t node, double angle, bool highway, const CityLayout &city) override;
    ~MajorMinor() {};
};
*/

class RandomDeviation : public ExtensionGoal {
    std::default_random_engine generator;
    std::normal_distribution<double> lengthDist;
    std::normal_distribution<double> angleDist;
  public:
    RandomDeviation(double weight, double meanLength, double devLength, double devAngle);
    std::pair<double, double> evaluate(size_t node, double angle, bool highway, const CityLayout& city) override;
    ~RandomDeviation() {}
};

class Paris : public ExtensionGoal {
    Vect2 center;
    double perpLength;
    double paraLength;
  public:
    Paris(double weight, Vect2 center, double perpLength, double paraLength);
    std::pair<double, double> evaluate(size_t node, double angle, bool highway, const CityLayout &city) override;
    ~Paris() {}
};

class Manhattan : public ExtensionGoal { // NOTE: develop a better "adaptive manhattan" for best per-quarter result
    double blockAngle;
    double perpLength;
    double paraLength;
  public:
    Manhattan(double weight, double blockAngle, double perpLength, double paraLength);
    ~Manhattan() {}
    std::pair<double, double> evaluate(size_t node, double angle, bool highway, const CityLayout &city) override;
};

class SanFrancisco : public ExtensionGoal {
    std::unique_ptr<SingleChannelMap> elevationMap;
    double length;
    double maxDev;
    double criticalSlope;
  public:
    SanFrancisco(double weight, std::unique_ptr<SingleChannelMap> elevationMap, double length, double maxDev, double criticalSlope);
    ~SanFrancisco() {}
    std::pair<double, double> evaluate(size_t node, double angle, bool highway, const CityLayout &city) override;
};

class RoadConstraint {
  public:
    virtual bool &apply(Extension &ex, const CityLayout &city) const = 0;
    virtual ~RoadConstraint() {}
};

#endif

