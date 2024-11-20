#ifndef _PLANNER_H_
#define _PLANNER_H_

#include <vector>
#include <optional>
#include <utility>
#include <memory>
#include "vect2.h"
#include "cityLayout.h"
#include "goal.h"


class NoSample{};

class Planner {
    CityLayout city;
    double growthSpeed;
    bool highwayMode;
    std::vector<size_t> quarter;
    size_t quarterEdge;
    size_t nextHighway(size_t edge) const;
    std::vector<std::pair<double, double>> getQuarterConstraints(size_t node) const;
    std::optional<double> getExtensionAngle(size_t node);
    bool isValidSample(size_t node);
    std::optional<size_t> roadPlanningSample();
    std::pair<double, double> applyExtensionGoals(size_t node, double base_angle);
  public:
    std::vector<std::unique_ptr<SampleGoal>> sampleGoals;
    std::vector<std::unique_ptr<ExtensionGoal>> extensionGoals;
    std::vector<std::unique_ptr<RoadConstraint>> roadConstraints;
    Planner(CityLayout city, double growthSpeed);
    inline const CityLayout &getCity() const {return city; }
    void step();
};

#endif

