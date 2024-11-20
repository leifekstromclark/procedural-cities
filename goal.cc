#include <vector>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <random>
#include "goal.h"
#include "vect2.h"
#include "cityLayout.h"
#include "util.h"


IntersectionRatio::IntersectionRatio(double weight, double targetRatio): SampleGoal{weight}, currentRatio{0.0}, targetRatio{targetRatio} {}

double IntersectionRatio::weigh(size_t node, bool highway, const CityLayout &city) const {
    if (highway) {
        if (currentRatio < targetRatio) {
            if (city.majorDegree(node) == 1) return 1.0;
            if (city.majorDegree(node) == 2) return 0.0;
            if (city.majorDegree(node) == 3) return 0.0;
        } else {
            if (city.majorDegree(node) == 1) return 0.0;
            if (city.majorDegree(node) == 2) return 0.5;
            if (city.majorDegree(node) == 3) return 1.0;
        }
    }
    return 1.0;
}

void IntersectionRatio::update(const CityLayout &city) {
    size_t deg2 = 0;
    size_t deg4 = 0;
    for (size_t i = 0; i < city.numNodes(); ++i) {
        if (city.nodeIsHighway(i)) {
            if (city.majorDegree(i) == 2) {
                ++deg2;
            } else if (city.majorDegree(i) == 4) {
                ++deg4;
            }
        }
    }
    if (deg4 == 0) {
        currentRatio = targetRatio + 1;
    } else {
        currentRatio = static_cast<double>(deg2) / static_cast<double>(deg4);
    }
}

RandomDeviation::RandomDeviation(double weight, double meanLength, double devLength, double devAngle): ExtensionGoal{weight}, generator{}, lengthDist{std::normal_distribution<double>(meanLength, devLength)}, angleDist{std::normal_distribution<double>(0.0, devAngle)} {}

std::pair<double, double> RandomDeviation::evaluate(size_t node, double angle, bool highway, const CityLayout &city) {
    return std::pair<double, double>{angleDist(generator), lengthDist(generator)};
}


Paris::Paris(double weight, Vect2 center, double perpLength, double paraLength): ExtensionGoal{weight}, center{center}, perpLength{perpLength}, paraLength{paraLength} {}

std::pair<double, double> Paris::evaluate(size_t node, double angle, bool highway, const CityLayout &city) {
    Vect2 toCenter = (center - city.nodePos(node));
    if (toCenter.getNorm() == 0) {
        return std::pair<double, double>{0.0, perpLength};
    }
    double towardsCenter = toCenter.worldAngle();
    
    double closest = towardsCenter;
    double closestDiff = angleDiff(towardsCenter, angle);;
    double length = perpLength;
    for (int i = 1; i < 4; ++i) {
        double diff = angleDiff(towardsCenter + M_PI_2 * i, angle);
        if (diff < closestDiff) {
            closest = towardsCenter + M_PI_2 * i;
            closestDiff = diff;
            if (i == 2) {
                length = perpLength;
            } else {
                length = paraLength;
            }
        }
    }
    return std::pair<double, double>{angleDiffCCW(closest, angle), length};
}

Manhattan::Manhattan(double weight, double blockAngle, double perpLength, double paraLength): ExtensionGoal{weight}, blockAngle{blockAngle}, perpLength{perpLength}, paraLength{paraLength} {}

std::pair<double, double> Manhattan::evaluate(size_t node, double angle, bool highway, const CityLayout &city) {
    double closest = blockAngle;
    double closestDiff = angleDiff(blockAngle, angle);
    double length = paraLength;
    for (int i = 1; i < 4; ++i) {
        double diff = angleDiff(blockAngle + M_PI_2 * i, angle);
        if (diff < closestDiff) {
            closest = blockAngle + M_PI_2 * i;
            closestDiff = diff;
            if (i == 2) {
                length = paraLength;
            } else {
                length = perpLength;
            }
        }
    }
    return std::pair<double, double>{angleDiffCCW(closest, angle), length};
}

SanFrancisco::SanFrancisco(double weight, std::unique_ptr<SingleChannelMap> elevationMap, double length, double maxDev, double criticalSlope): ExtensionGoal{weight}, elevationMap{std::move(elevationMap)}, length{length}, maxDev{maxDev}, criticalSlope{criticalSlope} {}

std::pair<double, double> SanFrancisco::evaluate(size_t node, double angle, bool highway, const CityLayout &city) {
    try {
        Vect2 pos = city.nodePos(node);
        Vect2 end = pos + Vect2(std::cos(angle), std::sin(angle)).scale(length);
        double slope = (elevationMap->sample(end) - elevationMap->sample(pos)) / (end - pos).getNorm();
        if (std::abs(slope) >= criticalSlope) {
            double dzdx = elevationMap->sample(pos + Vect2(0.1, 0)) - elevationMap->sample(pos + Vect2(-0.1, 0));
            double dzdy = elevationMap->sample(pos + Vect2(0, 0.1)) - elevationMap->sample(pos + Vect2(0, -0.1));
            Vect2 grad = Vect2(dzdx, dzdy).normalize();
            if (grad.getNorm() > 0) {
                double gradAngle = grad.worldAngle();
                double closest = gradAngle;
                double closestDiff = angleDiff(gradAngle, angle);
                for (int i = 1; i < 4; ++i) {
                    double diff = angleDiff(gradAngle + M_PI_2 * i, angle);
                    if (diff < closestDiff) {
                        closest = gradAngle + M_PI_2 * i;
                        closestDiff = diff;
                    }
                }
                double signedDiff = angleDiffCCW(closest, angle);
                if (closestDiff < maxDev) {
                    return std::pair<double, double>{signedDiff, length};
                }
                return std::pair<double, double>{maxDev * (signedDiff < 0 ? -1 : 1) , length};
            }
        }
    }
    catch (OutOfBounds &) {}
    return std::pair<double, double>{0.0, length};
}

