#include <optional>
#include <cmath>
#include <utility>
#include <cstdlib>
#include <iostream>
#include "planner.h"
#include "cityLayout.h"
#include "vect2.h"
#include "util.h"

// has a city state
    // city state has a list of planned roads, built roads, intersections, and lots


//planner step

    //sample

    // extend

    // goals

    // constraints

    // check block if necessary

    // update traffic and land use

    // BUILD


// a thought on tunnels. Maybe they are like roads that get extended to negate terrain slope

Planner::Planner(CityLayout city, double growthSpeed): city{city}, growthSpeed{growthSpeed}, highwayMode{true}, quarter{std::vector<size_t>()}, quarterEdge{0}, sampleGoals{std::vector<std::unique_ptr<SampleGoal>>()}, extensionGoals{std::vector<std::unique_ptr<ExtensionGoal>>()}, roadConstraints{std::vector<std::unique_ptr<RoadConstraint>>()} {}

size_t Planner::nextHighway(size_t edge) const {
    size_t curr = city.next(edge);
    while (!city.roadIsHighway(city.road(curr))) {
        curr = city.nextOutEdgeCW(curr);
    }
    return curr;
}

std::vector<std::pair<double, double>> Planner::getQuarterConstraints(size_t node) const {
    std::vector<std::pair<double, double>> constraints;
    size_t curr = quarterEdge;
    do {
        if (city.tip(curr) == node) {
            constraints.emplace_back((city.nodePos(city.tip(nextHighway(curr))) - city.nodePos(node)).worldAngle(), (city.nodePos(city.root(curr)) - city.nodePos(node)).worldAngle());
        }
        curr = nextHighway(curr);
    } while (curr != quarterEdge);
    return constraints;
}

std::optional<double> Planner::getExtensionAngle(size_t node) { // ASSUMES NODE IS EITHER A HIGHWAY OR A MEMBER OF QUARTER AND HAS AT LEAST ONE POTENTIAL EXTENSION
    std::vector<std::pair<double, double>> constraints;
    if (!highwayMode && city.nodeIsHighway(node)) {
        constraints = getQuarterConstraints(node);
    }
    if ((!highwayMode && city.nodeIsHighway(node) && constraints.size() == 0)) {
        return std::nullopt;
    }
    if (city.degree(node) == 1) {
        // pick straightest extension
        double baseAngle = city.lockedExt(node)[0];
        double straightest = city.potentialExt(node)[0];
        double straightestDiff = angleDiff(baseAngle + M_PI, straightest);
        for (auto pot : city.potentialExt(node)) {
            double potDiff = angleDiff(baseAngle + M_PI, pot);
            if (potDiff < straightestDiff) {
                straightest = pot;
                straightestDiff = potDiff;
            }
        }
        return std::optional<double>{straightest};
    } else {
        if (highwayMode || !city.nodeIsHighway(node)) {
            // pick random extension from all extensions
            int r = std::rand() % city.potentialExt(node).size();
            return std::optional<double>{city.potentialExt(node)[r]};
        } else {
            // constrain all potential exts. pick random one
            std::vector<double> angles;
            for (auto a : city.potentialExt(node)) {
                for (auto con : constraints) {
                    if (angleBetweenCCW(con.first, a, con.second)) {
                        angles.push_back(a);
                    }
                }
            }
            if (angles.size() > 0) {
                int r = std::rand() % angles.size();
                return std::optional<double>{angles[r]};
            }
        }
    }
    return std::nullopt;
}


bool Planner::isValidSample(size_t node) {
    if (city.potentialExt(node).size() > 0) {
        if (highwayMode) {
            return city.nodeIsHighway(node);
        } else {
            if (city.nodeIsHighway(node)) {
                if (getExtensionAngle(node)) {
                    return true;
                }
            } else {
                for (auto n : quarter) {
                    if (node == n) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

std::optional<size_t> Planner::roadPlanningSample() {
    std::vector<double> weights;
    std::vector<size_t> nodes;
    double sum = 0.0;
    for (size_t i = 0; i < city.numNodes(); ++i) {
        if (isValidSample(i)) {
            for (auto &goal : sampleGoals) { // sum is built in currently. We could have a "weighted sum" goal instead
                weights.push_back(goal->weigh(i, highwayMode, city) * goal->weight);
                nodes.push_back(i);
            }
            sum += weights.back();
        }
    }
    double r = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) * sum;
    double prefix = 0.0;
    for (size_t i = 0; i < nodes.size(); ++i) {
        prefix += weights[i];
        if (r < prefix || i == nodes.size() - 1) {
            return std::optional<size_t>{nodes[i]};
        }
    }
    return std::nullopt;
}


std::pair<double, double> Planner::applyExtensionGoals(size_t node, double base_angle) {
    double sum_weights = 0.0;
    double sum_dev = 0.0;
    double sum_len = 0.0;
    for (auto &goal : extensionGoals) {
        std::pair<double, double> params = goal->evaluate(node, base_angle, highwayMode, city);
        sum_dev += params.first;
        sum_len += params.second;
        sum_weights += goal->weight;
    }
    double angle = 0;
    double length = 0;
    if (sum_weights != 0) {
        angle = base_angle + sum_dev / sum_weights;
        length = sum_len / sum_weights;
    }

    return std::pair<double, double>{angle, length};
}


void Planner::step() {

    // BUG! minor streets are escaping their quarters! Disasterous!

    int num_samples = std::max(int(city.numNodes() * growthSpeed), 1); // in future versions this may be abstracted to allow more ways to manipulate growth
    for (int i = 0; i < num_samples; ++i) {
        
        // precompute data for relevant sample goals.
        for (auto &goal : sampleGoals) {
            try {
                PrecomputeGoal &pg = dynamic_cast<PrecomputeGoal&>(*goal);
                pg.update(city);
            }
            catch (std::bad_cast &) {}
        }
        
        // get a sample
        std::optional<size_t> toExtend = roadPlanningSample();

        // if no sample either switch mode or fail
        if (!toExtend) {
            if (highwayMode) {
                throw NoSample{};
            }
            //std::cout << "No sample. Switching modes." << std::endl;
            highwayMode = true;
            --i;
            continue;
        }

        // apply extensional goals
        double baseAngle = *getExtensionAngle(*toExtend);
        std::pair<double, double> params = applyExtensionGoals(*toExtend, baseAngle);
        double angle = params.first;
        double length = params.second;

        //need to make sure extension doesnt push road out of quarter bounds
        if (!highwayMode && city.nodeIsHighway(*toExtend)) {
            std::vector<std::pair<double, double>> constraints = getQuarterConstraints(*toExtend);
            bool bounded = false;
            for (auto con : constraints) {
                if (angleBetweenCCW(con.first, angle, con.second)) {
                    bounded = true;
                    break;
                }
            }
            if (!bounded) {
                //std::cout << "Out of Quarter failure: " << *toExtend << ", " << baseAngle << ", " << angle << std::endl;
                city.nodeEraseClosestExt(*toExtend, baseAngle);
                --i;
                continue;
            }
        }
        
        Extension ex = Extension{*toExtend, city.nodePos(*toExtend) + Vect2(std::cos(angle), std::sin(angle)).scale(length), highwayMode};

        // apply constraints
        bool failed = false;
        for (auto &constraint : roadConstraints) {
            if (constraint->apply(ex, city)) {
                failed = true;
                break;
            }
        }
        if (failed) {
            city.nodeEraseClosestExt(*toExtend, baseAngle);
            --i;
            continue;
        }
        
        std::optional<size_t> newEdge = city.extendNode(ex.base, ex.tip, ex.highway);

        if (!newEdge) {
            //std::cout << "Invariant failure." << std::endl;
            city.nodeEraseClosestExt(*toExtend, baseAngle);
            --i;
            continue;
        }

        if (highwayMode) {
            // if we created a new quarter (does not include splitting quarters) then switch modes and set the quarter edge
            std::optional<size_t> blockF = city.block(*newEdge);
            std::optional<size_t> blockR = city.block(city.twin(*newEdge));
            if (blockF && !blockR) {
                //std::cout << "New Quarter. Switching Modes." << std::endl;
                highwayMode = false;
                quarter.clear();
                quarterEdge = *newEdge;
            } else if (blockR && !blockF) {
                //std::cout << "New Quarter. Switching Modes." << std::endl;
                highwayMode = false;
                quarter.clear();
                quarterEdge = city.twin(*newEdge);
            }
        } else {
            // if we added a minor node put it in the current quarter
            if (!city.nodeIsHighway(city.tip(*newEdge))) {
                bool dup = false;
                for (auto node : quarter) {
                    if (node == city.tip(*newEdge)) dup = true;
                }
                if (!dup) quarter.push_back(city.tip(*newEdge));
            }
        }


        // check extended intersection then add.

        // try new methods in the future but for now just do one pass over the constraint stack
        // use illegal area rules from og parish muller article

        // add road segment to data structure, maintain halfedges, update blocks, intersections, give it a name, etc


        // consume an extension direction when connecting maybe? if no valid extenion in a certain threshold angle then reject? (if this rejection feature is in place then a subdivision may need to be undone)
        // maybe add a feature to not destroy peoples lots / buildings
    }

    // traffic and land use

    // district / heuristic based traffic sampling

    // high traffic streets and or highways get cool names
}

