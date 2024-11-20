#include <vector>
#include <optional>
#include <utility>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "cityLayout.h"
#include "vect2.h"
#include "util.h"


std::optional<size_t> CityLayout::block(size_t edge) const {
    if (edges[edge].outskirts) {
        return std::nullopt;
    }
    return std::optional<size_t>{edges[edge].block};
}


void CityLayout::nodeEraseClosestExt(size_t node, double angle) {
    if (nodes[node].potentialExt.size() > 0) {
        int closest_i = 0;
        double closest_diff = angleDiff(nodes[node].potentialExt[0], angle);
        for (int i = 1; i < nodes[node].potentialExt.size(); ++i) {
            double diff = angleDiff(nodes[node].potentialExt[i], angle);
            if (diff < closest_diff) {
                closest_i = i;
                closest_diff = diff;
            }
        }
        nodes[node].potentialExt.erase(nodes[node].potentialExt.begin() + closest_i);
    }
}


bool CityLayout::nodeAdjacent(size_t a, size_t b) const {
    size_t start = nodeEdge(a);
    size_t curr = start;
    do {
        if (tip(curr) == b) return true;
        curr = nextOutEdgeCW(curr);
    } while (curr != start);
    return false;
}

size_t CityLayout::closestOutEdgeCW(size_t node, double angle) const {
    size_t start = nodeEdge(node);
    size_t curr = start;
    do {
        if (angleBetweenCCW((nodePos(tip(nextOutEdgeCW(curr))) - nodePos(node)).worldAngle(), angle, (nodePos(tip(curr)) - nodePos(node)).worldAngle())) return nextOutEdgeCW(curr);
        curr = nextOutEdgeCW(curr);
    } while (curr != start);
}

size_t CityLayout::majorDegree(size_t node) const {
    size_t curr = nodeEdge(node);
    size_t count = 0;
    do {
        if (roadIsHighway(road(curr))) ++count;
        curr = nextOutEdgeCW(curr);
    } while (curr != nodeEdge(node));
    return count;
}

double areaCCW(const std::vector<Vect2> &points) {
    double minY = points[0].getY();
    for (const auto &v : points) {
        if (v.getY() < minY) {
            minY = v.getY();
        }
    }
    double sum = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        size_t inc = (i + 1) % points.size();
        sum += (points[i].getX() - points[inc].getX()) * (points[i].getY() + points[inc].getY() - 2 * minY);
    }
    return sum / 2;
}

bool CityLayout::validCycle(size_t edge) const {
    size_t curr = edge;
    std::vector<Vect2> points;
    do {
        points.push_back(nodePos(tip(curr)));
        curr = next(curr);
    } while (curr != edge);
    return areaCCW(points) >= 0;
}


CityLayout::CityLayout(Vect2 a, Vect2 b, double snapDist, double extendDist, double minLength, double minAngle): edges{std::vector<HalfEdge>()}, nodes{std::vector<Node>()}, roads{std::vector<RoadSegment>()}, blocks{std::vector<Block>()}, snapDist{snapDist}, extendDist{extendDist}, minLength{minLength}, minAngle{minAngle} {
    edges.push_back(HalfEdge{1, 1, 1, 0, 1, 0, true});
    edges.push_back(HalfEdge{0, 0, 0, 0, 0, 0, true});
    roads.push_back(RoadSegment{false, 5, true, 0});
    double angleA = (b-a).worldAngle();
    double angleB = (a-b).worldAngle();
    nodes.push_back(Node{a, true, std::vector<double>(3), std::vector<double>{angleA}, 0});
    nodes.push_back(Node{b, true, std::vector<double>(3), std::vector<double>{angleB}, 1});
    for (int i = 0; i < 3; ++i) {
        nodes[0].potentialExt[i] = angleA + M_PI_2 * (i+1);
        nodes[1].potentialExt[i] = angleB + M_PI_2 * (i+1);
    }
}

size_t CityLayout::addNode(Vect2 pos, bool highway, size_t edge) {
    double angle = (nodePos(tip(edge)) - pos).worldAngle();
    nodes.push_back(Node{pos, highway, std::vector<double>(3), std::vector<double>{angle}, edge});
    for (int i = 0; i < 3; ++i) {
        nodes.back().potentialExt[i] = angle + M_PI_2 * (i+1);
    }
    return nodes.size() - 1;
}


// returns the index of the half edge on the new road segment pointing to the new node
size_t CityLayout::extend(size_t base, Vect2 pos, bool highway) {
    // create new half edges and twin them
    edges.push_back(HalfEdge{});
    edges.push_back(HalfEdge{});
    size_t forward = edges.size() - 2;
    size_t rev = edges.size() - 1;
    edges[forward].twin = rev;
    edges[rev].twin = forward;

    // set nexts and prevs
    edges[forward].next = rev;
    edges[rev].prev = forward;
    double angle = (pos - nodePos(base)).worldAngle();
    size_t faceEdge = closestOutEdgeCW(base, angle);
    edges[rev].next = faceEdge;
    edges[prev(faceEdge)].next = forward;
    edges[forward].prev = prev(faceEdge);
    edges[faceEdge].prev = rev;

    // set tips roads and blocks
    edges[rev].tip = base; // must be done before making new node
    size_t newNode = addNode(pos, highway, rev);
    edges[forward].tip = newNode;
    roads.push_back(RoadSegment{false, 5, highway, forward});
    size_t newRoad = roads.size() - 1;
    edges[forward].road = newRoad;
    edges[rev].road = newRoad;
    std::optional<size_t> existingBlock = block(faceEdge);
    if (existingBlock) {
        edges[forward].block = *existingBlock;
        edges[rev].block = *existingBlock;
        edges[forward].outskirts = false;
        edges[rev].outskirts = false;
    } else {
        edges[forward].outskirts = true;
        edges[rev].outskirts = true;
    }

    // update extension angles
    nodeEraseClosestExt(base, angle);
    nodes[base].lockedExt.push_back(angle);

    return forward;
}

// returns the index of the half edge on the new road segment pointing to b
size_t CityLayout::connect(size_t a, size_t b, bool highway) {

    // create new half edges and twin them
    edges.push_back(HalfEdge{});
    edges.push_back(HalfEdge{});
    size_t forward = edges.size() - 2;
    size_t rev = edges.size() - 1;
    edges[forward].twin = rev;
    edges[rev].twin = forward;

    // set nexts and prevs
    double angleA = (nodePos(b) - nodePos(a)).worldAngle();
    double angleB = (nodePos(a) - nodePos(b)).worldAngle();
    size_t faceEdgeA = closestOutEdgeCW(a, angleA);
    size_t faceEdgeB = closestOutEdgeCW(b, angleB);
    edges[forward].next = faceEdgeB;
    edges[forward].prev = prev(faceEdgeA);
    edges[rev].next = faceEdgeA;
    edges[rev].prev = prev(faceEdgeB);
    edges[prev(faceEdgeA)].next = forward;
    edges[prev(faceEdgeB)].next = rev;
    edges[faceEdgeB].prev = forward;
    edges[faceEdgeA].prev = rev;

    // set tips and road
    edges[forward].tip = b;
    edges[rev].tip = a;
    roads.push_back(RoadSegment{false, 5, highway, forward});
    size_t newRoad = roads.size() - 1;
    edges[forward].road = newRoad;
    edges[rev].road = newRoad;

    // convert nodes to highways
    if (highway) {
        nodes[a].highway = true;
        nodes[b].highway = true;
    }

    // update extension angles
    nodeEraseClosestExt(a, angleA);
    nodeEraseClosestExt(b, angleB);
    nodes[a].lockedExt.push_back(angleA);
    nodes[b].lockedExt.push_back(angleB);

    // set blocks
    bool validA = validCycle(rev);
    bool validB = validCycle(forward);

    edges[forward].outskirts = false;
    edges[rev].outskirts = false;

    if (!validA && !validB) {
        edges[forward].outskirts = true;
        edges[rev].outskirts = true;
    } else {
        size_t newBlockEdge;
        if (validA && validB) {
            blocks[*block(faceEdgeB)].edge = faceEdgeB;
            newBlockEdge = faceEdgeA;
        } else if (validA) {
            edges[forward].outskirts = true;
            newBlockEdge = faceEdgeA;
        } else {
            edges[rev].outskirts = true;
            newBlockEdge = faceEdgeB;
        }
        blocks.push_back(Block{newBlockEdge});
        size_t newBlock = blocks.size() - 1;
        size_t curr = newBlockEdge;
        do {
            edges[curr].block = newBlock;
            edges[curr].outskirts = false;
            curr = next(curr);
        } while (curr != newBlockEdge);
    }

    return forward;
}

// returns the index of the new node
size_t CityLayout::subdivide(size_t rd, Vect2 pos) {
    // create new half edges and twin them
    edges.push_back(HalfEdge{});
    edges.push_back(HalfEdge{});
    size_t forward = edges.size() - 2;
    size_t rev = edges.size() - 1;
    edges[forward].twin = rev;
    edges[rev].twin = forward;

    // set nexts and prevs
    if (degree(tip(roadEdge(rd))) == 1) {
        edges[forward].next = rev;
        edges[rev].prev = forward;
    } else {
        edges[forward].next = next(roadEdge(rd));
        edges[rev].prev = prev(twin(roadEdge(rd)));
        edges[next(roadEdge(rd))].prev = forward;
        edges[prev(twin(roadEdge(rd)))].next = rev;
    }
    edges[forward].prev = roadEdge(rd);
    edges[rev].next = twin(roadEdge(rd));
    edges[roadEdge(rd)].next = forward;
    edges[twin(roadEdge(rd))].prev = rev;

    // fix node edge (necessary in some cases)
    nodes[tip(roadEdge(rd))].edge = rev;

    // set tips
    edges[forward].tip = tip(roadEdge(rd)); // must be done before making new node
    
    size_t newNode = addNode(pos, roadIsHighway(rd), twin(roadEdge(rd)));
    // update extension angles
    double angle = (nodePos(tip(forward)) - nodePos(newNode)).worldAngle();
    nodeEraseClosestExt(newNode, angle);
    nodes[newNode].lockedExt.push_back(angle);

    // continue setting tips
    edges[rev].tip = newNode;
    edges[roadEdge(rd)].tip = newNode;
    

    // set their roads and blocks
    roads.push_back(RoadSegment{false, 5, roadIsHighway(rd), forward});
    size_t newRoad = roads.size() - 1;
    edges[forward].road = newRoad;
    edges[rev].road = newRoad;

    std::optional<size_t> blockF = block(roadEdge(rd));
    std::optional<size_t> blockR = block(twin(roadEdge(rd)));
    if (blockF) {
        edges[forward].block = *blockF;
        edges[forward].outskirts = false;
    } else {
        edges[forward].outskirts = true;
    }
    if (blockR) {
        edges[rev].block = *blockR;
        edges[rev].outskirts = false;
    } else {
        edges[rev].outskirts = true;
    }

    return newNode;
}


// returns the index of the road segment and the position of intersection
std::optional<std::pair<size_t, Vect2>> CityLayout::getFirstIntersection(size_t base, size_t ignore, Vect2 target) const {
    double minDist = -1;
    Vect2 basePos = nodePos(base);
    std::optional<std::pair<size_t, Vect2>> minIntersection = std::nullopt;
    Coeff extCoeffs = getCoeff(basePos, target);
    for (size_t i = 0; i < roads.size(); ++i) {
        // do not intersect with incident edges
        size_t curr = nodeEdge(base);
        bool skip = false;
        do {
            if (road(curr) == i) {
                skip = true;
                break;
            }
            curr = nextOutEdgeCW(curr);
        } while (curr != nodeEdge(base));
        curr = nodeEdge(ignore);
        do {
            if (road(curr) == i) {
                skip = true;
                break;
            }
            curr = nextOutEdgeCW(curr);
        } while (curr != nodeEdge(ignore));
        if (skip) {
            continue;
        }

        Vect2 otherA = nodePos(tip(roads[i].edge));
        Vect2 otherB = nodePos(root(roads[i].edge));
        std::optional<Vect2> result = intersect(extCoeffs, getCoeff(otherA, otherB));
        if (result && std::min(basePos.getX(), target.getX()) <= result->getX() && result->getX() <= std::max(basePos.getX(), target.getX()) && std::min(basePos.getY(), target.getY()) <= result->getY() && result->getY() <= std::max(basePos.getY(), target.getY()) && std::min(otherA.getX(), otherB.getX()) <= result->getX() && result->getX() <= std::max(otherA.getX(), otherB.getX()) && std::min(otherA.getY(), otherB.getY()) <= result->getY() && result->getY() <= std::max(otherA.getY(), otherB.getY())) {
            double dist = (*result - basePos).getNorm();
            if (minDist < 0 || dist < minDist) {
                minDist = dist;
                minIntersection = std::optional<std::pair<size_t, Vect2>>{std::pair<size_t, Vect2>{i, *result}};
            }
        }
    }
    return minIntersection;
}

std::optional<size_t> CityLayout::snapNode(Vect2 pos, size_t ignore) const {
    std::optional<size_t> candidate = std::nullopt;
    double minDist;
    for (size_t i = 0; i < nodes.size(); ++i) {
        double dist = (pos - nodePos(i)).getNorm();
        if (i != ignore && dist < snapDist) {
            if (!candidate || snapDist < minDist) {
                candidate = std::optional<size_t>{i};
                minDist = dist;
            }
        }
    }
    return candidate;
}


bool CityLayout::checkMinAngle(size_t node, Vect2 ex) const {
    size_t curr = nodeEdge(node);
    do {
        if (ex.getAngle(nodePos(tip(curr)) - nodePos(node)) <= minAngle) return true;
        curr = nextOutEdgeCW(curr);
    } while(curr != nodeEdge(node));
    return false;
}

// returns the index of the half edge on the new road segment pointing to the new node
std::optional<size_t> CityLayout::extendNode(size_t base, Vect2 pos, bool highway) {
    // KNOWN BUG. EXTENDING ALONG EXISTING LINE BREAKS THINGS. FIXING THIS COULD BE AS SIMPLE AS PROHIBITING SUPER TIGHT ANGLED EXTENSIONS.
    // KNOWN BUG. THERES SOMETHING WRONG WITH EITHER INCREDIBLY SHORT LINES OR INCREDIBLY SHORT SUBDIVISIONS. THIS CAN PROBABLY BE FIXED WITH A COMBINATION OF SNAPPING AND MINIMUM SEGMENT LENGTH
    // THESE PROBABLY ALSO APPLY TO CONNECTING

    //FIXED

    // check min length + min angle
    double length = (pos - nodePos(base)).getNorm();
    if (length <= minLength || checkMinAngle(base, (pos - nodePos(base)))) return std::nullopt;

    // apply extension optimization to try to avoid future minLength segments
    Vect2 extOptTip = (pos - nodePos(base)).normalize().scale(length + extendDist) + nodePos(base);

    std::optional<std::pair<size_t, Vect2>> in = getFirstIntersection(base, base, extOptTip);
    if (!in) {
        // check for snap
        std::optional<size_t> snap = snapNode(pos, base);
        if (snap) {
            return connectNodes(base, *snap, highway);
        } else {
            return extend(base, pos, highway);
        }
    }
    // check for snap
    std::optional<size_t> snap = snapNode(in->second, base);
    if (snap) {
        return connectNodes(base, *snap, highway);
    } else if ((in->second - nodePos(base)).getNorm() > minLength) { // check minlength
        size_t target = subdivide(in->first, in->second);
        return connect(base, target, highway);
    }
    return std::nullopt;
}


// returns the index of the half edge on the new road segment pointing to target node
std::optional<size_t> CityLayout::connectNodes(size_t base, size_t target, bool highway) {
    if (base != target && !nodeAdjacent(base, target) && potentialExt(target).size() > 0) { // consider also checking base potentialExts for invariant sake (would prlly also do this in extend). This is also where we would check alignment angle with potential connection.

        // check min length + min angle
        double length = (nodePos(target) - nodePos(base)).getNorm();
        if (length <= minLength || checkMinAngle(base, (nodePos(target) - nodePos(base))) || checkMinAngle(target, (nodePos(base) - nodePos(target)))) return std::nullopt;

        std::optional<std::pair<size_t, Vect2>> in = getFirstIntersection(base, target, nodePos(target));
        if (!in) {
            return std::optional<size_t>{connect(base, target, highway)};
        }
        /*
        size_t prunedTip = subdivide(in->first, in->second);
        
        return std::optional<size_t>{connect(base, prunedTip, highway)};
        */
    }
    return std::nullopt;
}

std::vector<Vect2> CityLayout::blockPoly(size_t block) const { // make it so we dont recompute this (save it in block)
    size_t curr = blockEdge(block);
    std::vector<Vect2> points;
    do {
        Vect2 a = nodePos(root(curr));
        Vect2 b = nodePos(tip(curr));
        Vect2 c = nodePos(tip(next(curr)));
        Vect2 offsetA = Vect2(-(b-a).getY(), (b-a).getX()).normalize().scale(width(road(curr)) / 2);
        Vect2 offsetC = Vect2(-(c-b).getY(), (c-b).getX()).normalize().scale(width(road(next(curr))) / 2);
        std::optional<Vect2> result = intersect(getCoeff(a+offsetA, b+offsetA), getCoeff(b+offsetC, c+offsetC));
        if (result) {
            points.push_back(*result);
        } else {
            points.push_back(b + offsetA);
            if ((b-a).dot(c-b) < 0) {
                points.push_back(b + offsetC);
            }
        }
        curr = next(curr);
    } while (curr != blockEdge(block));
    
    return points;
}

