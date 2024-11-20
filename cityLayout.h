#include <vector>
#include <optional>
#include <utility>
#include "vect2.h"

#ifndef _CITYLAYOUT_H_
#define _CITYLAYOUT_H_

struct RoadSegment {
    bool built;
    int width;
    bool highway;
    size_t edge;
};

struct Block {
  size_t edge;
};

struct Node {
  Vect2 pos;
  bool highway;
  std::vector<double> potentialExt;
  std::vector<double> lockedExt;
  size_t edge;
};

struct HalfEdge {
  size_t next;
  size_t prev;
  size_t twin;
  size_t road;
  size_t tip;
  size_t block;
  bool outskirts;
};


class CityLayout {
    std::vector<HalfEdge> edges;
    std::vector<Node> nodes;
    std::vector<RoadSegment> roads;
    std::vector<Block> blocks;
    double snapDist;
    double extendDist;
    double minLength;
    double minAngle;
    bool validCycle(size_t edge) const;
    bool checkMinAngle(size_t node, Vect2 ex) const;
    std::optional<size_t> snapNode(Vect2 pos, size_t ignore) const;
    size_t addNode(Vect2 pos, bool highway, size_t edge);
    // returns the index of the new node
    size_t subdivide(size_t rd, Vect2 pos);
    // returns the index of the half edge on the new road segment pointing to the new node
    size_t extend(size_t base, Vect2 pos, bool highway);
    // returns the index of the half edge on the new road segment pointing to b
    size_t connect(size_t a, size_t b, bool highway);
    // returns the index of the road segment and the position of intersection
    std::optional<std::pair<size_t, Vect2>> getFirstIntersection(size_t base, size_t ignore, Vect2 target) const;
  public:
    CityLayout(Vect2 a, Vect2 b, double snapDist, double extendDist, double minLength, double minAngle);
    inline size_t numNodes() const {return nodes.size(); }
    inline size_t numRoads() const {return roads.size(); }
    inline size_t numBlocks() const {return blocks.size(); }
    inline size_t next(size_t edge) const {return edges[edge].next; }
    inline size_t prev(size_t edge) const {return edges[edge].prev; }
    inline size_t twin(size_t edge) const {return edges[edge].twin; }
    inline size_t tip(size_t edge) const {return edges[edge].tip; }
    inline size_t root(size_t edge) const {return tip(twin(edge)); }
    inline size_t road(size_t edge) const {return edges[edge].road; }
    std::optional<size_t> block(size_t edge) const;
    inline size_t degree(size_t node) const {return nodes[node].lockedExt.size(); }
    size_t majorDegree(size_t node) const;
    inline size_t nodeEdge(size_t node) const {return nodes[node].edge; }
    inline size_t roadEdge(size_t road) const {return roads[road].edge; }
    inline size_t blockEdge(size_t block) const {return blocks[block].edge; }
    inline size_t width(size_t road) const {return roads[road].width; }
    inline Vect2 nodePos(size_t node) const {return nodes[node].pos; }
    inline std::vector<double> &potentialExt(size_t node) {return nodes[node].potentialExt; }
    inline const std::vector<double> &lockedExt(size_t node) const {return nodes[node].lockedExt; }
    inline bool roadIsHighway(size_t road) const {return roads[road].highway; }
    inline bool nodeIsHighway(size_t node) const {return nodes[node].highway; }
    void nodeEraseClosestExt(size_t node, double angle);
    bool nodeAdjacent(size_t a, size_t b) const;
    // returns the index of a half edge
    size_t closestOutEdgeCW(size_t node, double angle) const;
    inline size_t nextOutEdgeCW(size_t edge) const {return next(twin(edge)); };
    // returns the index of the half edge on the new road segment pointing to the new node
    std::optional<size_t> extendNode(size_t base, Vect2 pos, bool highway);
    // returns the index of the half edge on the new road segment pointing to target
    std::optional<size_t> connectNodes(size_t base, size_t target, bool highway);
    std::vector<Vect2> blockPoly(size_t block) const;
};

#endif

