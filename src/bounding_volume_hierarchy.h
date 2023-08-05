#pragma once
#include "common.h"
#include <array>
#include <framework/ray.h>
#include <vector>


struct Node {
    glm::vec3 lBound = glm::vec3(1000.0f);
    glm::vec3 uBound = glm::vec3(-1000.0f);
    bool isInterior = false;
    std::vector<int> subs;
    int level;
};

struct TriangleBVH {
    int meshIndex;
    glm::uvec3 triangleMesh;
    glm::vec3 centroid;
};

// Forward declaration.
struct Scene;

class BoundingVolumeHierarchy {
public:
    // Constructor. Receives the scene and builds the bounding volume hierarchy.
    BoundingVolumeHierarchy(Scene* pScene, const Features& f);

    void recursionBuilder(std::vector<TriangleBVH> triangles, int level, size_t curNodeIndex);
    void recursionSAHBuilder(std::vector<TriangleBVH> triangles, int level, size_t curNodeIndex);
    int getIndexOfChildNode(std::vector<TriangleBVH> t1Triangles, int level);
    float computeCost(Node& node, size_t splitVal, std::vector<TriangleBVH> triangles);
    // Return how many levels there are in the tree that you have constructed.
    [[nodiscard]] int numLevels() const;

    // Return how many leaf nodes there are in the tree that you have constructed.
    [[nodiscard]] int numLeaves() const;

    // Visual Debug 1: Draw the bounding boxes of the nodes at the selected level.
    void debugDrawLevel(int level);

    // Visual Debug 2: Draw the triangles of the i-th leaf
    void debugDrawLeaf(int leafIdx);

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo, const Features& features) const;
    bool intersectRecursion(const Features& features, Ray& ray, HitInfo& hitInfo, const Node& node, std::vector<Node>& boxes) const;
    void drawBoxes(std::vector<Node>& boxes, const Node& target) const;
    bool containsTriangle(Node& n, const Node& target) const;

    int m_numLevels = 0;
    int m_numLeaves = 0;
    Scene* m_pScene;
    std::vector<Node> listOfNodes;
};