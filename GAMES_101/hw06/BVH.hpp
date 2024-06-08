//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <ctime>
#include <memory>
#include <span>
#include <vector>

#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Object.hpp"
#include "Ray.hpp"
#include "Vector.hpp"

struct BVHBuildNode {
    Bounds3 bounds;
    BVHBuildNode* left{nullptr};
    BVHBuildNode* right{nullptr};
    Object* object{nullptr};

    int splitAxis = 0, firstPrimOffset = 0, nPrimitives = 0;
};

void dump(BVHBuildNode* node, int depth = 0);

// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;

// BVHAccel Declarations
class BVHAccel
{
public:
    // BVHAccel Public Types
    enum class SplitMethod { NAIVE, SAH };

    // BVHAccel Public Methods
    BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1,
             SplitMethod splitMethod = SplitMethod::NAIVE);
    Bounds3 WorldBound() const;
    ~BVHAccel();

    Intersection Intersect(const Ray& ray) const;
    Intersection getIntersection(BVHBuildNode* node, const Ray& ray) const;
    bool IntersectP(const Ray& ray) const;
    BVHBuildNode* root;
    // std::unique_ptr<BVHBuildNode> root;

    // BVHAccel Private Methods
    BVHBuildNode* recursiveBuild(std::vector<Object*>& objects);

    BVHBuildNode* recursiveBuildSAH(std::span<Object*> objects);

    // BVHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object*> primitives;

    int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
};

#endif  // RAYTRACING_BVH_H
