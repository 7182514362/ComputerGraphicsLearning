#include "BVH.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <nlohmann/json.hpp>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include "Bounds3.hpp"
#include "Object.hpp"
#include "Vector.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)),
      splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty()) return;

    // root = recursiveBuild(primitives);
    root = recursiveBuildSAH(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*>& objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    // Bounds3 bounds;
    // for (int i = 0; i < objects.size(); ++i) {
    //     bounds = Union(bounds, objects[i]->getBounds());
    // }

    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    } else if (objects.size() == 2) {
        std::vector<Object*> leftShapes{objects[0]};
        std::vector<Object*> rightShapes{objects[1]};
        node->left = recursiveBuild(leftShapes);
        node->right = recursiveBuild(rightShapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    } else {
        Bounds3 bounds;
        for (int i = 0; i < objects.size(); ++i) {
            bounds = Union(bounds, objects[i]->getBounds().Centroid());
        }
        // 按最长轴划分
        int dim = bounds.maxExtent();
        switch (dim) {
            case 0:  // x
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                           f2->getBounds().Centroid().x;
                });
                break;
            case 1:  // y
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                           f2->getBounds().Centroid().y;
                });
                break;
            case 2:  // z
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                           f2->getBounds().Centroid().z;
                });
                break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

struct Bucket {
    Bounds3 bounds;
    std::vector<Object*> objects;
};

static void sahPartition(std::span<Object*> objects, std::span<Object*>& lhs,
                         std::span<Object*>& rhs)
{
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i) {
        bounds.Union(objects[i]->getBounds());
        // bounds = Union(bounds, objects[i]->getBounds().Centroid());
    }

    // 按最长轴划分
    int dim = bounds.maxExtent();
    assert(dim >= 0 && dim <= 2);

    std::sort(objects.begin(), objects.end(), [dim](auto f1, auto f2) {
        return f1->getBounds().Centroid()[dim] <
               f2->getBounds().Centroid()[dim];
    });

    constexpr int bucketsNum = 16;
    std::array<Bucket, bucketsNum> buckets;

    double bucket_width_reciprocal = bucketsNum / bounds.Diagonal()[dim];

    for (Object* obj : objects) {
        Bounds3 objBounds = obj->getBounds();
        std::size_t bucketIdx = (objBounds.Centroid()[dim] - bounds.pMin[dim]) *
                                bucket_width_reciprocal;
        assert(bucketIdx < bucketsNum);

        buckets[bucketIdx].bounds.Union(objBounds);
        buckets[bucketIdx].objects.push_back(obj);
    }

    double minCost = std::numeric_limits<double>::max();
    // double totalSurfaceArea_reciprocal = 1.0 / bounds.SurfaceArea();

    int leftCount = 0;
    Bounds3 leftBounds;
    for (int i = 0; i < bucketsNum - 1; ++i) {
        if (buckets[i].objects.empty()) {
            continue;
        }

        leftBounds.Union(buckets[i].bounds);
        int count = leftCount + buckets[i].objects.size();

        Bounds3 rightBounds;
        for (int j = i + 1; j < bucketsNum; ++j) {
            rightBounds.Union(buckets[j].bounds);
        }

        double totalCost = leftBounds.SurfaceArea() + rightBounds.SurfaceArea();
        if (minCost > totalCost) {
            minCost = totalCost;
            leftCount = count;
        }
    }
    // std::cout << leftCount << '\n';
    lhs = objects.subspan(0, leftCount);
    rhs = objects.subspan(leftCount);
}

inline BVHBuildNode* createNode(Object* object)
{
    BVHBuildNode* node = new BVHBuildNode();
    node->object = object;
    node->left = nullptr;
    node->right = nullptr;
    if (object != nullptr) {
        node->bounds = object->getBounds();
    }
    return node;
}

BVHBuildNode* BVHAccel::recursiveBuildSAH(std::span<Object*> objects)
{
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        return createNode(objects[0]);
    } else if (objects.size() == 2) {
        BVHBuildNode* node = createNode(nullptr);
        node->left = createNode(objects[0]);
        node->right = createNode(objects[1]);
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    } else if (objects.size() != 0) {
        std::span<Object*> leftShapes;
        std::span<Object*> rightShapes;
        sahPartition(objects, leftShapes, rightShapes);

        assert(objects.size() == (leftShapes.size() + rightShapes.size()));

        if (leftShapes.empty() || rightShapes.empty()) {
            leftShapes = objects.subspan(0, 1);
            rightShapes = objects.subspan(1);
        }

        BVHBuildNode* node = createNode(nullptr);
        node->left = recursiveBuildSAH(leftShapes);
        node->right = recursiveBuildSAH(rightShapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    return nullptr;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root) {
        return isect;
    }
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    if (node == nullptr) {
        return Intersection();
    }
    bool isIsc =
        node->bounds.IntersectP(ray, ray.direction_inv,
                                {ray.direction.x < 0.0f, ray.direction.y < 0.0f,
                                 ray.direction.z < 0.0f});
    ray.addHitCheckCounter();
    if (!isIsc) {
        return Intersection();
    }
    if (node->left == nullptr && node->right == nullptr) {
        auto intersect = node->object->getIntersection(ray);
        ray.addHitCheckCounter();
        return intersect;
    }
    Intersection hit1 = getIntersection(node->left, ray);
    Intersection hit2 = getIntersection(node->right, ray);

    return (hit1.distance < hit2.distance) ? hit1 : hit2;
}

inline std::string stringify(const Vector3f& v)
{
    return "(" + std::to_string(v.x) + "," + std::to_string(v.y) + "," +
           std::to_string(v.z) + ")";
}

inline std::string stringify(const Bounds3& bounds)
{
    return "[" + stringify(bounds.pMin) + ", " + stringify(bounds.pMax) + "]";
}

inline std::string indent(int depth) { return std::string(depth << 1, ' '); }

void dump(BVHBuildNode* node, int depth)
{
    if (node == nullptr) {
        return;
    }
    std::cout << indent(depth) << "bounds: " << stringify(node->bounds) << '\n';
    if (node->object != nullptr) {
        std::cout << indent(depth)
                  << "object bounds: " << stringify(node->object->getBounds())
                  << '\n';
    }
    if (node->left != nullptr) {
        std::cout << indent(depth) << "left:\n";
        dump(node->left, depth + 1);
    }
    if (node->right != nullptr) {
        std::cout << indent(depth) << "right:\n";
        dump(node->right, depth + 1);
    }
}