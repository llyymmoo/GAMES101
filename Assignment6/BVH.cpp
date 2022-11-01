#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    switch (splitMethod)
    {
        case SplitMethod::NAIVE :
            std::cout << "Partition method : NAIVE" << std::endl;
            root = recursiveBuild(primitives);
            break;
        
        case SplitMethod::SAH :
            std::cout << "Partition method : SAH" << std::endl;
            c_trav_ = 0.01;
            c_isect_ = 0.001;
            root = recursiveBuildSAH(primitives);
            break;
    }

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuildSAH(std::vector<Object*>objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    
    if (objects.size() == 1) {
        // Create leaf _SAH_Node_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() >= 2) {
        // calculate max dim
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        
        switch (dim) {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                        f2->getBounds().Centroid().x;
                });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                        f2->getBounds().Centroid().y;
                });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                        f2->getBounds().Centroid().z;
                });
                break;
        }

        // calculate surface area
        float total_areas = 0.0f;
        std::vector<float> areas;
        areas.reserve(objects.size());
        for (auto& obj : objects)
        {
            areas.push_back(obj->getBounds().SurfaceArea());
            total_areas += areas.back();
        }

        // find max partition point Using SAH cost
        // total cost = c_trav + S_A / S_N * N_A * c_isect + S_B / S_N * N_B * c_isect
        // c_trav  : cost of bbox check
        // S_A     : surface area of left node primitives
        // S_N     : surface area of total primitives
        // N_A     : num of primitives in the left node
        // c_isect : cost of bbox check & intersection check for one primitive
        int N = objects.size();
        int N_A = 0;
        float S_A = 0.0f;
        float S_N_inv = 1.0f / total_areas;
        float total_cost = std::numeric_limits<float>::max();
        int partition_point;
        for (int i = 0; i < N - 1; ++i)
        {
            S_A += areas[i];
            ++N_A;
            float tmp_cost = c_trav_ + S_A * S_N_inv * N_A * c_isect_ + (total_areas - S_A) * S_N_inv * (N - N_A) * c_isect_;
            if (tmp_cost < total_cost)
            {
                total_cost = tmp_cost;
                partition_point = i;
            }
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + partition_point + 1;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuildSAH(leftshapes);
        node->right = recursiveBuildSAH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
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

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    Intersection isect;

    recursiveSearch(node, isect, ray);

    return isect;
}

void BVHAccel::recursiveSearch(BVHBuildNode* node, Intersection& isect, const Ray& ray) const
{
    if (node->left == nullptr && node->right == nullptr)
    {
        if (node->bounds.IntersectP(ray))
        {
            Intersection tmp_isect;
            tmp_isect = node->object->getIntersection(ray);

            if (tmp_isect.happened && tmp_isect.distance < isect.distance)
                isect = tmp_isect;
            
            return;
        }

        return;
    }

    if (node->left != nullptr && node->left->bounds.IntersectP(ray))
        recursiveSearch(node->left, isect, ray);

    if (node->right != nullptr && node->right->bounds.IntersectP(ray))
        recursiveSearch(node->right, isect, ray);

    return;
}