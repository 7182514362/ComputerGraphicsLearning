//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <numbers>

#include "Intersection.hpp"
#include "Vector.hpp"
#include "global.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum) {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(const Ray &ray, const std::vector<Object *> &objects,
                  float &tNear, uint32_t &index, Object **hitObject) const
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection itsc = intersect(ray);
    if (!itsc.happened) {
        return {};
    }
    if (itsc.m->hasEmission()) {
        if (depth == 0) {
            return itsc.m->getEmission();
        } else {
            return {};
        }
    }
    Vector3f p = itsc.coords;
    Vector3f wo = ray.direction;
    Vector3f N = itsc.normal;

    Intersection light_pos;
    float pdf_light;
    sampleLight(light_pos, pdf_light);

    Vector3f x = light_pos.coords;
    Vector3f ws = (p - x).normalized();
    Vector3f NN = light_pos.normal.normalized();
    float distance = (p - x).norm();

    Vector3f l_direct(0);
    Intersection l_to_obj = intersect(Ray(x, ws));
    if (l_to_obj.happened && std::abs(distance - l_to_obj.distance) < 0.001f) {
        Vector3f f_r = itsc.m->eval(wo, -ws, N);
        l_direct = light_pos.emit * f_r * dotProduct(-ws, N) *
                   dotProduct(ws, NN) / std::pow(distance, 2.0f) / pdf_light;
    }

    Vector3f l_indirect(0);
    float P_RR = get_random_float();
    if (P_RR < RussianRoulette) {
        Vector3f wi = itsc.m->sample(wo, N).normalized();
        Ray ray2(p, wi);
        Intersection obj_to_scene = intersect(ray2);
        if (obj_to_scene.happened && !obj_to_scene.m->hasEmission()) {
            Vector3f l = castRay(ray2, depth + 1);
            Vector3f f_r = itsc.m->eval(wo, wi, N);
            float pdf = itsc.m->pdf(wo, wi, N);
            l_indirect = l * f_r * dotProduct(wi, N) / pdf / RussianRoulette;
        }
    }

    return l_direct + l_indirect;
}