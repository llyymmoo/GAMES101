//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
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
        if (objects[k]->hasEmit()){
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

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
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

// Implementation of pathTracing 
Vector3f Scene::shade(Intersection isect, const Vector3f wo) const
{
    Vector3f L_dir(0.0f, 0.0f, 0.0f);
    Vector3f L_indir(0.0f, 0.0f, 0.0f);

    Intersection pos_light;
    float pdf_light;
    sampleLight(pos_light, pdf_light);

    Vector3f dir_to_sample_light = normalize(pos_light.coords - isect.coords);
    Ray ray(isect.coords, dir_to_sample_light);

    Intersection isect_to_light = bvh->getIntersection(bvh->root, ray);
    if (isect_to_light.happened && isect_to_light.obj->hasEmit())
        L_dir = pos_light.emit
                * isect.m->eval(wo, dir_to_sample_light, isect.normal)
                * (isect.normal.dot(dir_to_sample_light))
                * (pos_light.normal.dot(-dir_to_sample_light))
                / ((isect.coords - pos_light.coords).norm() * (isect.coords - pos_light.coords).norm())
                / pdf_light;
    

    int num = (std::rand() % 10) + 1;
    if (num <= 8)
    {
        Vector3f wi = normalize(isect.m->sample(wo, isect.normal));
        Ray indir_ray(isect.coords+0.0001*wi, wi);
        Intersection indir_isect = bvh->getIntersection(bvh->root, indir_ray);
        if (indir_isect.happened && !indir_isect.obj->hasEmit())
            L_indir = shade(indir_isect, -wi)
                      * isect.m->eval(wo, wi, isect.normal)
                      * (wi.dot(isect.normal))
                      / isect.m->pdf(wi, wo, isect.normal)
                      / RussianRoulette;
    }

    return L_dir + L_indir;
}

// Implementation of rayTracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Vector3f color = backgroundColor;

    Intersection isect = bvh->getIntersection(bvh->root, ray);

    if (isect.happened)
    {
        if (!isect.obj->hasEmit())
            color = shade(isect, -ray.direction);
        else
            color = isect.m->getEmission();
    } 

    return color;
}