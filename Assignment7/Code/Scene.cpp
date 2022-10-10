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
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Intersection hit_obj = intersect(ray);
    // std::cout << "[castRay] " << "Depth: " << depth << " - Ray: " << ray << std::endl;
    // std::cout << "[castRay] " << "obj: " << (hit_obj.happened ? "shot" : "miss") << std::endl;
    if (!hit_obj.happened) return this->backgroundColor;
    return shade(hit_obj, -ray.direction, depth);
}

// shade(p, wo)
//   sampleLight(inter, pdf_light)
//   Get x, ws, NN, emit from inter
//   Shoot a ray from p to x
//   If the ray is not blocked in the middle
//     L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x-p|^2 / pdf_light
//   L_indir = 0.0
//   Test Russian Roulette with probability RussianRoulette
//   wi = sample(wo, N)
//   Trace a ray r(p, wi)
//   If ray r hit a non-emitting object at q
//     L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N) / pdf(wo, wi, N) / RussianRoulette
//   Return L_dir + L_indir
Vector3f Scene::shade(Intersection& hit_obj, Vector3f wo, int depth) const {

    // std::cout << "[shade] " << "hit_obj name: " << hit_obj.obj->getName() << std::endl;

    // if (depth + 1 > maxDepth) { return {}; }  // This line is used for preventing too much recursion when debug.

    // check if the object emits light.
    if (hit_obj.m -> hasEmission()) {
        // std::cout << "[shade] " << "emit: " << hit_obj.m->getEmission() << std::endl;
        return hit_obj.m -> getEmission();
    }

    // std::cout << "[shade] Start." << std::endl;

    // basic information about shading point
    Vector3f p  = hit_obj.coords;  // shading point
    Material *m = hit_obj.m;       // material at shading point
    Vector3f N  = hit_obj.normal;  // normal of shading point

    // std::cout << "[shade] " << "obj: coords " << p << std::endl;
    // std::cout << "[shade] " << "obj: normal " << N << std::endl;

    // Direct Illumination
    Intersection inter_light;
    float pdf_light;
    sampleLight(inter_light, pdf_light);

    Vector3f x = inter_light.coords;
    Vector3f x_minus_p = x - p;
    Vector3f ws = normalize(x_minus_p);
    Vector3f NN = normalize(inter_light.normal);
    Vector3f emit = inter_light.emit;

    // std::cout << "[shade] " << "sample: coords " << inter_light.coords << std::endl;
    // std::cout << "[shade] " << "sample: normal " << inter_light.normal << std::endl;
    // std::cout << "[shade] " << "sample: pdf " << pdf_light << std::endl;
    // std::cout << "[shade] " << "sample: x_minus_p " << x_minus_p << std::endl;
    // std::cout << "[shade] " << "sample: ws " << ws << std::endl;
    // std::cout << "[shade] " << "sample: NN " << NN << std::endl;

    Vector3f L_dir(0);
    Intersection inter_dir = intersect(Ray(p, ws));  // shot a ray
    float test_l2 = (inter_dir.coords - inter_light.coords).norm();
    // std::cout << "[shade] " << "Direct: Test " << test_l2 << " = " << inter_dir.coords << " - " << inter_light.coords << std::endl;
    if (test_l2 < EPSILON) {
        // if not block
        // std::cout << "[shade] " << "Direct: not block." << std::endl;

        Vector3f f_r = m->eval(ws, wo, N);   // BRDF
        float r2     = dotProduct(x_minus_p, x_minus_p);
        float cos_i  = dotProduct(-ws, NN);  // 随机光线和光源的衰减
        float cos_r  = dotProduct(ws, N);    // 随机光线和物体的衰减

        // std::cout << "[shade] " << "Li: "    << emit  << std::endl;
        // std::cout << "[shade] " << "r2: "    << r2    << std::endl;
        // std::cout << "[shade] " << "f_r: "   << f_r   << std::endl;
        // std::cout << "[shade] " << "cos_i: " << cos_i << std::endl;
        // std::cout << "[shade] " << "cos_r: " << cos_r << std::endl;
        // std::cout << "[shade] " << "pdf: "   << pdf_light << std::endl;

        L_dir = emit / r2 * f_r * cos_i * cos_r / pdf_light;
    }
    // std::cout << "[shade] " << "Direct: " << L_dir << std::endl;

    // Indirect Illumination
    Vector3f L_indir(0);
    if (get_random_float() < RussianRoulette) {
        // std::cout << "[shade] " << "Indirect: hit." << std::endl;

        Vector3f wi    = m->sample(wo, N);    // Randomly choose ONE direction wi~pdf(w)
        Vector3f f_r   = m->eval(wi, wo, N);  // BRDF
        Vector3f color = castRay(Ray(p, wi), depth + 1);
        float cos = dotProduct(wi, N);
        float pdf = m->pdf(wi, wo, N);

        // std::cout << "[shade] " << "ray: " << color << std::endl;
        // std::cout << "[shade] " << "f_r: " << f_r << std::endl;
        // std::cout << "[shade] " << "cos: " << cos << std::endl;
        // std::cout << "[shade] " << "pdf: " << pdf << std::endl;

        L_indir = color * f_r * cos / pdf / RussianRoulette;
    }
    // std::cout << "[shade] " << "Indirect: " << L_indir << std::endl;

    return L_dir + L_indir;
}