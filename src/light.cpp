#include "light.h"
#include "config.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>

// samples a segment light source
// you should fill in the vectors position and color with the sampled position and color
void sampleSegmentLight(const SegmentLight& segmentLight, glm::vec3& position, glm::vec3& color)
{
    glm::vec3 p1 = segmentLight.endpoint0;
    glm::vec3 c1 = segmentLight.color0;
    glm::vec3 p2 = segmentLight.endpoint1;
    glm::vec3 c2 = segmentLight.color1;

    glm::vec3 l = p2 - p1;

    float ran = (rand() % 1001) / 1000.f;
    position = p1 + l * ran;
    color = ((ran * c2) + ((1 - ran) * c1));
}

float areaOfTriangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2)
{
    return glm::length(glm::cross(v2 - v0, v2 - v1)) / 2.0f;
}

// samples a parallelogram light source
// you should fill in the vectors position and color with the sampled position and color
void sampleParallelogramLight(const ParallelogramLight& parallelogramLight, glm::vec3& position, glm::vec3& color)
{

    float a = (rand() % 1001) / 1000.0f;
    float b = (rand() % 1001) / 1000.0f;

    position = parallelogramLight.v0 + a * parallelogramLight.edge01 + b * parallelogramLight.edge02;


    glm::vec3 c0 = parallelogramLight.color0;
    glm::vec3 c1 = parallelogramLight.color1;
    glm::vec3 c2 = parallelogramLight.color2;
    glm::vec3 c3 = parallelogramLight.color3;

    glm::vec3 p = position;

    color = 
        c0 * (1 - a) * (1 - b) +
        c1 * a * (1 - b) +
        c2 * (1-a) * b + 
        c3 * a * b;
 
}

// test the visibility at a given light sample
// returns 1.0 if sample is visible, 0.0 otherwise
float testVisibilityLightSample(const glm::vec3& samplePos, const glm::vec3& debugColor, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo)
{
    glm::vec3 shadowRayDir = glm::normalize(samplePos - (ray.origin + ray.direction * ray.t));
    glm::vec3 shadowRayStart = (ray.origin + ray.direction * ray.t) + hitInfo.normal * 0.0001f;
    Ray shadow(shadowRayStart, shadowRayDir, std::numeric_limits<float>::max());
    float lightT = glm::length((ray.origin + ray.direction * ray.t) - samplePos);
    if (bvh.intersect(shadow, hitInfo, features)) {
        if (shadow.t < lightT) {
            drawRay(shadow, glm::vec3(1.0f, 0.0f, 0.0f));
            return 0.0f;
        }
    }
    drawRay(shadow, debugColor);
    return 1.0f;
}

// given an intersection, computes the contribution from all light sources at the intersection point
// in this method you should cycle the light sources and for each one compute their contribution
// don't forget to check for visibility (shadows!)

// Lights are stored in a single array (scene.lights) where each item can be either a PointLight, SegmentLight or ParallelogramLight.
// You can check whether a light at index i is a PointLight using std::holds_alternative:
// std::holds_alternative<PointLight>(scene.lights[i])
//
// If it is indeed a point light, you can "convert" it to the correct type using std::get:
// PointLight pointLight = std::get<PointLight>(scene.lights[i]);
//
//
// The code to iterate over the lights thus looks like this:
// for (const auto& light : scene.lights) {
//     if (std::holds_alternative<PointLight>(light)) {
//         const PointLight pointLight = std::get<PointLight>(light);
//         // Perform your calculations for a point light.
//     } else if (std::holds_alternative<SegmentLight>(light)) {
//         const SegmentLight segmentLight = std::get<SegmentLight>(light);
//         // Perform your calculations for a segment light.
//     } else if (std::holds_alternative<ParallelogramLight>(light)) {
//         const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
//         // Perform your calculations for a parallelogram light.
//     }
// }
//
// Regarding the soft shadows for **other** light sources **extra** feature:
// To add a new light source, define your new light struct in scene.h and modify the Scene struct (also in scene.h)
// by adding your new custom light type to the lights std::variant. For example:
// std::vector<std::variant<PointLight, SegmentLight, ParallelogramLight, MyCustomLightType>> lights;
//
// You can add the light sources programmatically by creating a custom scene (modify the Custom case in the
// loadScene function in scene.cpp). Custom lights will not be visible in rasterization view.
glm::vec3 computeLightContribution(const Scene& scene, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo)
{

    glm::vec3 shading(0.0f, 0.0f, 0.0f);
    for (const auto& light : scene.lights){
        if (features.enableShading) {
            if (std::holds_alternative<PointLight>(light)) {
                const PointLight pointLight = std::get<PointLight>(light);
                if (features.enableHardShadow) {
                    if (testVisibilityLightSample(pointLight.position, pointLight.color, bvh, features, ray, hitInfo) == 0.0f)
                         continue;
                    shading += computeShading(pointLight.position, pointLight.color, features, ray, hitInfo);
                } else {
                    shading += computeShading(pointLight.position, pointLight.color, features, ray, hitInfo);
                } 
            }
            if (features.enableSoftShadow) {
                if (std::holds_alternative<SegmentLight>(light)) {
                    const SegmentLight segmentLight = std::get<SegmentLight>(light);
                    glm::vec3 p = glm::vec3 { 0 };
                    glm::vec3 c = glm::vec3 { 0 };
                    glm::vec3 color = shading;
                    for (int i = 0; i < 5; i++) {
                        sampleSegmentLight(segmentLight, p, c);
                        if (testVisibilityLightSample(p, c, bvh, features, ray, hitInfo) == 0.0f)
                             continue;
                        shading += computeShading(p, c, features, ray, hitInfo);
                        
                        drawRay(Ray { (ray.origin + ray.t * ray.direction), glm::normalize(p - (ray.origin + ray.t * ray.direction)) }, c);
                    }
                    shading /= 5;
                } else if (std::holds_alternative<ParallelogramLight>(light)) {
                    const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
                    glm::vec3 p = glm::vec3 { 0 };
                    glm::vec3 c = glm::vec3 { 0 };
                    for (int i = 0; i < 30; i++) {
                        sampleParallelogramLight(parallelogramLight, p, c);
                        if (testVisibilityLightSample(p, c, bvh, features, ray, hitInfo) == 0.0f)
                            continue;
                        shading += computeShading(p, c, features, ray, hitInfo);
                        drawRay(Ray { (ray.origin + ray.t * ray.direction), glm::normalize(p - (ray.origin + ray.t * ray.direction)) }, c);
                    }
                    shading /= 30;
                }
            }
        } else {
            return hitInfo.material.kd;
        }
    }
    return shading;
}
 
