#include "texture.h"
#include <cmath>
#include <glm/geometric.hpp>
#include <shading.h>
#include <render.h>
#include <random>
#include <draw.cpp>
#include <iostream>
std::mt19937 gen(rand());
std::uniform_real_distribution<float> unif(0.0f, 1.0f);
const glm::vec3 computeShading(const glm::vec3& lightPosition, const glm::vec3& lightColor, const Features& features, Ray ray, HitInfo hitInfo)
{   
    glm::vec3 color(0.0f);
    glm::vec3 kd(0.0f);
    if (features.enableTextureMapping) {
        kd = acquireTexel(*hitInfo.material.kdTexture,hitInfo.texCoord,features);
    } else {
        kd = hitInfo.material.kd;
    }
    glm::vec3 L = glm::normalize(lightPosition - (ray.origin+ray.direction*ray.t));
    glm::vec3 V = glm::normalize(ray.origin - (ray.origin + ray.direction * ray.t));
    float angle = glm::degrees(glm::acos(glm::dot(L, hitInfo.normal) / ((glm::length(hitInfo.normal) * glm::length(L)))));
    if (angle < 90.0f && hitInfo.material.ks != glm::vec3(0.0f)) {
        glm::vec3 R = 2 * glm::dot(L, hitInfo.normal) * hitInfo.normal - L;
        if (glm::dot(R, V) > 0.0f) {
            color += lightColor * hitInfo.material.ks * glm::pow((glm::dot(R, V)), hitInfo.material.shininess);
        }
    }
    color += lightColor * kd * glm::max(0.0f, (glm::dot(L, hitInfo.normal)));
    return color;
}


const Ray computeReflectionRay (Ray ray, HitInfo hitInfo)
{
    // Do NOT use glm::reflect!! write your own code.
    Ray reflectionRay {};
    // TODO: implement the reflection ray computation.
    glm::vec3 rr = ray.direction - 2 * glm::dot(hitInfo.normal, glm::normalize(ray.direction)) * hitInfo.normal;
    reflectionRay.direction = rr;
    reflectionRay.origin = ray.origin + ray.direction * ray.t - 0.001f*ray.direction;
    reflectionRay.t = std::numeric_limits<float>::max();
    return reflectionRay;
}
const glm::vec3 findNonColinear(glm::vec3 w)
{
    float value = abs(w.x);
    if (abs(w.y) < value) {
        value = abs(w.y);
    }
    if (abs(w.z) < value) {
        value = abs(w.z);
    }
    if (abs(w.x) == value) {
        w.x = 1.0f;
    } else if (abs(w.y) == value) {
        w.y = 1.0f;
    } else if (abs(w.z) == value) {
        w.z = 1.0f;
    }
    return w;
}

const Ray glossyReflectionRay(Ray reflectionRay, HitInfo hitInfo)
{
    Ray glossyRay;
    glm::vec3 reflectionVector = reflectionRay.origin + reflectionRay.direction * 100.0f;
    glm::vec3 w = glm::normalize(reflectionRay.origin + reflectionRay.direction * 100.0f);
    glm::vec3 t = findNonColinear(w);
    glm::vec3 u = glm::cross(t, w) / glm::length(glm::cross(t, w));
    glm::vec3 v = glm::cross(w, u);
    float randU = -(hitInfo.material.shininess * 3 / 2) + (unif(gen) * hitInfo.material.shininess * 3);
    float randV = -(hitInfo.material.shininess * 3 / 2) + (unif(gen) * hitInfo.material.shininess * 3);
    glm::vec3 glossyReflectionVector = reflectionVector + randU * u + randV * v;
    glossyRay.direction = glm::normalize(glossyReflectionVector);
    glossyRay.origin = reflectionRay.origin;
    glossyRay.t = std::numeric_limits<float>::max();
    return glossyRay;
}

