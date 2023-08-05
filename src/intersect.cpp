#include "intersect.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <limits>


bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p) {
    glm::vec3 n0 = glm::cross(v0 - p, v1 - p);
    glm::vec3 n1 = glm::cross(v1 - p, v2 - p);
    glm::vec3 n2 = glm::cross(v2 - p, v0 - p);

    float dot1 = glm::dot(n0, n1);
    float dot2 = glm::dot(n0, n2);

    return dot1 >= 0 && dot2 >= 0;
}

bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    float dot = glm::dot(plane.normal, ray.direction);
    if (dot == 0) {
        return false;
    }
    float top = glm::dot(plane.D * plane.normal - ray.origin, plane.normal)
    float t =  top / dot;
    if (t <= 0 || t >= ray.t) {
        return false;
    }
    ray.t = t;
    return true;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    Plane triangle;
    glm::vec3 cross = glm::cross(v0 - v1, v0 - v2);
    glm::vec3 normal = glm::abs(glm::normalize(cross));
    float distance = glm::abs(glm::dot(normal, v0));
    return Plane(distance, normal);
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo)
{
    float t = ray.t;
    Plane plane = trianglePlane(plane, ray);
    glm::vec3 p = ray.origin + ray.t * ray.direction;
    if (intersectRayWithPlane(plane, ray) && pointInTriangle(vertex0, vertex1, vertex2, plane.normal, p)) {
        return true;
    }
    ray.t = t;
    return false;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
    float a = pow(glm::length(ray.direction), 2);
    float b = glm::dot(ray.direction, ray.origin - sphere.center) * 2;
    float c = pow(glm::length(ray.origin - sphere.center), 2) - pow(sphere.radius, 2);
    float d = b * b - 4 * a * c;
    float t;
    if (d == 0) {
        t = -b / (a * 2);
    }
    else if (d > 0) {
        float p0 = (-b - sqrt(d)) / (a * 2);
        float p1 = (-b + sqrt(d)) / (a * 2);
        t = p0 > 0 ? std::min(p0, p1) : t = p1;
    } else {
        return false;
    }

    if (t > 0 && t < ray.t) {
        ray.t = t;
        hitInfo.normal = glm::normalize(ray.origin - sphere.center + t * ray.direction);
        hitInfo.material = &sphere.material;
        return true;
    }
    return false;
}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
    float xIn = (box.lower.x - ray.origin.x) / ray.direction.x;
    float xOut = (box.upper.x - ray.origin.x) / ray.direction.x;
    float yIn = (box.lower.y - ray.origin.y) / ray.direction.y;
    float yOut = (box.upper.y - ray.origin.y) / ray.direction.y;
    float zIn = (box.lower.z - ray.origin.z) / ray.direction.z;
    float zOut = (box.upper.z - ray.origin.z) / ray.direction.z;

    float xMax = std::max(xIn, xOut);
    float xMin = std::min(xIn, xOut);

    float yMax = std::max(yIn, yOut);
    float yMin = std::min(yIn, yOut);

    float zMax = std::max(zIn, zOut);
    float zMin = std::min(zIn, zOut);

    float lastIn = std::max({xMin, yMin, zMin});
    float firstOut = std::min({xMax, yMax, zMax});

    if (firstOut < lastIn || firstOut <= 0) {
        return false;
    }
    if (lastIn > 0 && lastIn < ray.t) {
        ray.t = lastIn;
    }
    else if (firstOut < ray.t) {
        ray.t = firstOut;
    }
    return true;
}