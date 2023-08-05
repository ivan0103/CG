#include "interpolate.h"
#include <glm/geometric.hpp>

float triangleArea(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) {
    return glm::length(glm::cross(v2 - v0, v0 - v1)) / 2;
}

glm::vec3 computeBarycentricCoord (const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& p)
{
    float A = triangleArea(v0, v1, v2);
    float a = triangleArea(v1, p, v2) / A; // aV0
    float b = triangleArea(v0, p, v2) / A; // bV1
    float c = triangleArea(v0, p, v1) / A; // cV2

    return glm::vec3(a, b, c);
}

glm::vec3 interpolateNormal (const glm::vec3& n0, const glm::vec3& n1, const glm::vec3& n2, const glm::vec3 barycentricCoord)
{
    return n0 * barycentricCoord.x + n1 * barycentricCoord.y + n2 * barycentricCoord.z;
}

glm::vec2 interpolateTexCoord(const glm::vec2& t0, const glm::vec2& t1, const glm::vec2& t2, const glm::vec3 barycentricCoord)
{
    return t0 * barycentricCoord.x + t1 * barycentricCoord.y + t2 * barycentricCoord.z;
}
    