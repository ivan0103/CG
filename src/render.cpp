#include "render.h"
#include "intersect.h"
#include "light.h"
#include "screen.h"
#include "shading.h"
#include "texture.h"
#include <iostream>
#include <cmath>
#include <framework/trackball.h>
#include <iostream>
#ifdef NDEBUG
#include <omp.h>
#endif
Image posx(std::filesystem::path("../../../data/enviromap/posx.jpg"));
Image negx(std::filesystem::path("../../../data/enviromap/negx.jpg"));
Image posy(std::filesystem::path("../../../data/enviromap/posy.jpg"));
Image negy(std::filesystem::path("../../../data/enviromap/negy.jpg"));
Image posz(std::filesystem::path("../../../data/enviromap/posz.jpg"));
Image negz(std::filesystem::path("../../../data/enviromap/negz.jpg"));

glm::vec2 getUV(glm::vec2 uv) {
    uv = uv + 1.0f;
    if (uv.x == 0 && uv.y == 0) { 
        return uv;
    }
    if (uv.x == 0) {
        uv.y = uv.y / 2.0f;
        return uv;
    }
    if (uv.y == 0) {
        uv.x = uv.x / 2.0f;
        return uv;
    }
    else {
        return uv / 2.0f;
    }
    
}



typedef std::vector<glm::vec3> Array;
typedef std::vector<Array> Matrix;
typedef std::vector<float> ArrayOfFloats;
typedef std::vector<ArrayOfFloats> MatrixOfFloats;


MatrixOfFloats getGaussian(int size, float sigma)
{
    MatrixOfFloats kernel(size, ArrayOfFloats(size, 0.0f));
    float sum = 0.0f;
    int i, j;
    int height = size;
    int width = size;

    for (i = -height/2; i <= height/2; i++) {
        for (j = -width/2; j <= width/2; j++) {
            kernel[i+height/2][j+width/2] = exp(-(i * i + j * j) / (2 * sigma * sigma)) / (2 * 3.14159265359f * sigma * sigma);
            sum += kernel[i + height/2][j + width/2];
        }
    }

    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            kernel[i][j] /= sum;
        }
    }

    return kernel;
}

glm::vec3 applyFilterAt(std::vector<glm::vec3>& PS, MatrixOfFloats& filter, int i, int j, glm::ivec2 res)
{
    int w = filter[0].size();
    int h = filter.size();
    glm::vec3 sum = glm::vec3 { 0.0f };
    Screen screen = Screen(res, false);
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
            int index = screen.indexAt(i + x - w / 2, j + y - h / 2);
            glm::vec3 c = PS[index];
            sum += filter[x][y] * c;
        }
    }
    return sum;
}

glm::vec3 getFinalColor(const Scene& scene, const BvhInterface& bvh, Ray ray, const Features& features, int rayDepth)
{
    HitInfo hitInfo;
    if (bvh.intersect(ray, hitInfo, features)) {
        
        glm::vec3 Lo = computeLightContribution(scene, bvh, features, ray, hitInfo);
        if (features.enableRecursive && rayDepth < 5 && hitInfo.material.ks != glm::vec3 {0.0f}) {
            Ray reflection = computeReflectionRay(ray, hitInfo);
            if (features.extra.enableGlossyReflection && hitInfo.material.shininess != 0.0f) {
                glm::vec3 colour(0.0f);
                for (int i = 0; i < 50; i++) {
                    Ray glossyRay = glossyReflectionRay(reflection, hitInfo);
                    colour += getFinalColor(scene, bvh, glossyRay, features, rayDepth);
                }
                colour = colour / 50.0f;
                Lo += colour;
            } else {
                Lo += hitInfo.material.ks * getFinalColor(scene, bvh, reflection, features, rayDepth + 1);
            }   
        }
        drawRay(ray, Lo);
        // Draw a white debug ray if the ray hits.
        

        // Set the color of the pixel to white if the ray hits.
        return Lo;
    } else {
        if (features.extra.enableEnvironmentMapping) {
            glm::vec3 dir = ray.direction;
            glm::vec3 scaled = dir / glm::max(glm::max(glm::abs(dir.x), glm::abs(dir.y)), glm::abs(dir.z));
            glm::vec3 skyboxColor(0.0f);
            if (scaled.x == 1.0f) {
                skyboxColor += acquireTexel(posx, getUV(glm::vec2(scaled.z, scaled.y)), features);
            }
            if (scaled.x == -1.0f) {
                skyboxColor += acquireTexel(negx, getUV(glm::vec2(-scaled.z, scaled.y)), features);
            }
            if (scaled.y == 1.0f) {
                skyboxColor += acquireTexel(posy, getUV(glm::vec2(-scaled.z, scaled.x)), features);
            }
            if (scaled.y == -1.0f) {
                skyboxColor += acquireTexel(negy, getUV(glm::vec2(scaled.x, -scaled.z)), features);
            }
            if (scaled.z == 1.0f) {
                skyboxColor += acquireTexel(negz, getUV(glm::vec2(-scaled.x, scaled.y)), features);
            }
            if (scaled.z == -1.0f) {
                skyboxColor += acquireTexel(posz, getUV(glm::vec2(scaled.x, scaled.y)), features);
            }
            drawRay(ray, skyboxColor);
            return skyboxColor;
        }
        // Draw a red debug ray if the ray missed.
        drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
        // Set the color of the pixel to black if the ray misses.
        return glm::vec3(0.0f);
    }
}

void renderRayTracing(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    glm::ivec2 windowResolution = screen.resolution();
    // Enable multi threading in Release mode
#ifdef NDEBUG
#pragma omp parallel for schedule(guided)
#endif
    glm::vec3 c;
    Matrix threshholded(windowResolution.y, Array(windowResolution.x));
    glm::vec3 luminanceThresh = glm::vec3 { 0.2125, 0.7154, 0.0721 };


    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
            const glm::vec2 normalizedPixelPos {
                float(x) / float(windowResolution.x) * 2.0f - 1.0f,
                float(y) / float(windowResolution.y) * 2.0f - 1.0f
            };
            if (features.extra.enableMultipleRaysPerPixel) {
                int n = 2;
                for (int p = 0; p < n; p++) {
                    for (int q = 0; q < n; q++) {
                        float r = (rand() % 1001) / 1000;
                        const glm::vec2 normalizedJitteredPixelPos {
                            (float(x) + (p + r) / n) / float(windowResolution.x) * 2.0f - 1.0f,
                            (float(y) + (q + r) / n) / float(windowResolution.y) * 2.0f - 1.0f
                        };
                        const Ray cameraRay = camera.generateRay(normalizedJitteredPixelPos);
                        c += getFinalColor(scene, bvh, cameraRay, features);
                    }
                }
                c /= n * n;
                screen.setPixel(x, y, c);
            }

            else {
                const Ray cameraRay = camera.generateRay(normalizedPixelPos);
                c = getFinalColor(scene, bvh, cameraRay, features);

                if (features.extra.enableBloomEffect) {
                    float brightness = glm::dot(c, luminanceThresh);
                    if (brightness > 0.7f) {
                        threshholded[x][y] = c;
                        screen.setPixel(x, y, c);
                    } else {
                        screen.setPixel(x, y, glm::vec3 { 0.0f });
                    }
                } else {
                    screen.setPixel(x, y, c);
                }
            }
          
        }
    }

    if (features.extra.enableBloomEffect) {
        std::vector<glm::vec3> PS = screen.pixels();
        std::vector<glm::vec3> PS_POST;
        for (int y = 0; y < windowResolution.y; y++) {
            for (int x = 0; x != windowResolution.x; x++) {
                int i = screen.indexAt(x, y);
                glm::vec3 c = PS[i];
                PS_POST.push_back(c);
            }
        }

        MatrixOfFloats gaussian = getGaussian(9, 0.99);
        for (int y = 9; y < windowResolution.y - 9; y++) {
            for (int x = 9; x < windowResolution.x - 9; x++) {
                int i = screen.indexAt(x, y);
                glm::vec3 c = PS[i];
                PS_POST[i] = applyFilterAt(PS, gaussian, x, y, screen.resolution());
            }
        }

        for (int y = 0; y < windowResolution.y; y++) {
            for (int x = 0; x != windowResolution.x; x++) {
                int i = screen.indexAt(x, y);
                screen.setPixel(x, y, 1.5f * PS_POST[i]);
            }
        }


    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    std::cout << "Render time with current specification = " << time << "[ms] " << std::endl;

}

