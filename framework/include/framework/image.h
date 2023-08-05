#pragma once
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()
#include <filesystem>
#include <vector>

struct Image {
public:
    explicit Image(const std::filesystem::path& filePath);

public:
    int width, height;
    std::vector<glm::vec3> pixels;
};
