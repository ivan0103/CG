#include "texture.h"
#include <framework/image.h>
#include <iostream>

glm::vec3 acquireTexel(const Image& image, const glm::vec2& texCoord, const Features& features)
{
    // TODO: implement this function.
    // Given texcoords, return the corresponding pixel of the image
    // The pixel are stored in a 1D array of row major order
    // you can convert from position (i,j) to an index using the method seen in the lecture
    // Note, the center of the first pixel is at image coordinates (0.5, 0.5)
    int i = std::floor(texCoord.x * image.height);
    int j = std::floor((1.0f - texCoord.y) * image.width);
    float texelIndex = j * image.height + i;
    return image.pixels[texelIndex];
}
