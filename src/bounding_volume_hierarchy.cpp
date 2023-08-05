#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "intersect.h"
#include "scene.h"
#include "texture.h"
#include "interpolate.h"
#include <glm/glm.hpp>
#include <iostream>

std::vector<TriangleBVH> getAllSceneTriangles(Scene* m_pScene);
TriangleBVH createTriangle(Mesh& mesh, int meshIndex, int triangleMeshIndex);
Node initializeNode(std::vector<TriangleBVH>& triangles, Scene* m_pScene);
long double computeChildProb(long double p, Node& child);

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene, const Features& f)
    : m_pScene(pScene)
{   
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::vector<TriangleBVH> triangles = getAllSceneTriangles(m_pScene);
    Node root = initializeNode(triangles, m_pScene);
    root.level = 0;
    listOfNodes.push_back(root);
    if (f.extra.enableBvhSahBinning) {
        recursionSAHBuilder(triangles, 0, 0);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        int time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        std::cout << triangles .size() << " triangle[s]. BVH SAH Construction = " <<  time << "[ms]" << std::endl;

    }
    else {
        recursionBuilder(triangles, 0, 0);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        int time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        std::cout << triangles.size() << " triangle[s]. BVH Standard Construction = " << time << "[ms]" << std::endl;

    }
}

void BoundingVolumeHierarchy::recursionBuilder(std::vector<TriangleBVH> triangles, int level, size_t curNodeIndex)
{
    Node& node = listOfNodes[curNodeIndex];
    bool canBeSplit = triangles.size() > 1;
    if (!canBeSplit || level > 30) {
        this->m_numLevels = glm::max(this->m_numLevels, level + 1);
        listOfNodes[curNodeIndex].isInterior = false;
        this->m_numLeaves++;
        return;
    }
    listOfNodes[curNodeIndex].isInterior = true;
    std::sort(triangles.begin(), triangles.end(), [&](TriangleBVH t1, TriangleBVH t2) { return t1.centroid[level % 3] < t2.centroid[level % 3]; });
    size_t indexDiv = (triangles.size() / 2) - 1;
    std::vector<TriangleBVH> splitOne(&triangles[0], &triangles[indexDiv + 1]);
    std::vector<TriangleBVH> splitTwo(&triangles[indexDiv + 1], &triangles[triangles.size() - 1]);
    splitTwo.push_back(triangles[triangles.size() - 1]);

    std::vector<int> updatedNodes;
    int index1 = getIndexOfChildNode(splitOne, level);
    updatedNodes.push_back(index1);
    int index2 = getIndexOfChildNode(splitTwo, level);
    updatedNodes.push_back(index2);

    listOfNodes[curNodeIndex].subs = updatedNodes;

    recursionBuilder(splitOne, level + 1, index1);
    recursionBuilder(splitTwo, level + 1, index2);
}

 void BoundingVolumeHierarchy::recursionSAHBuilder(std::vector<TriangleBVH> triangles, int level, size_t curNodeIndex)
{
    Node& node = listOfNodes[curNodeIndex];
    bool canBeSplit = triangles.size() > 1;
    if (!canBeSplit || level > 30) {
        this->m_numLevels = glm::max(this->m_numLevels, level + 1);
        listOfNodes[curNodeIndex].isInterior = false;
        this->m_numLeaves++;
        return;
    }
    listOfNodes[curNodeIndex].isInterior = true;

    size_t indexDiv = (triangles.size() / 2) - 1;
    float cost = std::numeric_limits<float>::max();
    int axis = 0;

    for (int x = 0; x < 2; x++) {
        std::sort(triangles.begin(), triangles.end(), [&](TriangleBVH t1, TriangleBVH t2) { return t1.centroid[x] < t2.centroid[x]; });
        if (!triangles.size() == 1) {
            float val = computeCost(node, indexDiv, triangles);
            if (val < cost) {
                cost = val;
                axis = x;
            }
        }
        else {
            for (size_t i = 0; i < triangles.size() - 1; i++) {
                float val = computeCost(node, i, triangles);
                if (val < cost) {
                    cost = val;
                    indexDiv = i;
                    axis = x;
                }
            }
        }
    }


    std::sort(triangles.begin(), triangles.end(), [&](TriangleBVH t1, TriangleBVH t2) { return t1.centroid[axis] < t2.centroid[axis]; });
    std::vector<TriangleBVH> splitOne(&triangles[0], &triangles[indexDiv + 1]);
    std::vector<TriangleBVH> splitTwo(&triangles[indexDiv + 1], &triangles[triangles.size() - 1]);
    splitTwo.push_back(triangles[triangles.size() - 1]);

    std::vector<int> updatedNodes;
    int index1 = getIndexOfChildNode(splitOne, level);
    updatedNodes.push_back(index1);
    int index2 = getIndexOfChildNode(splitTwo, level);
    updatedNodes.push_back(index2);

    listOfNodes[curNodeIndex].subs = updatedNodes;

    recursionSAHBuilder(splitOne, level + 1, index1);
    recursionSAHBuilder(splitTwo, level + 1, index2);
}

 float BoundingVolumeHierarchy::computeCost(Node& node, size_t splitVal, std::vector<TriangleBVH> triangles)
 {
    std::vector<TriangleBVH> splitOne(&triangles[0], &triangles[splitVal+1]);
    std::vector<TriangleBVH> splitTwo(&triangles[splitVal+1], &triangles[triangles.size() - 1]);
    splitTwo.push_back(triangles[triangles.size() - 1]);
        Node child1 = initializeNode(splitOne, m_pScene);
        Node child2 = initializeNode(splitTwo, m_pScene);

        float x = (node.uBound.x - node.lBound.x);
        float y = (node.uBound.y - node.lBound.y);
        float z = (node.uBound.z - node.lBound.z);
        long double areaParent = x * y * z;

        long double probC1 = computeChildProb(areaParent, child1);
        long double probC2 = computeChildProb(areaParent, child2);

        return probC1 * splitOne.size() + probC2 * splitTwo.size();

}

 long double computeChildProb(long double areaParent, Node& child) {
    float x = (child.uBound.x - child.lBound.x);
    float y = (child.uBound.y - child.lBound.y);
    float z = (child.uBound.z - child.lBound.z);
    long double areaChild =  x * y * z;
    return  areaChild /  areaParent;
}

int BoundingVolumeHierarchy::getIndexOfChildNode(std::vector<TriangleBVH> t1Triangles, int level)
{
    Node n = initializeNode(t1Triangles, m_pScene);
    n.level = level + 1;
    listOfNodes.push_back(n);
    int newNodeIndex = listOfNodes.size() - 1;
    return newNodeIndex;
}

std::vector<TriangleBVH> getAllSceneTriangles(Scene* m_pScene)
{
    std::vector<TriangleBVH> result;
    for (int meshIndex = 0; meshIndex < m_pScene->meshes.size(); meshIndex++) {
        Mesh m = m_pScene->meshes[meshIndex];
        for (int triangleMeshIndex = 0; triangleMeshIndex < m.triangles.size(); triangleMeshIndex++) {
            result.push_back(createTriangle(m, meshIndex, triangleMeshIndex));
        }
    }
    return result;
}

TriangleBVH createTriangle(Mesh& mesh, int meshIndex, int triangleMeshIndex)
{
    glm::uvec3 tIndeces = mesh.triangles[triangleMeshIndex];
    float xIndex = tIndeces.x;
    float yIndex = tIndeces.y;
    float zIndex = tIndeces.z;

    glm::vec3 posX = mesh.vertices[xIndex].position;
    glm::vec3 posY = mesh.vertices[yIndex].position;
    glm::vec3 posZ = mesh.vertices[zIndex].position;

    glm::vec3 centroid = (posX + posY + posZ) / 3.0f;
    TriangleBVH triangle = TriangleBVH(meshIndex, mesh.triangles[triangleMeshIndex], centroid);
    return triangle;
}

Node initializeNode(std::vector<TriangleBVH>& triangles, Scene* m_pScene)
{
    Node n;
    n.isInterior = false;
    std::vector<int> children;
    for (TriangleBVH& t : triangles) {
        children.push_back(t.meshIndex);
        children.push_back(t.triangleMesh.x);
        children.push_back(t.triangleMesh.y);
        children.push_back(t.triangleMesh.z);
    };
    n.subs = children;

    for (int i = 0; i < n.subs.size(); i += 4) {
        Mesh& v = m_pScene->meshes[n.subs[i]];
        for (int j = 1; j <= 3; j++) {
            n.lBound = glm::min(n.lBound, v.vertices[n.subs[i + j]].position);
            n.uBound = glm::max(n.uBound, v.vertices[n.subs[i + j]].position);
        }
    }
    return n;
}

int BoundingVolumeHierarchy::numLevels() const
{
    return m_numLevels;
}

// Return the number of leaf nodes in the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display for Visual Debug 2.
int BoundingVolumeHierarchy::numLeaves() const
{
    return m_numLeaves;
}

// Use this function to visualize your BVH. This is useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDrawLevel(int level)
{
    for (int i = 0; i < listOfNodes.size(); i++) {
        if (listOfNodes[i].level == level) {
            AxisAlignedBox aabb { listOfNodes[i].lBound, listOfNodes[i].uBound };
            drawAABB(aabb, DrawMode::Wireframe, glm::vec3(0.05f, 1.0f, 0.05f), 1.0f);
        }
    }
}

// Use this function to visualize your leaf nodes. This is useful for debugging. The function
// receives the leaf node to be draw (think of the ith leaf node). Draw the AABB of the leaf node and all contained triangles.
// You can draw the triangles with different colors. NoteL leafIdx is not the index in the node vector, it is the
// i-th leaf node in the vector.
void BoundingVolumeHierarchy::debugDrawLeaf(int leafIdx)
{
    // Draw the AABB as a transparent green box.
    //AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);
    int acc = 0;
    for (Node& node : listOfNodes) {
        if (!node.isInterior) {
            acc++;
            if (acc == leafIdx) {
                AxisAlignedBox aabb { node.lBound, node.uBound };
                drawAABB(aabb, DrawMode::Wireframe, glm::vec3(1.0f, 1.0f, 1.0f), 1.0f);
                for (int i = 0; i < node.subs.size(); i += 4) {
                    Mesh& m = m_pScene->meshes[node.subs[i]];
                    std::vector<Vertex> coords;
                    for (int j = 1; j <= 3; j++) {
                        coords.push_back(m.vertices[node.subs[i + j]]);
                    }
                    drawTriangle(coords[0], coords[1], coords[2]);
                }
                break;
            }
        }
    }
}


// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h.
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo, const Features& features) const
{
    // If BVH is not enabled, use the naive implementation.
    if (!features.enableAccelStructure) {
        bool hit = false;
        // Intersect with all triangles of all meshes.
        for (const auto& mesh : m_pScene->meshes) {
            for (const auto& tri : mesh.triangles) {
                const auto v0 = mesh.vertices[tri[0]];
                const auto v1 = mesh.vertices[tri[1]];
                const auto v2 = mesh.vertices[tri[2]];
                if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo)) {
                    hitInfo.material = mesh.material;
                    hitInfo.barycentricCoord = computeBarycentricCoord(v0.position, v1.position, v2.position, ray.origin + ray.direction * ray.t);
                    hitInfo.texCoord = interpolateTexCoord(v0.texCoord, v1.texCoord, v2.texCoord, hitInfo.barycentricCoord);
                    glm::vec3 p = ray.origin + ray.t * ray.direction;
                    
                    if (features.enableNormalInterp) {
                        hitInfo.normal = interpolateNormal(v0.normal, v1.normal, v2.normal, computeBarycentricCoord(v0.position, v1.position, v2.position, p));
                        Ray debugIntN = Ray { p, hitInfo.normal, 2.0f };
                        Ray debugN0 = Ray { v0.position, v0.normal, 1.0f };
                        Ray debugN1 = Ray { v1.position, v1.normal, 1.0f };
                        Ray debugN2 = Ray { v2.position, v2.normal, 1.0f };
                        drawRay(debugN0, glm::vec3 { 1.0f, 0.0f, 0.0f });
                        drawRay(debugN1, glm::vec3 { 1.0f, 0.0f, 0.0f });
                        drawRay(debugN2, glm::vec3 { 1.0f, 0.0f, 0.0f });
                        drawRay(debugIntN, glm::vec3 { 0.0f, 1.0f, 0.0f });
                        hit = true;
                    } else {
                        hitInfo.normal = glm::normalize(glm::cross(v2.position - v0.position, v0.position - v1.position));
                        hit = true;
                    }
                }
            }
        } // she like to fuck me suck me, suck then fuck me, i'm the ice cream man, she chunky monkey
        // Intersect with spheres.
        for (const auto& sphere : m_pScene->spheres)
            hit |= intersectRayWithShape(sphere, ray, hitInfo);
        return hit;
    } else {
        Node root = listOfNodes[0];
        std::vector<Node> boxes;
        return intersectRecursion(features, ray, hitInfo, root, boxes);
    }
}

bool BoundingVolumeHierarchy::intersectRecursion(const Features& features, Ray& ray, HitInfo& hitInfo, const Node& node, std::vector<Node>& boxes) const
{
    AxisAlignedBox nodeBox { node.lBound, node.uBound };
    float temp = ray.t;
    if (!intersectRayWithShape(nodeBox, ray)) {
        ray.t = temp;
        return false;
    }
    ray.t = temp;
    boxes.push_back(node);
    bool success = false;
    if (!node.isInterior) {
        for (int t = 0; t < node.subs.size(); t+= 4) {
            Mesh& m = m_pScene->meshes[node.subs[t]];
            std::vector<Vertex> vertices;
            for (int v = 1; v <= 3; v++) {
                vertices.push_back(m.vertices[node.subs[t + v]]);
            }
            if (intersectRayWithTriangle(vertices[0].position, vertices[1].position, vertices[2].position, ray, hitInfo)) {
                    success = true;
                    hitInfo.material = m.material;
                    hitInfo.barycentricCoord = computeBarycentricCoord(vertices[0].position, vertices[1].position, vertices[2].position, ray.origin + ray.direction * ray.t);
                    hitInfo.texCoord = interpolateTexCoord(vertices[0].texCoord, vertices[1].texCoord, vertices[2].texCoord, hitInfo.barycentricCoord);
                    if (features.enableNormalInterp) {
                        glm::vec3 p = ray.origin + ray.t * ray.direction;
                        hitInfo.normal = interpolateNormal(vertices[0].normal, vertices[1].normal, vertices[2].normal, computeBarycentricCoord(vertices[0].position, vertices[1].position, vertices[2].position, p));
                        Ray debugIntN = Ray { p, hitInfo.normal, 2.0f };
                        Ray debugN0 = Ray { vertices[0].position, vertices[0].normal, 1.0f };
                        Ray debugN1 = Ray { vertices[1].position, vertices[1].normal, 1.0f };
                        Ray debugN2 = Ray { vertices[2].position, vertices[2].normal, 1.0f };
                        drawRay(debugN0, glm::vec3 { 1.0f, 0.0f, 0.0f });
                        drawRay(debugN1, glm::vec3 { 1.0f, 0.0f, 0.0f });
                        drawRay(debugN2, glm::vec3 { 1.0f, 0.0f, 0.0f });
                        drawRay(debugIntN, glm::vec3 { 0.0f, 1.0f, 0.0f });
                    } else {
                        hitInfo.normal = glm::normalize(glm::cross(vertices[0].position - vertices[1].position, vertices[0].position - vertices[2].position));
                    }
                    if (features.enableBVHDebug) {
                        drawTriangle(vertices[0], vertices[1], vertices[2]);
                        drawBoxes(boxes, node);
                        drawAABB(nodeBox, DrawMode::Wireframe, glm::vec3(0.0f, 1.0f, 0.0f), 1.0f);
                    }
            }
        }
    } else {
        const Node& firstChild = listOfNodes[node.subs[0]];
        const Node& secondChild = listOfNodes[node.subs[1]];

        glm::vec3 distanceFirstChild = ray.origin * 2.0f - (firstChild.uBound + firstChild.lBound);
        glm::vec3 distanceSecondChild = ray.origin * 2.0f - (secondChild.uBound + secondChild.lBound);
        float dFC = glm::dot(distanceFirstChild, distanceFirstChild);
        float dSC = glm::dot(distanceSecondChild, distanceSecondChild);
        bool rCloser =  dFC > dSC ;


        bool firstChildRes = intersectRecursion(features, ray, hitInfo, rCloser ? secondChild : firstChild, boxes);
        bool secondChildRes = intersectRecursion(features, ray, hitInfo, rCloser ? firstChild : secondChild, boxes);
        if (!success) {
            success = firstChildRes ? firstChildRes : secondChildRes;
        }
    }
    return success;
}

void BoundingVolumeHierarchy::drawBoxes(std::vector<Node>& boxes, const Node& target) const
{
    for (Node& n : boxes) {
        if (containsTriangle(n, target)) {
            AxisAlignedBox ng { n.lBound, n.uBound };
            drawAABB(ng, DrawMode::Wireframe, glm::vec3(0.0f, 1.0f, 0.0f), 1.0f);
        }
        else {
            AxisAlignedBox nr { n.lBound, n.uBound };
            drawAABB(nr, DrawMode::Wireframe, glm::vec3(1.0f, 0.0f, 0.0f), 1.0f);
        }
    }
}
bool BoundingVolumeHierarchy::containsTriangle(Node& n, const Node& target) const
{
    bool success = false;
    if (!n.isInterior) {
        if (n.isInterior == target.isInterior && n.lBound == target.lBound && n.uBound == target.uBound && n.subs == target.subs && n.level == target.level) {
            return true;
        }
        return false;
    }
    else {
        Node child1 = listOfNodes[n.subs[0]];
        Node child2 = listOfNodes[n.subs[1]];
        success = containsTriangle(child1, target) || containsTriangle(child2, target);
    }
    return success;
}

