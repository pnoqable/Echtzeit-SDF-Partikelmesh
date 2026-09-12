#include "SceneRenderer.hpp"
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <cstring>

namespace {
constexpr int kMeshVboCount = 7; // raylib Mesh::vboId[]
}

SceneRenderer::~SceneRenderer() {
    if (m_mesh.uploaded) {
        rlUnloadVertexArray(m_mesh.handle.vaoId);
        for (int i = 0; i < kMeshVboCount; ++i) {
            if (m_mesh.handle.vboId[i])
                rlUnloadVertexBuffer(m_mesh.handle.vboId[i]);
        }
    }
}

Color SceneRenderer::backgroundColor() const {
    return (m_theme == SystemTheme::Theme::Dark) ? Color{26, 26, 34, 255} : RAYWHITE;
}

Color SceneRenderer::lineColor() const {
    return (m_theme == SystemTheme::Theme::Dark) ? LIGHTGRAY : DARKGRAY;
}

void SceneRenderer::drawParticles(const ParticleSystem& system) {
    for (const auto& p : system.particles) {
        DrawSphereEx({p.position.x, p.position.y, p.position.z}, 0.005f, 4, 4, RED);
    }
}

void SceneRenderer::rebuildMesh(const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles) {
    int vertexCount = static_cast<int>(positions.size());
    int triangleCount = static_cast<int>(triangles.size());

    m_mesh.vertexCount = vertexCount;
    m_mesh.triangleCount = triangleCount;

    m_mesh.vertices.assign(static_cast<size_t>(vertexCount) * 3, 0.0f);
    for (int i = 0; i < vertexCount; ++i) {
        m_mesh.vertices[i * 3 + 0] = positions[i].x;
        m_mesh.vertices[i * 3 + 1] = positions[i].y;
        m_mesh.vertices[i * 3 + 2] = positions[i].z;
    }

    m_mesh.indices.assign(static_cast<size_t>(triangleCount) * 3, 0);
    for (int t = 0; t < triangleCount; ++t) {
        m_mesh.indices[t * 3 + 0] = static_cast<unsigned short>(triangles[t].i0);
        m_mesh.indices[t * 3 + 1] = static_cast<unsigned short>(triangles[t].i1);
        m_mesh.indices[t * 3 + 2] = static_cast<unsigned short>(triangles[t].i2);
    }

    // Vertex-Normalen als Durchschnitt der Face-Normalen
    std::vector<glm::vec3> faceNormals(triangleCount, glm::vec3(0.0f));
    for (int t = 0; t < triangleCount; ++t) {
        const auto& tri = triangles[t];
        glm::vec3 e1 = positions[tri.i1] - positions[tri.i0];
        glm::vec3 e2 = positions[tri.i2] - positions[tri.i0];
        glm::vec3 fn = glm::cross(e1, e2);
        float len = glm::length(fn);
        if (len > 1e-8f) faceNormals[t] = fn / len;
    }

    m_mesh.normals.assign(static_cast<size_t>(vertexCount) * 3, 0.0f);
    for (int t = 0; t < triangleCount; ++t) {
        const auto& tri = triangles[t];
        m_mesh.normals[tri.i0 * 3 + 0] += faceNormals[t].x;
        m_mesh.normals[tri.i0 * 3 + 1] += faceNormals[t].y;
        m_mesh.normals[tri.i0 * 3 + 2] += faceNormals[t].z;
        m_mesh.normals[tri.i1 * 3 + 0] += faceNormals[t].x;
        m_mesh.normals[tri.i1 * 3 + 1] += faceNormals[t].y;
        m_mesh.normals[tri.i1 * 3 + 2] += faceNormals[t].z;
        m_mesh.normals[tri.i2 * 3 + 0] += faceNormals[t].x;
        m_mesh.normals[tri.i2 * 3 + 1] += faceNormals[t].y;
        m_mesh.normals[tri.i2 * 3 + 2] += faceNormals[t].z;
    }
    for (int i = 0; i < vertexCount; ++i) {
        glm::vec3 n(m_mesh.normals[i * 3 + 0], m_mesh.normals[i * 3 + 1], m_mesh.normals[i * 3 + 2]);
        float len = glm::length(n);
        if (len > 1e-8f) n /= len;
        m_mesh.normals[i * 3 + 0] = n.x;
        m_mesh.normals[i * 3 + 1] = n.y;
        m_mesh.normals[i * 3 + 2] = n.z;
    }

    if (m_mesh.uploaded) {
        rlUnloadVertexArray(m_mesh.handle.vaoId);
        for (int i = 0; i < kMeshVboCount; ++i) {
            if (m_mesh.handle.vboId[i])
                rlUnloadVertexBuffer(m_mesh.handle.vboId[i]);
        }
        m_mesh.handle = {};
        m_mesh.uploaded = false;
    }

    m_mesh.handle.vertexCount = vertexCount;
    m_mesh.handle.triangleCount = triangleCount;
    m_mesh.handle.vertices = m_mesh.vertices.data();
    m_mesh.handle.indices = m_mesh.indices.data();
    m_mesh.handle.normals = m_mesh.normals.data();
    m_mesh.handle.texcoords = nullptr;

    UploadMesh(&m_mesh.handle, false);
    m_mesh.uploaded = true;
}

void SceneRenderer::updateMeshVertices(const std::vector<glm::vec3>& positions) {
    if (!m_mesh.uploaded) return;
    for (int i = 0; i < m_mesh.vertexCount && i < static_cast<int>(positions.size()); ++i) {
        m_mesh.vertices[i * 3 + 0] = positions[i].x;
        m_mesh.vertices[i * 3 + 1] = positions[i].y;
        m_mesh.vertices[i * 3 + 2] = positions[i].z;
    }
    rlUpdateVertexBuffer(m_mesh.handle.vboId[0], m_mesh.vertices.data(),
        m_mesh.vertexCount * 3 * sizeof(float), 0);
}

void SceneRenderer::ensureMaterial() {
    if (!m_materialReady) {
        m_material = LoadMaterialDefault();
        m_materialReady = true;
    }
    Color diffuse = (m_theme == SystemTheme::Theme::Dark) ? Color{70, 74, 86, 255} : RAYWHITE;
    m_material.maps[MATERIAL_MAP_DIFFUSE].color = diffuse;
}

void SceneRenderer::drawMesh(const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles, bool wireframe) {
    if (triangles.empty()) return;

    int count = static_cast<int>(triangles.size());
    if (!m_mesh.uploaded || m_mesh.vertexCount != static_cast<int>(positions.size()) ||
        m_mesh.triangleCount != count) {
        rebuildMesh(positions, triangles);
    } else {
        updateMeshVertices(positions);
    }

    ensureMaterial();
    DrawMesh(m_mesh.handle, m_material, MatrixIdentity());

    if (wireframe) {
        Color wf = lineColor();
        rlDisableDepthTest();
        rlDisableBackfaceCulling();
        rlBegin(RL_LINES);
        for (const auto& t : triangles) {
            rlColor4ub(wf.r, wf.g, wf.b, wf.a);
            rlVertex3f(positions[t.i0].x, positions[t.i0].y, positions[t.i0].z);
            rlVertex3f(positions[t.i1].x, positions[t.i1].y, positions[t.i1].z);
            rlVertex3f(positions[t.i1].x, positions[t.i1].y, positions[t.i1].z);
            rlVertex3f(positions[t.i2].x, positions[t.i2].y, positions[t.i2].z);
            rlVertex3f(positions[t.i2].x, positions[t.i2].y, positions[t.i2].z);
            rlVertex3f(positions[t.i0].x, positions[t.i0].y, positions[t.i0].z);
        }
        rlEnd();
        rlEnableDepthTest();
        rlEnableBackfaceCulling();
    }
}

void SceneRenderer::drawSDFBounds(const SDF& sdf) {
    glm::vec3 bmin = sdf.boundsMin();
    glm::vec3 bmax = sdf.boundsMax();
    Vector3 size = {
        bmax.x - bmin.x,
        bmax.y - bmin.y,
        bmax.z - bmin.z
    };
    Vector3 center = {
        (bmin.x + bmax.x) * 0.5f,
        (bmin.y + bmax.y) * 0.5f,
        (bmin.z + bmax.z) * 0.5f
    };
    DrawCubeWires(center, size.x, size.y, size.z, lineColor());
}

void SceneRenderer::drawAxes(float length) {
    DrawLine3D({0,0,0}, {length,0,0}, RED);
    DrawLine3D({0,0,0}, {0,length,0}, GREEN);
    DrawLine3D({0,0,0}, {0,0,length}, BLUE);
}