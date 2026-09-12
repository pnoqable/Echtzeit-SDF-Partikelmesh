#include "SceneRenderer.hpp"
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
constexpr int kMeshVboCount = 7; // raylib Mesh::vboId[]
constexpr float kHeatmapUnderRatio = 0.85f;  // d < 0.85h
constexpr float kHeatmapOverRatio  = 1.15f;  // d > 1.15h
constexpr float kHeatmapFarRatio   = 1.6f;   // oberes Ende der Farbskala
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

void SceneRenderer::drawParticlesHeatmap(const ParticleSystem& system, float targetSpacing) {
    const auto& particles = system.particles;
    std::vector<float> nearest(particles.size(), std::numeric_limits<float>::max());
    for (const auto& pair : system.spatialHash().pairs()) {
        float d = glm::length(particles[pair.i].position - particles[pair.j].position);
        nearest[pair.i] = std::min(nearest[pair.i], d);
        nearest[pair.j] = std::min(nearest[pair.j], d);
    }
    for (size_t i = 0; i < particles.size(); ++i) {
        float ratio = nearest[i] == std::numeric_limits<float>::max()
            ? kHeatmapFarRatio
            : nearest[i] / targetSpacing;
        float t; Color color;
        if (ratio < kHeatmapUnderRatio) {
            // zu dicht: rot
            t = ratio / kHeatmapUnderRatio;
            color = ColorLerp(RED, GREEN, t);
        } else if (ratio <= kHeatmapOverRatio) {
            // Zielbereich: gruen
            color = GREEN;
        } else {
            // zu weit: blau
            t = std::min(1.0f, (ratio - kHeatmapOverRatio) / (kHeatmapFarRatio - kHeatmapOverRatio));
            color = ColorLerp(GREEN, BLUE, t);
        }
        DrawSphereEx({particles[i].position.x, particles[i].position.y, particles[i].position.z}, 0.005f, 4, 4, color);
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

void SceneRenderer::drawMesh(const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles, bool wireframe, int topologyRevision) {
    if (triangles.empty()) return;

    int count = static_cast<int>(triangles.size());
    if (!m_mesh.uploaded || m_mesh.vertexCount != static_cast<int>(positions.size()) ||
        m_mesh.triangleCount != count || m_mesh.topologyRevision != topologyRevision) {
        rebuildMesh(positions, triangles);
        m_mesh.topologyRevision = topologyRevision;
    } else {
        updateMeshVertices(positions);
    }

    ensureMaterial();
    DrawMesh(m_mesh.handle, m_material, MatrixIdentity());

    if (wireframe) {
        Color wf = lineColor();
        // Depth-Test bleibt AKTIV: So verdeckt die gefuellte Vorderseite
        // Drahtkanten der Rueckseite (vorher rlDisableDepthTest -> die
        // gesamte Rueckseite schien durch den Koerper hindurch).
        rlDisableBackfaceCulling();
        rlBegin(RL_LINES);
        // Drahtlinien minimal entlang der Vertex-Normalen nach aussen schieben,
        // damit sie auf der Vorderseite nicht mit der Flaeche z-fighten.
        const float wireOffset = 0.001f;
        for (const auto& t : triangles) {
            rlColor4ub(wf.r, wf.g, wf.b, wf.a);
            rlVertex3f(
                positions[t.i0].x + m_mesh.normals[t.i0 * 3 + 0] * wireOffset,
                positions[t.i0].y + m_mesh.normals[t.i0 * 3 + 1] * wireOffset,
                positions[t.i0].z + m_mesh.normals[t.i0 * 3 + 2] * wireOffset);
            rlVertex3f(
                positions[t.i1].x + m_mesh.normals[t.i1 * 3 + 0] * wireOffset,
                positions[t.i1].y + m_mesh.normals[t.i1 * 3 + 1] * wireOffset,
                positions[t.i1].z + m_mesh.normals[t.i1 * 3 + 2] * wireOffset);
            rlVertex3f(
                positions[t.i1].x + m_mesh.normals[t.i1 * 3 + 0] * wireOffset,
                positions[t.i1].y + m_mesh.normals[t.i1 * 3 + 1] * wireOffset,
                positions[t.i1].z + m_mesh.normals[t.i1 * 3 + 2] * wireOffset);
            rlVertex3f(
                positions[t.i2].x + m_mesh.normals[t.i2 * 3 + 0] * wireOffset,
                positions[t.i2].y + m_mesh.normals[t.i2 * 3 + 1] * wireOffset,
                positions[t.i2].z + m_mesh.normals[t.i2 * 3 + 2] * wireOffset);
            rlVertex3f(
                positions[t.i2].x + m_mesh.normals[t.i2 * 3 + 0] * wireOffset,
                positions[t.i2].y + m_mesh.normals[t.i2 * 3 + 1] * wireOffset,
                positions[t.i2].z + m_mesh.normals[t.i2 * 3 + 2] * wireOffset);
            rlVertex3f(
                positions[t.i0].x + m_mesh.normals[t.i0 * 3 + 0] * wireOffset,
                positions[t.i0].y + m_mesh.normals[t.i0 * 3 + 1] * wireOffset,
                positions[t.i0].z + m_mesh.normals[t.i0 * 3 + 2] * wireOffset);
        }
        rlEnd();
        rlEnableBackfaceCulling();
    }
}

void SceneRenderer::drawMeshQuality(const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles, float poorAngleDeg) {
    if (positions.empty() || triangles.empty()) return;
    // Depth-Test bleibt AKTIV: Qualitaetslinien der Rueckseite sollen vom
    // gefuellten Mesh verdeckt werden (wie beim Wireframe in drawMesh()).
    rlDisableBackfaceCulling();
    rlBegin(RL_LINES);
    for (const auto& t : triangles) {
        if (t.i0 >= positions.size() || t.i1 >= positions.size() || t.i2 >= positions.size()) continue;
        const glm::vec3& a = positions[t.i0];
        const glm::vec3& b = positions[t.i1];
        const glm::vec3& c = positions[t.i2];
        glm::vec3 ab = glm::normalize(b - a), ac = glm::normalize(c - a);
        glm::vec3 ba = glm::normalize(a - b), bc = glm::normalize(c - b);
        glm::vec3 ca = glm::normalize(a - c), cb = glm::normalize(b - c);
        float angA = std::acos(glm::clamp(glm::dot(ab, ac), -1.0f, 1.0f)) * 180.0f / glm::pi<float>();
        float angB = std::acos(glm::clamp(glm::dot(ba, bc), -1.0f, 1.0f)) * 180.0f / glm::pi<float>();
        float angC = std::acos(glm::clamp(glm::dot(ca, cb), -1.0f, 1.0f)) * 180.0f / glm::pi<float>();
        float minDeg = std::min({angA, angB, angC});
        // grün -> gelb -> rot nach Qualität
        Color col;
        if (minDeg < poorAngleDeg) col = RED;
        else col = ColorLerp(RED, GREEN, std::min(1.0f, (minDeg - poorAngleDeg) / poorAngleDeg));
        rlColor4ub(col.r, col.g, col.b, col.a);
        rlVertex3f(a.x, a.y, a.z); rlVertex3f(b.x, b.y, b.z);
        rlVertex3f(b.x, b.y, b.z); rlVertex3f(c.x, c.y, c.z);
        rlVertex3f(c.x, c.y, c.z); rlVertex3f(a.x, a.y, a.z);
    }
    rlEnd();
    rlEnableBackfaceCulling();
}

void SceneRenderer::drawParticleSelection(const ParticleSystem& system, int index, bool showGrid, bool showNeighbors, bool showForces, bool showNormal) {
    if (index < 0 || index >= static_cast<int>(system.particles.size())) return;
    const auto& p = system.particles[index];
    glm::vec3 pos = p.position;

    if (showGrid) {
        float cs = system.spatialHash().cellSize();
        SpatialHash::CellKey key = system.spatialHash().cellOf(pos);
        glm::vec3 center{ (key.x + 0.5f) * cs, (key.y + 0.5f) * cs, (key.z + 0.5f) * cs };
        Color dim = Fade(LIGHTGRAY, 0.6f);
        DrawCubeWires({ center.x, center.y, center.z }, cs, cs, cs, dim);
    }

    if (showNeighbors) {
        Color nb = Fade(SKYBLUE, 0.9f);
        rlBegin(RL_LINES);
        for (const auto& pair : system.spatialHash().pairs()) {
            if (pair.i == static_cast<uint32_t>(index) || pair.j == static_cast<uint32_t>(index)) {
                uint32_t other = pair.i == static_cast<uint32_t>(index) ? pair.j : pair.i;
                const glm::vec3& q = system.particles[other].position;
                rlColor4ub(nb.r, nb.g, nb.b, nb.a);
                rlVertex3f(pos.x, pos.y, pos.z);
                rlVertex3f(q.x, q.y, q.z);
            }
        }
        rlEnd();
    }

    if (showForces) {
        glm::vec3 force(0.0f);
        float R = system.parameters.repulsionRadius;
        float k = system.parameters.repulsionStrength;
        for (const auto& pair : system.spatialHash().pairs()) {
            if (pair.i != static_cast<uint32_t>(index) && pair.j != static_cast<uint32_t>(index)) continue;
            uint32_t other = pair.i == static_cast<uint32_t>(index) ? pair.j : pair.i;
            glm::vec3 diff = system.particles[other].position - pos;
            float d = glm::length(diff);
            if (d < 1e-6f || d >= R) continue;
            float w = k * (1.0f - d / R) * (1.0f - d / R) / d;
            force += -w * (diff / d);
        }
        glm::vec3 tangent = force - glm::dot(force, p.normal) * p.normal;
        float scale = 0.15f;
        glm::vec3 endTotal = pos + force * scale;
        glm::vec3 endTan = pos + tangent * scale;
        DrawLine3D({ pos.x, pos.y, pos.z }, { endTotal.x, endTotal.y, endTotal.z }, YELLOW);
        DrawLine3D({ pos.x, pos.y, pos.z }, { endTan.x, endTan.y, endTan.z }, MAGENTA);
    }

    if (showNormal) {
        glm::vec3 end = pos + p.normal * 0.1f;
        DrawLine3D({ pos.x, pos.y, pos.z }, { end.x, end.y, end.z }, GREEN);
    }

    DrawSphereEx({ pos.x, pos.y, pos.z }, 0.012f, 8, 8, WHITE);
}

void SceneRenderer::drawTrail(const std::vector<glm::vec3>& points) {
    if (points.empty()) return;
    Color c = Fade(ORANGE, 0.9f);
    for (size_t i = 1; i < points.size(); ++i) {
        DrawLine3D({ points[i-1].x, points[i-1].y, points[i-1].z },
                   { points[i].x, points[i].y, points[i].z }, c);
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