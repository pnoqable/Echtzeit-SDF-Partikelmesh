#pragma once

#include "../simulation/ParticleSystem.hpp"
#include "../simulation/SDF.hpp"
#include "../mesh/Triangulation.hpp"
#include "../platform/SystemTheme.hpp"
#include <raylib.h>
#include <glm/glm.hpp>
#include <vector>

class SceneRenderer {
public:
    SceneRenderer() = default;
    ~SceneRenderer();

    void setTheme(SystemTheme::Theme theme) { m_theme = theme; }
    SystemTheme::Theme theme() const { return m_theme; }
    Color backgroundColor() const;
    Color lineColor() const;

    void drawParticles(const ParticleSystem& system);
    void drawParticlesHeatmap(const ParticleSystem& system, float targetSpacing);
    void drawMesh(const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles, bool wireframe, int topologyRevision);
    void drawSDFBounds(const SDF& sdf);
    void drawAxes(float length = 2.0f);

private:
    SystemTheme::Theme m_theme = SystemTheme::Theme::Light;

    struct RenderMesh {
        int vertexCount = 0;
        int triangleCount = 0;
        int topologyRevision = -1;
        std::vector<float> vertices;
        std::vector<float> normals;
        std::vector<unsigned short> indices;
        ::Mesh handle = {};
        bool uploaded = false;
    };

    void rebuildMesh(const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles);
    void updateMeshVertices(const std::vector<glm::vec3>& positions);
    void ensureMaterial();

    RenderMesh m_mesh;
    ::Material m_material = {};
    bool m_materialReady = false;
};