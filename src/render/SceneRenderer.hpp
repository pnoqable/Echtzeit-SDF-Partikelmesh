#pragma once

#include "../simulation/ParticleSystem.hpp"
#include "../simulation/SDF.hpp"
#include "../mesh/Triangulation.hpp"
#include "../mesh/VoronoiDual.hpp"
#include "../platform/SystemTheme.hpp"
#include "ParticleBillboardRenderer.hpp"
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
    Color particleColor() const;

    void drawParticles(const ParticleSystem& system);
    void drawParticlesHeatmap(const ParticleSystem& system, float targetSpacing);
    void drawMesh(const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles, bool drawFill, bool wireframe, int topologyRevision);
    void drawMeshQuality(const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles, float poorAngleDeg);
    void drawVoronoiDual(const VoronoiDual& dual, bool drawFill, bool wireframe, int topologyRevision);
    void drawParticleSelection(const ParticleSystem& system, int index, bool showGrid, bool showNeighbors, bool showForces, bool showNormal);
    void drawSpatialGrid(const ParticleSystem& system);
    void drawSDFProjections(const ParticleSystem& system);
    void drawTrail(const std::vector<glm::vec3>& points);
    void drawSDFBounds(const SDF& sdf);
    void drawAxes(float length = 2.0f);

    // Einfache Beleuchtung mit zwei Punktlichtern (Key + Fill). Die Positionen
    // der Lichter werden aus der Bounding-Box des gezeichneten Meshes abgeleitet
    // und folgen damit automatisch jeder Form.
    void setLighting(bool enabled);
    void setLightIntensities(float key, float fill);
    void setAmbient(float ambient);
    // Kamera-feste Beleuchtung: Lichter behalten ihre Richtung relativ zur
    // Kamera (View-Raum) statt relativ zum Objekt (Weltraum). Die Achsen,
    // an denen Key/Fill ausgerichtet sind, bleiben beim Drehen der Kamera
    // bildschirmfest.
    void setCameraLighting(bool enabled) { m_cameraLighting = enabled; }
    bool cameraLightingEnabled() const { return m_cameraLighting; }

    bool lightingEnabled() const { return m_lighting; }
    float keyIntensity() const { return m_keyIntensity; }
    float fillIntensity() const { return m_fillIntensity; }
    float ambient() const { return m_ambient; }

    // Weiche Beleuchtung: interpolierte Vertex-Normalen statt geometrischer
    // Face-Normalen (Smooth Shading statt Flat Shading).
    void setSmoothShading(bool enabled) { m_smoothShading = enabled; }
    bool smoothShadingEnabled() const { return m_smoothShading; }

    // Raue Fraktal-Textur im Smooth-Modus: Normalen-Kippung ueber den
    // Weltraum-Gradienten eines fbm-Value-Noises; amplitude & von 0 bis 1,
    // freq skalaliert die Detailgroesse.
    void setRoughTexture(bool enabled) { m_roughTexture = enabled; }
    void setRoughness(float amplitude, float freq) { m_roughness = amplitude; m_roughFreq = freq; }
    bool roughTextureEnabled() const { return m_roughTexture; }

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

    void rebuildMesh(RenderMesh& rm, const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles);
    void updateMeshVertices(RenderMesh& rm, const std::vector<glm::vec3>& positions);
    void ensureMaterial();
    // Gemeinsamer gerenderter Mesh-Pass (gefuellte Flaeche + unabhaengiges
    // Drahtgitter), den Triangulation und Voronoi-Dual in gleicher Weise nutzen.
    void drawFillPass(RenderMesh& rm, const std::vector<glm::vec3>& positions);
    void drawWireframePass(RenderMesh& rm, const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles);
    void drawVoronoiWireframe(const VoronoiDual& dual);
    void unloadRenderMesh(RenderMesh& rm);

    RenderMesh m_mesh;
    RenderMesh m_dualMesh;
    ::Material m_material = {};
    bool m_materialReady = false;

    bool m_lighting = true;
    bool m_cameraLighting = false;
    bool m_smoothShading = false;
    bool m_roughTexture = false;
    float m_roughness = 0.25f;
    float m_roughFreq = 10.0f;
    float m_keyIntensity = 1.0f;
    float m_fillIntensity = 0.4f;
    float m_ambient = 0.12f;
    ::Shader m_lightShader = {};
    ::Material m_materialLit = {};
    int m_locAmbient = -1;
    int m_locLightPos0 = -1, m_locLightColor0 = -1;
    int m_locLightPos1 = -1, m_locLightColor1 = -1;
    int m_locShininess = -1;
    int m_locSmooth = -1;
    int m_locRough = -1;
    int m_locRoughFreq = -1;

    // Kamerafeste Partikel-Billboards (Hilfsklasse kapselt Shader + Instancing).
    ParticleBillboardRenderer m_billboards;
    std::vector<ParticleBillboardRenderer::Instance> m_particleInstances;
};