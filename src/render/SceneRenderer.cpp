#include "SceneRenderer.hpp"
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
constexpr int kMeshVboCount = 7; // raylib Mesh::vboId[]
constexpr float kHeatmapUnderRatio = 0.85f;  // d < 0.85h
constexpr float kHeatmapOverRatio  = 1.15f;  // d > 1.15h
constexpr float kHeatmapFarRatio   = 1.6f;   // oberes Ende der Farbskala

// Vertex-Shader: Standard-Contract von raylib (Attribute/Uniforms), transformiert
// die Position in den View-Raum. Die Normale wird ebenfalls interpoliert
// uebertragen, damit der Fragment-Shader wahlweise Smooth- (Vertex-Normalen)
// oder Flat-Shading (geometrische Face-Normale) verwenden kann.
const char* kLightVS = R"GLSL(
#version 330

in vec3 vertexPosition;
in vec3 vertexNormal;

uniform mat4 matModel;
uniform mat4 matView;
uniform mat4 matProjection;

out vec3 vViewPos;
out vec3 vViewNormal;
out vec3 vWorldPos;

void main() {
    vec4 wpos = matModel * vec4(vertexPosition, 1.0);
    vViewPos = (matView * wpos).xyz;
    vViewNormal = (matView * matModel * vec4(vertexNormal, 0.0)).xyz;
    vWorldPos = wpos.xyz;
    gl_Position = matProjection * matView * wpos;
}
)GLSL";

// Fragment-Shader: Zwei-Punktlicht-Beleuchtung (Key + Fill) im View-Raum.
// Je nach uSmooth wird die Normale aus den Screen-Space-Partialableitungen der
// interpolierenden Position rekonstruiert (Flat Shading, rein geometrisch)
// oder die interpolierte Vertex-Normale verwendet (Smooth Shading, weichere
// Kruemmungsdarstellung). Im Smooth-Modus kann uRoughness die Normale mit dem
// Gradienten eines fraktalen Value-Noises (fbm, in Weltkoordinaten) kippen
// und so eine rauhe Fraktal-Oberflaechenstruktur erzeugen. colDiffuse
// (Basis-Albedo) wird von raylib DrawMesh() gesetzt.
const char* kLightFS = R"GLSL(
#version 330

in vec3 vViewPos;
in vec3 vViewNormal;
in vec3 vWorldPos;

uniform vec4 colDiffuse;
uniform float uAmbient;
uniform vec3 uLightPos0;
uniform vec3 uLightColor0;
uniform vec3 uLightPos1;
uniform vec3 uLightColor1;
uniform float uShininess;
uniform int uSmooth;
uniform float uRoughness;
uniform float uRoughFreq;
uniform mat4 matView;

out vec4 finalColor;

// Gradient-Noise (3D Value-Noise) fuer die rauhe Fraktal-Textur: billig und
// ohne Textursampler, direkt im Weltraum adressierbar.
float hash(vec3 p) {
    p = fract(p * 0.1031);
    p += dot(p, p.zyx + 31.32);
    return fract((p.x + p.y) * p.z);
}

float noise(vec3 p) {
    vec3 i = floor(p);
    vec3 f = fract(p);
    vec3 u = f * f * (3.0 - 2.0 * f);
    return mix(
        mix(mix(hash(i),               hash(i + vec3(1,0,0)), u.x),
            mix(hash(i + vec3(0,1,0)), hash(i + vec3(1,1,0)), u.x), u.y),
        mix(mix(hash(i + vec3(0,0,1)), hash(i + vec3(1,0,1)), u.x),
            mix(hash(i + vec3(0,1,1)), hash(i + vec3(1,1,1)), u.x), u.y),
        u.z);
}

float fbm(vec3 p) {
    float v = 0.0;
    float a = 0.5;
    for (int i = 0; i < 4; ++i) {
        v += a * noise(p);
        p = p * 2.03 + vec3(11.61);
        a *= 0.5;
    }
    return v;
}

// Numerischer Gradient des Fraktal-Feldes (zentrale Differenzen, Weltraum).
vec3 fbmGrad(vec3 p) {
    float e = 0.08;
    return vec3(
        fbm(p + vec3(e, 0.0, 0.0)) - fbm(p - vec3(e, 0.0, 0.0)),
        fbm(p + vec3(0.0, e, 0.0)) - fbm(p - vec3(0.0, e, 0.0)),
        fbm(p + vec3(0.0, 0.0, e)) - fbm(p - vec3(0.0, 0.0, e))) / (2.0 * e);
}

vec3 shadeLight(vec3 V, vec3 N, vec3 lightPos, vec3 lightColor) {
    vec3 L = lightPos - vViewPos;
    float dist = length(L);
    L /= max(dist, 1e-5);
    // Weicher quadratischer Abfall, kein heisser Punktlicht-Blowup in der Naehe.
    float attenuation = 1.0 / (1.0 + 0.09 * dist + 0.032 * dist * dist);
    float ndl = max(dot(N, L), 0.0);
    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), uShininess);
    return lightColor * (ndl + spec * 0.25) * attenuation;
}

void main() {
    vec3 N;
    if (uSmooth != 0) {
        // Interpolierte Vertex-Normale (Smooth Shading). KEINE Blickrichtungs-
        // Umkehr: die gespeicherten Normalen zeigen konsistent nach aussen. Ein
        // View-Flip wuerde die Lichtabgewandte Seite an flachen Randwinkeln
        // aufhellen (dot(N,V) < 0 -> Normale Richtung Licht umklappen).
        N = normalize(vViewNormal);
        if (uRoughness > 0.0) {
            // Fraktale Rauheit: Feld-Gradient im Weltraum, wird aber erst nach
            // einer Drehung in den View-Raum auf die (View-)Normale angewandt.
            // Vorher verarbeiteten wir hier Welt- und View-Koordinaten gemischt.
            vec3 g = (matView * vec4(fbmGrad(vWorldPos * uRoughFreq), 0.0)).xyz;
            N = normalize(N - uRoughness * g);
            if (dot(N, vViewNormal) < 0.0) N = -N; // Konsistenz zur gespeicherten Normale
        }
    } else {
        // Geometrische Flächennormale (Flat Shading) aus den Screen-Space-
        // Partialableitungen. Bei sehr flachen Winkeln (nahezu Kantensicht)
        // kollabiert das Kreuzprodukt -> Fallback auf die interpolierte Normale.
        // Ausrichtung an der gespeicherten, konsistent aussen gerichteten Normale
        // statt an der Blickrichtung; so bleiben Lichtabgewandte Seiten dunkel.
        vec3 gN = cross(dFdx(vViewPos), dFdy(vViewPos));
        N = length(gN) > 1e-9 ? normalize(gN) : normalize(vViewNormal);
        if (dot(N, vViewNormal) < 0.0) N = -N;
    }
    vec3 V = normalize(-vViewPos);
    vec3 acc = vec3(0.0);
    acc += shadeLight(V, N, uLightPos0, uLightColor0);
    acc += shadeLight(V, N, uLightPos1, uLightColor1);
    vec3 color = clamp(colDiffuse.rgb * (vec3(uAmbient) + acc), 0.0, 1.0);
    // Das Framebuffer ist sRGB: lineare Beleuchtungswerte wuerden ohne Gamma-
    // Enkodierung zu dunkel erscheinen (Schattenseiten ~0.1..0.3 wirken schwarz).
    color = pow(color, vec3(1.0 / 2.2));
    finalColor = vec4(color, colDiffuse.a);
}
)GLSL";

// Weltpositionen der beiden Lichter: Skalierung mit dem Scene-Radius, damit die
// Beleuchtung bei jeder Formgröße gleich wirkt.
constexpr glm::vec3 kLightKeyDir  = glm::vec3(-0.62f, 0.78f, 0.42f);
constexpr glm::vec3 kLightFillDir = glm::vec3(0.70f, -0.35f, -0.62f);
constexpr glm::vec3 kLightKeyColor  = glm::vec3(1.0f, 0.98f, 0.92f);
constexpr glm::vec3 kLightFillColor = glm::vec3(0.55f, 0.66f, 1.0f);
constexpr float kLightDistScale = 2.4f;
constexpr float kShininess = 28.0f;
} // namespace

void SceneRenderer::unloadRenderMesh(RenderMesh& rm) {
    if (!rm.uploaded) return;
    rlUnloadVertexArray(rm.handle.vaoId);
    for (int i = 0; i < kMeshVboCount; ++i) {
        if (rm.handle.vboId[i])
            rlUnloadVertexBuffer(rm.handle.vboId[i]);
    }
    rm.handle = {};
    rm.uploaded = false;
}

SceneRenderer::~SceneRenderer() {
    unloadRenderMesh(m_mesh);
    unloadRenderMesh(m_dualMesh);
    unloadRenderMesh(m_selectedMesh);
    if (m_lightShader.id != 0)
        UnloadShader(m_lightShader);
    m_billboards.unload();
    m_edgeLines.unload();
}

Color SceneRenderer::backgroundColor() const {
    return (m_theme == SystemTheme::Theme::Dark) ? Color{26, 26, 34, 255} : RAYWHITE;
}

Color SceneRenderer::lineColor() const {
    return (m_theme == SystemTheme::Theme::Dark) ? LIGHTGRAY : DARKGRAY;
}

// Dezente Partikelfarbe (kein Signalfarb-Rot wie in der Heatmap): softes
// Silbergrau auf dunklem, gedecktes Schiefergrau auf hellem Hintergrund.
Color SceneRenderer::particleColor() const {
    return (m_theme == SystemTheme::Theme::Dark)
        ? Color{ 205, 208, 220, 255 }
        : Color{ 118, 120, 135, 255 };
}

void SceneRenderer::drawParticles(const ParticleSystem& system) {
    // Instanz-Beschreibung pro Partikel fuellen; die Kugelgeometrie liefert der
    // Billboard-Renderer (kapselt Shader + Achtkant-Instancing).
    auto& inst = m_particleInstances;
    inst.resize(system.particles.size());
    const Color color = particleColor();
    for (size_t i = 0; i < inst.size(); ++i) {
        const auto& p = system.particles[i];
        inst[i] = { p.position, p.normal, color };
    }
    m_billboards.draw(inst.data(), inst.size());
}

void SceneRenderer::drawParticlesHeatmap(const ParticleSystem& system, float targetSpacing) {
    const auto& particles = system.particles;
    std::vector<float> nearest(particles.size(), std::numeric_limits<float>::max());
    for (const auto& pair : system.spatialHash().pairs()) {
        float d = glm::length(particles[pair.i].position - particles[pair.j].position);
        nearest[pair.i] = std::min(nearest[pair.i], d);
        nearest[pair.j] = std::min(nearest[pair.j], d);
    }
    auto& inst = m_particleInstances;
    inst.resize(particles.size());
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
        const auto& p = particles[i];
        inst[i] = { p.position, p.normal, color };
    }
    m_billboards.draw(inst.data(), inst.size());
}

void SceneRenderer::rebuildMesh(RenderMesh& rm, const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles) {
    int vertexCount = static_cast<int>(positions.size());
    int triangleCount = static_cast<int>(triangles.size());

    rm.vertexCount = vertexCount;
    rm.triangleCount = triangleCount;

    rm.vertices.assign(static_cast<size_t>(vertexCount) * 3, 0.0f);
    for (int i = 0; i < vertexCount; ++i) {
        rm.vertices[i * 3 + 0] = positions[i].x;
        rm.vertices[i * 3 + 1] = positions[i].y;
        rm.vertices[i * 3 + 2] = positions[i].z;
    }

    rm.indices.assign(static_cast<size_t>(triangleCount) * 3, 0);
    for (int t = 0; t < triangleCount; ++t) {
        rm.indices[t * 3 + 0] = static_cast<unsigned short>(triangles[t].i0);
        rm.indices[t * 3 + 1] = static_cast<unsigned short>(triangles[t].i1);
        rm.indices[t * 3 + 2] = static_cast<unsigned short>(triangles[t].i2);
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

    rm.normals.assign(static_cast<size_t>(vertexCount) * 3, 0.0f);
    for (int t = 0; t < triangleCount; ++t) {
        const auto& tri = triangles[t];
        rm.normals[tri.i0 * 3 + 0] += faceNormals[t].x;
        rm.normals[tri.i0 * 3 + 1] += faceNormals[t].y;
        rm.normals[tri.i0 * 3 + 2] += faceNormals[t].z;
        rm.normals[tri.i1 * 3 + 0] += faceNormals[t].x;
        rm.normals[tri.i1 * 3 + 1] += faceNormals[t].y;
        rm.normals[tri.i1 * 3 + 2] += faceNormals[t].z;
        rm.normals[tri.i2 * 3 + 0] += faceNormals[t].x;
        rm.normals[tri.i2 * 3 + 1] += faceNormals[t].y;
        rm.normals[tri.i2 * 3 + 2] += faceNormals[t].z;
    }
    for (int i = 0; i < vertexCount; ++i) {
        glm::vec3 n(rm.normals[i * 3 + 0], rm.normals[i * 3 + 1], rm.normals[i * 3 + 2]);
        float len = glm::length(n);
        if (len > 1e-8f) n /= len;
        rm.normals[i * 3 + 0] = n.x;
        rm.normals[i * 3 + 1] = n.y;
        rm.normals[i * 3 + 2] = n.z;
    }

    unloadRenderMesh(rm);

    rm.handle.vertexCount = vertexCount;
    rm.handle.triangleCount = triangleCount;
    rm.handle.vertices = rm.vertices.data();
    rm.handle.indices = rm.indices.data();
    rm.handle.normals = rm.normals.data();
    rm.handle.texcoords = nullptr;

    UploadMesh(&rm.handle, false);
    rm.uploaded = true;
}

void SceneRenderer::updateMeshVertices(RenderMesh& rm, const std::vector<glm::vec3>& positions) {
    if (!rm.uploaded) return;
    for (int i = 0; i < rm.vertexCount && i < static_cast<int>(positions.size()); ++i) {
        rm.vertices[i * 3 + 0] = positions[i].x;
        rm.vertices[i * 3 + 1] = positions[i].y;
        rm.vertices[i * 3 + 2] = positions[i].z;
    }
    rlUpdateVertexBuffer(rm.handle.vboId[0], rm.vertices.data(),
        rm.vertexCount * 3 * sizeof(float), 0);
}

void SceneRenderer::ensureMaterial() {
    if (!m_materialReady) {
        m_material = LoadMaterialDefault();
        m_materialReady = true;
    }
    Color diffuse = (m_theme == SystemTheme::Theme::Dark) ? Color{70, 74, 86, 255} : RAYWHITE;
    m_material.maps[MATERIAL_MAP_DIFFUSE].color = diffuse;

    if (m_lightShader.id == 0) {
        m_lightShader = LoadShaderFromMemory(kLightVS, kLightFS);
        if (m_lightShader.id != 0) {
            m_materialLit = LoadMaterialDefault();
            m_materialLit.shader = m_lightShader;
            m_locAmbient     = GetShaderLocation(m_lightShader, "uAmbient");
            m_locLightPos0   = GetShaderLocation(m_lightShader, "uLightPos0");
            m_locLightColor0 = GetShaderLocation(m_lightShader, "uLightColor0");
            m_locLightPos1   = GetShaderLocation(m_lightShader, "uLightPos1");
            m_locLightColor1 = GetShaderLocation(m_lightShader, "uLightColor1");
            m_locShininess   = GetShaderLocation(m_lightShader, "uShininess");
            m_locSmooth      = GetShaderLocation(m_lightShader, "uSmooth");
            m_locRough       = GetShaderLocation(m_lightShader, "uRoughness");
            m_locRoughFreq   = GetShaderLocation(m_lightShader, "uRoughFreq");
        }
    }
    if (m_materialLit.shader.id != 0)
        m_materialLit.maps[MATERIAL_MAP_DIFFUSE].color = diffuse;

    // Akzent-Material der ausgewaehlten Voronoi-Zelle: gleicher Light-Shader,
    // aber warme, deutlich abgesetzte Grundfarbe.
    if (m_materialSelected.shader.id == 0) {
        m_materialSelected = LoadMaterialDefault();
        m_materialSelected.shader = m_lightShader;
    }
    Color accent = (m_theme == SystemTheme::Theme::Dark) ? Color{255, 140, 40, 255} : Color{210, 90, 20, 255};
    m_materialSelected.maps[MATERIAL_MAP_DIFFUSE].color = accent;
}

void SceneRenderer::setLighting(bool enabled) {
    m_lighting = enabled;
}

void SceneRenderer::setLightIntensities(float key, float fill) {
    m_keyIntensity = key;
    m_fillIntensity = fill;
}

void SceneRenderer::setAmbient(float ambient) {
    m_ambient = std::max(0.0f, ambient);
}

void SceneRenderer::syncMesh(const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles, int topologyRevision) {
    if (triangles.empty()) return;

    int count = static_cast<int>(triangles.size());
    if (!m_mesh.uploaded || m_mesh.vertexCount != static_cast<int>(positions.size()) ||
        m_mesh.triangleCount != count || m_mesh.topologyRevision != topologyRevision) {
        rebuildMesh(m_mesh, positions, triangles);
        m_mesh.topologyRevision = topologyRevision;
    } else {
        updateMeshVertices(m_mesh, positions);
    }

    ensureMaterial();
}

void SceneRenderer::drawMeshFill(const std::vector<glm::vec3>& positions) {
    drawFillPass(m_mesh, positions);
}

void SceneRenderer::drawMeshWireframe(const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles) {
    drawWireframePass(m_mesh, positions, triangles);
}

void SceneRenderer::drawFillPass(RenderMesh& rm, const std::vector<glm::vec3>& positions, ::Material* overrideMaterial) {
    const bool useLit = overrideMaterial
        ? (overrideMaterial->shader.id != 0)
        : (m_lighting && m_materialLit.shader.id != 0);
    const ::Material* drawMat = overrideMaterial ? overrideMaterial : (useLit ? &m_materialLit : &m_material);

    if (useLit) {
        // Bounding-Box der aktuellen Vertex-Positionen als Bulle fuer die
        // Lichtpositionen: Center + Richtung * (Radius * Skalierung).
        glm::vec3 bmin(std::numeric_limits<float>::max());
        glm::vec3 bmax(std::numeric_limits<float>::lowest());
        for (const auto& p : positions) {
            bmin = glm::min(bmin, p);
            bmax = glm::max(bmax, p);
        }
        glm::vec3 center = 0.5f * (bmin + bmax);
        float radius = std::max(0.1f, 0.5f * glm::length(bmax - bmin));
        Matrix view = rlGetMatrixModelview();

        auto viewSpaceLight = [&](const glm::vec3& dir, const glm::vec3& color, float intensity, glm::vec3& outPos, glm::vec3& outColor) {
            glm::vec3 nd = glm::normalize(dir);
            if (m_cameraLighting) {
                // Kamera-fest: Die Lichtrichtung bleibt relativ zur Kamera
                // konstant (bildschirmfest), unabhaengig davon, wie die Kamera
                // um das Objekt dreht. Nur die Zentrums-Uebersetzung wird in den
                // View-Raum transformiert, die Richtung diesen nicht gedreht.
                Vector3 c = Vector3Transform({ center.x, center.y, center.z }, view);
                outPos = { c.x + nd.x * (radius * kLightDistScale),
                           c.y + nd.y * (radius * kLightDistScale),
                           c.z + nd.z * (radius * kLightDistScale) };
            } else {
                // Global (objektfest): Lichtposition im Weltraum um den
                // Objekt-Mittelpunkt und zusammen mit dem Objekt durch die
                // Kamera-Matrix gedreht.
                glm::vec3 wpos = center + nd * (radius * kLightDistScale);
                Vector3 vp = Vector3Transform({ wpos.x, wpos.y, wpos.z }, view);
                outPos = { vp.x, vp.y, vp.z };
            }
            outColor = color * intensity;
        };

        glm::vec3 pos0, color0, pos1, color1;
        viewSpaceLight(kLightKeyDir, kLightKeyColor, m_keyIntensity, pos0, color0);
        viewSpaceLight(kLightFillDir, kLightFillColor, m_fillIntensity, pos1, color1);

        float ambient = m_ambient;
        float shininess = kShininess;
        // WICHTIG: raylibs SetShaderValue ruft glUniform* direkt auf das aktuell
        // gebundene Programm. Erst rlEnableShader() aktiviert unseren Licht-Shader,
        // vorher wuerden die Uniformen im Default-Shader landen (dunkles Mesh).
        rlEnableShader(m_lightShader.id);
        if (m_locAmbient != -1)     SetShaderValue(m_lightShader, m_locAmbient,     &ambient,   SHADER_UNIFORM_FLOAT);
        if (m_locShininess != -1)   SetShaderValue(m_lightShader, m_locShininess,   &shininess, SHADER_UNIFORM_FLOAT);
        if (m_locLightPos0 != -1)   SetShaderValue(m_lightShader, m_locLightPos0,   glm::value_ptr(pos0),  SHADER_UNIFORM_VEC3);
        if (m_locLightColor0 != -1) SetShaderValue(m_lightShader, m_locLightColor0, glm::value_ptr(color0), SHADER_UNIFORM_VEC3);
        if (m_locLightPos1 != -1)   SetShaderValue(m_lightShader, m_locLightPos1,   glm::value_ptr(pos1),  SHADER_UNIFORM_VEC3);
        if (m_locLightColor1 != -1) SetShaderValue(m_lightShader, m_locLightColor1, glm::value_ptr(color1), SHADER_UNIFORM_VEC3);
        if (m_locSmooth != -1)    { const int smoothVal = m_smoothShading ? 1 : 0; SetShaderValueV(m_lightShader, m_locSmooth, &smoothVal, SHADER_UNIFORM_INT, 1); }
        const float roughAmp = m_roughTexture ? m_roughness : 0.0f;
        if (m_locRough != -1)     SetShaderValue(m_lightShader, m_locRough, &roughAmp, SHADER_UNIFORM_FLOAT);
        if (m_locRoughFreq != -1) SetShaderValue(m_lightShader, m_locRoughFreq, &m_roughFreq, SHADER_UNIFORM_FLOAT);

        DrawMesh(rm.handle, *drawMat, MatrixIdentity());
    } else {
        DrawMesh(rm.handle, *drawMat, MatrixIdentity());
    }
}

void SceneRenderer::drawWireframePass(RenderMesh& rm, const std::vector<glm::vec3>& positions, const std::vector<Triangle>& triangles) {
    Color wf = Fade(lineColor(), 0.5f); // Drahtgitter gedaempft, damit es die Flaeche nicht ueberstrahlt
    // Depth-Test bleibt AKTIV: So verdeckt die gefuellte Vorderseite
    // Drahtkanten der Rueckseite (vorher rlDisableDepthTest -> die
    // gesamte Rueckseite schien durch den Koerper hindurch).
    // Drahtlinien minimal entlang der Vertex-Normalen nach aussen schieben,
    // damit sie auf der Vorderseite nicht mit der Flaeche z-fighten.
    const float wireOffset = 0.001f;

    m_scratchSegments.clear();
    m_scratchSegments.reserve(triangles.size() * 3 / 2 + 1);
    for (const auto& t : triangles) {
        auto edge = [&](size_t i, size_t j) {
            if (i > j) return;
            EdgeLineRenderer::Segment s;
            s.a = positions[i] + glm::vec3(rm.normals[i * 3 + 0], rm.normals[i * 3 + 1], rm.normals[i * 3 + 2]) * wireOffset;
            s.b = positions[j] + glm::vec3(rm.normals[j * 3 + 0], rm.normals[j * 3 + 1], rm.normals[j * 3 + 2]) * wireOffset;
            s.color = wf;
            m_scratchSegments.push_back(s);
        };
        edge(t.i0, t.i1);
        edge(t.i1, t.i2);
        edge(t.i2, t.i0);
    }

    // Dezent bleiben: schmale Kernlinie, schmaler Halo.
    m_edgeLines.draw(m_scratchSegments.data(), m_scratchSegments.size(), 1.6f, 1.0f, backgroundColor());
}

void SceneRenderer::syncVoronoiDual(const VoronoiDual& dual, int topologyRevision) {
    const auto& verts = dual.fillVertices();
    const auto& faces = dual.faceTriangles();
    if (verts.empty() || faces.empty()) return;

    // Zellflaechen wie die Triangulation als belichteten Mesh-Pass zeichnen
    // (Flat-Shading-Shader, geregelt ueber das globale Beleuchtungs-Setup).
    int count = static_cast<int>(faces.size());
    if (!m_dualMesh.uploaded || m_dualMesh.vertexCount != static_cast<int>(verts.size()) ||
        m_dualMesh.triangleCount != count || m_dualMesh.topologyRevision != topologyRevision) {
        rebuildMesh(m_dualMesh, verts, faces);
        m_dualMesh.topologyRevision = topologyRevision;
    } else {
        updateMeshVertices(m_dualMesh, verts);
    }
    ensureMaterial();
}

void SceneRenderer::drawDualFill(const VoronoiDual& dual) {
    const auto& verts = dual.fillVertices();
    if (verts.empty()) return;
    drawFillPass(m_dualMesh, verts);
}

void SceneRenderer::drawDualWireframe(const VoronoiDual& dual) {
    const auto& verts = dual.vertices();
    const auto& norms = dual.vertexNormals();
    const auto& edges = dual.edges();
    if (verts.empty() || edges.empty()) return;

    // Zellgrenzen des Zentroid-Duals: die eigentlichen Dual-Kanten (keine
    // Fan-Speichen), entlang der jeweiligen Face-Normale leicht angehoben,
    // sonst z-fighten/verdecken sie mit der gefuellten Oberflaeche.
    // Lila Akzentfarbe (abgesetzt vom neutralen Triangulations-Wireframe),
    // ebenfalls gedaempft wie die anderen Drahtgitter-Linien.
    Color c = (m_theme == SystemTheme::Theme::Dark)
        ? Color{ 188, 130, 255, 127 }
        : Color{ 84, 32, 150, 127};
    const float lift = 0.002f;

    m_scratchSegments.clear();
    m_scratchSegments.reserve(edges.size());
    for (const auto& e : edges) {
        if (e.first >= verts.size() || e.second >= verts.size()) continue;
        const glm::vec3 n0 = e.first < norms.size() ? norms[e.first] : glm::vec3(0.0f);
        const glm::vec3 n1 = e.second < norms.size() ? norms[e.second] : glm::vec3(0.0f);
        EdgeLineRenderer::Segment s;
        s.a = verts[e.first] + n0 * lift;
        s.b = verts[e.second] + n1 * lift;
        s.color = c;
        m_scratchSegments.push_back(s);
    }
    // Herausstechend: breiterer Halo, klar lesbare Kernlinie.
    m_edgeLines.draw(m_scratchSegments.data(), m_scratchSegments.size(), 2.4f, 1.2f, backgroundColor());
}

int SceneRenderer::pickSelectedCell(const VoronoiDual& dual, const Ray& ray) const {
    const auto& verts = dual.fillVertices();
    const auto& tris = dual.faceTriangles();
    const auto& cells = dual.cells();
    if (verts.empty() || tris.empty() || cells.empty()) return -1;

    // Möller-Trumbore gegen jede Zell-Flaeche. Die Dreiecke sind in der
    // Fan-Reihenfolge von rebuildFaces angeordnet: pro Zelle liegen exakt
    // cell.corners.size() Dreiecke fortlaufend (in Zellen-Reihenfolge).
    glm::vec3 ro(ray.position.x, ray.position.y, ray.position.z);
    glm::vec3 rd(ray.direction.x, ray.direction.y, ray.direction.z);

    float bestT = std::numeric_limits<float>::max();
    int bestCell = -1;
    size_t triBase = 0;
    for (size_t ci = 0; ci < cells.size(); ++ci) {
        const auto& cell = cells[ci];
        const size_t n = cell.corners.size();
        for (size_t k = 0; k < n; ++k) {
            const Triangle& t = tris[triBase + k];
            const glm::vec3& p0 = verts[t.i0];
            const glm::vec3& p1 = verts[t.i1];
            const glm::vec3& p2 = verts[t.i2];

            glm::vec3 e1 = p1 - p0;
            glm::vec3 e2 = p2 - p0;
            glm::vec3 p = glm::cross(rd, e2);
            float det = glm::dot(e1, p);
            if (std::fabs(det) < 1e-9f) continue;
            float inv = 1.0f / det;
            glm::vec3 tv = ro - p0;
            float u = glm::dot(tv, p) * inv;
            if (u < 0.0f || u > 1.0f) continue;
            glm::vec3 q = glm::cross(tv, e1);
            float v = glm::dot(rd, q) * inv;
            if (v < 0.0f || u + v > 1.0f) continue;
            float tHit = glm::dot(e2, q) * inv;
            if (tHit >= 0.0f && tHit < bestT) {
                bestT = tHit;
                bestCell = static_cast<int>(ci);
            }
        }
        triBase += n;
    }
    return bestCell;
}

void SceneRenderer::drawSelectedCell(const VoronoiDual& dual, int cellIndex, int topologyRevision) {
    const auto& verts = dual.fillVertices();
    const auto& tris = dual.faceTriangles();
    const auto& cells = dual.cells();
    if (cellIndex < 0 || cellIndex >= static_cast<int>(cells.size()) ||
        verts.empty() || tris.empty()) return;

    // Zell-Mesh nur neu aufbauen, wenn sich Auswahl oder Topologie geaendert
    // haben (die Dual-Geometrie bleibt zwischen Rebuilds stabil).
    if (m_selectedCell != cellIndex || m_selectedTopology != topologyRevision) {
        m_selectedCell = cellIndex;
        m_selectedTopology = topologyRevision;

        // Fan-Dreiecke der Zelle: triBase markiert den Start in faceTriangles
        // (Fortlaufend je Zelle, siehe rebuildFaces). Erstes Dreieck traegt im
        // Zentral-Vertex (Partikel-Position im fillVertices-Pool).
        size_t triBase = 0;
        for (int ci = 0; ci < cellIndex; ++ci) triBase += cells[ci].corners.size();
        const size_t n = cells[cellIndex].corners.size();
        if (n < 3) return;
        const uint32_t centerIdx = tris[triBase].i0;

        // Kompakten Vertex-Pool (Zentrale + Ecken) und lokale Fan-Indizes bauen.
        std::vector<uint32_t> localSrc;
        std::vector<glm::vec3> localVerts;
        auto localOf = [&](uint32_t g) -> uint32_t {
            for (size_t i = 0; i < localSrc.size(); ++i)
                if (localSrc[i] == g) return static_cast<uint32_t>(i);
            localSrc.push_back(g);
            localVerts.push_back(verts[g]);
            return static_cast<uint32_t>(localVerts.size() - 1);
        };
        const uint32_t centerLocal = localOf(centerIdx);
        std::vector<Triangle> localTris;
        localTris.reserve(n);
        for (size_t k = 0; k < n; ++k) {
            const Triangle& t = tris[triBase + k];
            localTris.push_back(Triangle{ centerLocal, localOf(t.i1), localOf(t.i2) });
        }

        rebuildMesh(m_selectedMesh, localVerts, localTris);
        m_selectedPositions = localVerts;

        // Leicht entlang der Vertex-Normalen anheben, damit die Akzentflaeche
        // nicht mit der gefuellten Dual-Flaeche z-fightet.
        const float lift = 0.0015f;
        for (int i = 0; i < m_selectedMesh.vertexCount; ++i) {
            m_selectedMesh.vertices[i * 3 + 0] += m_selectedMesh.normals[i * 3 + 0] * lift;
            m_selectedMesh.vertices[i * 3 + 1] += m_selectedMesh.normals[i * 3 + 1] * lift;
            m_selectedMesh.vertices[i * 3 + 2] += m_selectedMesh.normals[i * 3 + 2] * lift;
            m_selectedPositions[i].x += m_selectedMesh.normals[i * 3 + 0] * lift;
            m_selectedPositions[i].y += m_selectedMesh.normals[i * 3 + 1] * lift;
            m_selectedPositions[i].z += m_selectedMesh.normals[i * 3 + 2] * lift;
        }
        rlUpdateVertexBuffer(m_selectedMesh.handle.vboId[0], m_selectedMesh.vertices.data(),
            m_selectedMesh.vertexCount * 3 * sizeof(float), 0);
    }

    if (m_selectedMesh.uploaded) {
        ensureMaterial();
        drawFillPass(m_selectedMesh, m_selectedPositions, &m_materialSelected);
    }
}

void SceneRenderer::drawPathPolyline(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& normals) {
    if (points.size() < 2) return;

    // Akzentfarbe fuer den Pfad: knalliges Gruen auf dunklem, kräftiges
    // Waldgruen auf hellem Hintergrund (abgesetzt von Auswahl-Orange).
    Color c = (m_theme == SystemTheme::Theme::Dark)
        ? Color{ 64, 255, 128, 127 }
        : Color{ 0, 130, 60, 127 };

    m_scratchSegments.clear();
    m_scratchSegments.reserve(points.size() - 1);
    const float lift = 0.004f;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        const glm::vec3 n0 = i < normals.size() ? normals[i] : glm::vec3(0.0f);
        const glm::vec3 n1 = (i + 1) < normals.size() ? normals[i + 1] : glm::vec3(0.0f);
        EdgeLineRenderer::Segment s;
        s.a = points[i] + n0 * lift;
        s.b = points[i + 1] + n1 * lift;
        s.color = c;
        m_scratchSegments.push_back(s);
    }
    // Kräftige, gut sichtbare Kernlinie mit weitem Halo.
    m_edgeLines.draw(m_scratchSegments.data(), m_scratchSegments.size(), 3.2f, 1.6f, backgroundColor());
}

void SceneRenderer::drawSpatialGrid(const ParticleSystem& system) {
    float cs = system.spatialHash().cellSize();
    Color dim = Fade(BLUE, 0.5f);
    for (const SpatialHash::CellKey& key : system.spatialHash().occupiedCells()) {
        glm::vec3 center{ (key.x + 0.5f) * cs, (key.y + 0.5f) * cs, (key.z + 0.5f) * cs };
        DrawCubeWires({ center.x, center.y, center.z }, cs, cs, cs, dim);
    }
}

void SceneRenderer::drawSDFProjections(const ParticleSystem& system) {
    Color proj = Fade({ 0, 228, 228, 255 }, 0.5f);
    rlBegin(RL_LINES);
    for (const auto& p : system.particles) {
        glm::vec3 diff = p.position - p.projectionFrom;
        if (glm::dot(diff, diff) < 1e-10f) continue;
        rlColor4ub(proj.r, proj.g, proj.b, proj.a);
        rlVertex3f(p.projectionFrom.x, p.projectionFrom.y, p.projectionFrom.z);
        rlVertex3f(p.position.x, p.position.y, p.position.z);
    }
    rlEnd();
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