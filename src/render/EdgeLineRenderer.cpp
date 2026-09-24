#include "EdgeLineRenderer.hpp"
#include <raymath.h>
#include <rlgl.h>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <array>
#include <cmath>

namespace {

// Vertex-Shader: spannt jedes Segment als Quad in der Bildebene auf. Die Quad-
// Geometrie (4 statische Ecken) wird per Instancing fuer jede Kante wiederholt.
// Pro Ecke liegen die Corner-Info an; die beiden Welt-Endpunkte (a/b) plus
// Farbe kommen als Instanz-Attribute. corner.x waehlt den Punkt entlang der
// Kante (0 = a, 1 = b), corner.y die Seite quer zur Kante (-1/+1). Beide
// Endpunkte werden projiziert, daraus die Screen-Space-Senkrechte berechnet und
// die Ecke um uHaloHalfPx Pixel quer zur Kante verschoben — die Dicke ist damit
// pixelkonstant, nicht abhaengig von Zoom oder glLineWidth.
const char* kEdgeVS = R"GLSL(
#version 330

layout(location = 0) in vec2 aCorner;   // Quad-Ecke (0..1 entlang, -1..+1 quer)
layout(location = 1) in vec3 aA;        // Endpunkt A (Welt) — Instanz
layout(location = 2) in vec3 aB;        // Endpunkt B (Welt) — Instanz
layout(location = 3) in vec4 aColor;    // Segmentfarbe (ubyte norm.) — Instanz

uniform mat4 uProjection;
uniform mat4 uModelView;
uniform vec2 uScreenSize;   // Bildschirmgroesse in Pixeln
uniform float uHaloHalfPx;  // halbe Halo-Breite in Pixeln

out vec4 vColor;
out float vCornerY;         // -1..+1 quer zur Kante (fuer den Kernel im FS)

void main() {
    vec4 clipA = uProjection * uModelView * vec4(aA, 1.0);
    vec4 clipB = uProjection * uModelView * vec4(aB, 1.0);
    vec2 ndcA = clipA.xy / clipA.w;
    vec2 ndcB = clipB.xy / clipB.w;

    // Pixel-Koordinaten (0..uscreen) der beiden Endpunkte.
    vec2 pxA = (ndcA * 0.5 + 0.5) * uScreenSize;
    vec2 pxB = (ndcB * 0.5 + 0.5) * uScreenSize;

    // Screen-Space-Senkrechte: +90° zur projizierten Kantenrichtung.
    vec2 dir = pxB - pxA;
    vec2 perp = dir.y * dir.y + dir.x * dir.x > 1e-12
        ? vec2(-dir.y, dir.x) / max(length(dir), 1e-6)
        : vec2(1.0, 0.0);

    // Eckpunkt: entlang der Kante interpolieren, dann quer verschieben.
    // Die Verschiebung wird in Pixel gerechnet und erst am Ende zurueck in NDC
    // gemappt, damit die Dicke bildschirmkonstant (und nicht NDC-verzerrt) ist.
    vec2 px = mix(pxA, pxB, aCorner.x) + perp * (aCorner.y * uHaloHalfPx);
    vec2 ndc = px * 2.0 / uScreenSize - 1.0;

    // Tiefe von der interpolierten Clip-Position uebernehmen, nur xy ersetzen:
    // so liegt die Kante auf der korrekten Tiefe (kein z-Fight, kein falsches
    // Ueberblenden mit der darunter liegenden Flaeche). w = 1 => NDC bereits
    // nach der perspektivischen Division.
    vec4 clip = mix(clipA, clipB, aCorner.x);
    gl_Position = vec4(ndc.x, ndc.y, clip.z / clip.w, 1.0);

    vColor = aColor;
    vCornerY = aCorner.y;
}
)GLSL";

// Fragment-Shader: Kernlinie + Halo. Der Abstand zur Kantenmitte (vCornerY mal
// uHaloHalfPx in Pixeln) entscheidet, ob der Pixl zur hellen Kernlinie gehoert
// (Segmentfarbe) oder zur dunklen Unterlage (Halo-Farbe). An den Grenzen wird
// mit smoothstep weich ausgeblendet, damit die Linienkanten nicht zackig sind.
const char* kEdgeFS = R"GLSL(
#version 330

in vec4 vColor;
in float vCornerY;

uniform float uHaloHalfPx;  // halbe Halo-Breite in Pixeln
uniform float uCoreHalfPx;  // halbe Kernbreite in Pixeln
uniform vec4 uHaloColor;    // Farbe der dunklen Unterlage

out vec4 finalColor;

void main() {
    if (abs(vCornerY) > 1.0) discard;  // Sicherheitsnetz (sollte nie passieren)

    // Abstand von der Kantenmitte in Pixeln.
    float d = abs(vCornerY) * uHaloHalfPx;

    // Uebergang Kernlinie -> Unterlage weich (knapp einen Pixel breit).
    float coreMix = smoothstep(max(uCoreHalfPx - 0.75, 0.0), uCoreHalfPx + 0.75, d);
    vec3 col = mix(vColor.rgb, uHaloColor.rgb, coreMix);

    // Aussenkante des Halos weich ausblenden.
    float edgeMix = smoothstep(uHaloHalfPx - 0.75, uHaloHalfPx, d);
    float alpha = (1.0 - edgeMix);

    finalColor = vec4(col, alpha * vColor.a);
}
)GLSL";

constexpr size_t kCornerCount = 4;
constexpr size_t kIndexCount = 6;   // 2 Dreiecke

} // namespace

EdgeLineRenderer::~EdgeLineRenderer() {
    unload();
}

void EdgeLineRenderer::unload() {
    if (m_vboInstance != 0) rlUnloadVertexBuffer(m_vboInstance);
    if (m_vboIndex != 0)    rlUnloadVertexBuffer(m_vboIndex);
    if (m_vboCorner != 0)   rlUnloadVertexBuffer(m_vboCorner);
    if (m_vao != 0)         rlUnloadVertexArray(m_vao);
    if (m_shader.id != 0)   UnloadShader(m_shader);
    m_vboCorner = m_vboIndex = m_vboInstance = 0;
    m_vao = 0;
    m_shader = {};
    m_instanceCapacity = 0;
    m_ready = false;
    m_scratch.clear();
    m_haloColorScratch.clear();
}

void EdgeLineRenderer::ensureResources() {
    if (m_ready) return;

    m_shader = LoadShaderFromMemory(kEdgeVS, kEdgeFS);
    if (m_shader.id == 0) {
        return;
    }
    m_locModelView  = GetShaderLocation(m_shader, "uModelView");
    m_locProjection = GetShaderLocation(m_shader, "uProjection");
    m_locScreenSize = GetShaderLocation(m_shader, "uScreenSize");
    m_locHaloHalfPx = GetShaderLocation(m_shader, "uHaloHalfPx");
    m_locCoreHalfPx = GetShaderLocation(m_shader, "uCoreHalfPx");
    m_locHaloColor  = GetShaderLocation(m_shader, "uHaloColor");
    if (m_locModelView == -1 || m_locProjection == -1 || m_locScreenSize == -1 ||
        m_locHaloHalfPx == -1 || m_locCoreHalfPx == -1 || m_locHaloColor == -1) {
        UnloadShader(m_shader);
        m_shader = {};
        return;
    }

    // Statisches Quad: 4 Ecken als Corner-Info (0..1 entlang, -1..+1 quer),
    // als zwei Dreiecke indexiert. Diese Geometrie liegt einmalig auf der GPU
    // und wird per Instancing fuer jede Kante wiederholt.
    std::array<float, kCornerCount * 2> corners = {
        { 0.0f, -1.0f,   // a, unten
          0.0f,  1.0f,   // a, oben
          1.0f, -1.0f,   // b, unten
          1.0f,  1.0f }  // b, oben
    };
    std::array<unsigned short, kIndexCount> indices = {
        0, 1, 2, 1, 3, 2
    };

    m_vboCorner = rlLoadVertexBuffer(corners.data(), static_cast<int>(corners.size() * sizeof(float)), false);
    m_vboIndex  = rlLoadVertexBufferElement(indices.data(), static_cast<int>(indices.size() * sizeof(unsigned short)), false);
    m_vao       = rlLoadVertexArray();
    if (m_vboCorner == 0 || m_vboIndex == 0 || m_vao == 0) {
        unload();
        return;
    }

    // Instanz-VBO wird beim ersten uploadInstances() angelegt (Capacity-Policy).
    configureAttributes();
    m_ready = true;
}

void EdgeLineRenderer::configureAttributes() {
    rlEnableVertexArray(m_vao);

    // Per-Vertex: Quad-Ecke (Location 0).
    rlEnableVertexBuffer(m_vboCorner);
    rlSetVertexAttribute(0, 2, RL_FLOAT, false, 0, 0);
    rlEnableVertexAttribute(0);

    // Indexbuffer.
    rlEnableVertexBufferElement(m_vboIndex);

    // Per-Instanz: Endpunkt A (1), Endpunkt B (2), Farbe (3) — als Divisor-1-
    // Attribute aus einem interleaved, dynamischen Puffer.
    if (m_vboInstance != 0) {
        rlEnableVertexBuffer(m_vboInstance);
        const int stride = static_cast<int>(sizeof(PackedInstance));
        rlSetVertexAttribute(1, 3, RL_FLOAT, false, stride, offsetof(PackedInstance, ax));
        rlSetVertexAttributeDivisor(1, 1);
        rlEnableVertexAttribute(1);
        rlSetVertexAttribute(2, 3, RL_FLOAT, false, stride, offsetof(PackedInstance, bx));
        rlSetVertexAttributeDivisor(2, 1);
        rlEnableVertexAttribute(2);
        rlSetVertexAttribute(3, 4, RL_UNSIGNED_BYTE, true, stride, offsetof(PackedInstance, r));
        rlSetVertexAttributeDivisor(3, 1);
        rlEnableVertexAttribute(3);
    }

    rlDisableVertexArray();
}

void EdgeLineRenderer::uploadInstances(const Segment* segments, size_t count) {
    m_scratch.resize(count);
    for (size_t i = 0; i < count; ++i) {
        PackedInstance& s = m_scratch[i];
        const Segment& in = segments[i];
        s.ax = in.a.x; s.ay = in.a.y; s.az = in.a.z;
        s.bx = in.b.x; s.by = in.b.y; s.bz = in.b.z;
        s.r = in.color.r; s.g = in.color.g;
        s.b = in.color.b; s.a = in.color.a;
    }

    if (count > m_instanceCapacity) {
        // Kapazitaet in 2er-Potenzen halten, um haeufiges Neuallokieren (VBO-
        // Neuaufbau + Attribut-Konfiguration) zu vermeiden.
        if (m_vboInstance != 0) rlUnloadVertexBuffer(m_vboInstance);
        m_instanceCapacity = 16;
        while (m_instanceCapacity < count) m_instanceCapacity *= 2;
        m_vboInstance = rlLoadVertexBuffer(nullptr,
            static_cast<int>(m_instanceCapacity * sizeof(PackedInstance)), true);
        if (m_vboInstance == 0) {
            return;
        }
        configureAttributes();  // Attribut-Pointer zeigen auf den neuen Puffer
    }

    rlUpdateVertexBuffer(m_vboInstance, m_scratch.data(),
        static_cast<int>(count * sizeof(PackedInstance)), 0);
}

void EdgeLineRenderer::draw(const Segment* segments, size_t count,
                            float haloHalfPx, float coreHalfPx, Color haloColor) {
    if (segments == nullptr || count == 0) {
        return;
    }
    ensureResources();
    if (!m_ready) {
        return;
    }

    uploadInstances(segments, count);

    // Vor dem Instanced-Draw evtl. angesammelte Immediate-Geometrie flushen,
    // damit unser Draw nicht von einem noch offenen Batch ueberdeckt wird.
    rlDrawRenderBatchActive();

    rlEnableShader(m_shader.id);
    rlSetUniformMatrix(m_locModelView, rlGetMatrixModelview());
    rlSetUniformMatrix(m_locProjection, rlGetMatrixProjection());
    const float screenSize[2] = { static_cast<float>(GetScreenWidth()), static_cast<float>(GetScreenHeight()) };
    rlSetUniform(m_locScreenSize, screenSize, RL_SHADER_UNIFORM_VEC2, 1);
    rlSetUniform(m_locHaloHalfPx, &haloHalfPx, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(m_locCoreHalfPx, &coreHalfPx, RL_SHADER_UNIFORM_FLOAT, 1);
    const float haloC[4] = { haloColor.r / 255.0f, haloColor.g / 255.0f, haloColor.b / 255.0f, haloColor.a / 255.0f };
    rlSetUniform(m_locHaloColor, haloC, RL_SHADER_UNIFORM_VEC4, 1);

    rlDisableBackfaceCulling();
    rlEnableVertexArray(m_vao);

    // Ein Instanced-Draw: Das statische Quad wird count-mal gezeichnet, jede
    // Instanz bekommt ihre Endpunkte/Farbe aus dem Instanz-VBO. Kein grosser
    // Index-Buffer noetig, daher keine 16-bit-Grenze.
    rlDrawVertexArrayElementsInstanced(0, static_cast<int>(kIndexCount), nullptr, static_cast<int>(count));;;

    rlDisableVertexArray();
    rlEnableBackfaceCulling();
}