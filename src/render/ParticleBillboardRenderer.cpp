#include "ParticleBillboardRenderer.hpp"
#include <raymath.h>
#include <rlgl.h>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <array>
#include <cmath>

namespace {

// Vertex-Shader: spannnt das Achteck in der Bildebene (View-Space) auf, dadurch
// sind die Boards kamerafest. Das Partikelzentrum wird vorher entlang der
// uebergebenen Oberflaechennormale angehoben (gegen Z-Buffer-Overdraw des
// darunter liegenden Meshes). Die Attribute 1..3 sind Instanz-Attribute
// (Divisor 1), werden also pro Partikel einmal geliefert.
const char* kBillboardVS = R"GLSL(
#version 330

layout(location = 0) in vec2 aCorner;   // Achteck-Ecke (Einheits-Vertikalabstand)
layout(location = 1) in vec3 aPosition; // Partikelposition (Instanz)
layout(location = 2) in vec3 aNormal;   // Oberflaechennormale (Instanz)
layout(location = 3) in vec4 aColor;    // Partikelfarbe (Instanz, ubyte norm.)

uniform mat4 uProjection;
uniform mat4 uModelView;
uniform float uLift;
uniform float uSize;

out vec4 vColor;

void main() {
    // Zentrum entlang der Normalen heben: Das Board liegt damit voll auf der
    // Oberflaeche statt im steilen Winkel im Mesh zu versinken.
    vec3 center = aPosition + aNormal * uLift;
    vec4 viewPos = uModelView * vec4(center, 1.0);
    // Achteck in der Bildebene: View-Space +X = rechts, +Y = oben.
    viewPos.xy += aCorner * uSize;
    gl_Position = uProjection * viewPos;
    vColor = aColor;
}
)GLSL";

// Fragment-Shader: Farbe durchreichen. Die Partikel sind (anders als das Mesh)
// unbeleuchtete Akzentfarbe, daher kein Flat-Shading/Gamma wie im Licht-Shader.
const char* kBillboardFS = R"GLSL(
#version 330

in vec4 vColor;
out vec4 finalColor;

void main() {
    finalColor = vColor;
}
)GLSL";

constexpr size_t kCornerCount = 8;
constexpr size_t kVertexCount = kCornerCount + 1;   // Zentrum + 8 Ecken
constexpr size_t kIndexCount  = kCornerCount * 3;   // 8 Dreiecke

} // namespace

ParticleBillboardRenderer::~ParticleBillboardRenderer() {
    unload();
}

void ParticleBillboardRenderer::unload() {
    if (m_vboIndex != 0)     rlUnloadVertexBuffer(m_vboIndex);
    if (m_vboCorner != 0)    rlUnloadVertexBuffer(m_vboCorner);
    if (m_vboInstance != 0)  rlUnloadVertexBuffer(m_vboInstance);
    if (m_vao != 0)          rlUnloadVertexArray(m_vao);
    if (m_shader.id != 0)    UnloadShader(m_shader);
    m_vboIndex = m_vboCorner = m_vboInstance = 0;
    m_vao = 0;
    m_shader = {};
    m_instanceCapacity = 0;
    m_ready = false;
    m_scratch.clear();
}

void ParticleBillboardRenderer::ensureResources() {
    if (m_ready) return;

    m_shader = LoadShaderFromMemory(kBillboardVS, kBillboardFS);
    if (m_shader.id == 0) return;
    m_locModelView  = GetShaderLocation(m_shader, "uModelView");
    m_locProjection = GetShaderLocation(m_shader, "uProjection");
    m_locLift       = GetShaderLocation(m_shader, "uLift");
    m_locSize       = GetShaderLocation(m_shader, "uSize");
    if (m_locModelView == -1 || m_locProjection == -1 ||
        m_locLift == -1 || m_locSize == -1) {
        // Shader abgelehnt (z.B. Attribute geandert) -> GL-Kontext geht kaputt.
        UnloadShader(m_shader);
        m_shader = {};
        return;
    }

    // Achteck: Zentrum (Index 0) + 8 Ecken auf dem Einheitskreis, als 8
    // Dreiecke (Zentrum -> c[i] -> c[i+1]) indexiert.
    std::array<float, kVertexCount * 2> corners = {};
    corners[0] = 0.0f;
    corners[1] = 0.0f;
    const float twoPi = 2.0f * glm::pi<float>();
    for (size_t i = 0; i < kCornerCount; ++i) {
        float a = twoPi * static_cast<float>(i) / static_cast<float>(kCornerCount);
        corners[(i + 1) * 2 + 0] = std::cos(a);
        corners[(i + 1) * 2 + 1] = std::sin(a);
    }

    std::array<unsigned short, kIndexCount> indices = {};
    for (size_t i = 0; i < kCornerCount; ++i) {
        indices[i * 3 + 0] = 0;
        indices[i * 3 + 1] = static_cast<unsigned short>(i + 1);
        indices[i * 3 + 2] = static_cast<unsigned short>((i + 1) % kCornerCount + 1);
    }

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

void ParticleBillboardRenderer::configureAttributes() {
    rlEnableVertexArray(m_vao);

    // Per-Vertex: Achteck-Ecke (Location 0).
    rlEnableVertexBuffer(m_vboCorner);
    rlSetVertexAttribute(0, 2, RL_FLOAT, false, 0, 0);
    rlEnableVertexAttribute(0);

    // Indexbuffer.
    rlEnableVertexBufferElement(m_vboIndex);

    // Per-Instanz: Position (1), Normale (2), Farbe (3) — als Divisor-1-Attribute
    // aus einem interleaved, dynamischen Puffer.
    if (m_vboInstance != 0) {
        rlEnableVertexBuffer(m_vboInstance);
        const int stride = static_cast<int>(sizeof(PackedInstance));
        rlSetVertexAttribute(1, 3, RL_FLOAT, false, stride, offsetof(PackedInstance, px));
        rlSetVertexAttributeDivisor(1, 1);
        rlEnableVertexAttribute(1);
        rlSetVertexAttribute(2, 3, RL_FLOAT, false, stride, offsetof(PackedInstance, nx));
        rlSetVertexAttributeDivisor(2, 1);
        rlEnableVertexAttribute(2);
        rlSetVertexAttribute(3, 4, RL_UNSIGNED_BYTE, true, stride, offsetof(PackedInstance, r));
        rlSetVertexAttributeDivisor(3, 1);
        rlEnableVertexAttribute(3);
    }

    rlDisableVertexArray();
}

void ParticleBillboardRenderer::uploadInstances(const Instance* instances, size_t count) {
    m_scratch.resize(count);
    for (size_t i = 0; i < count; ++i) {
        PackedInstance& s = m_scratch[i];
        const Instance& in = instances[i];
        s.px = in.position.x; s.py = in.position.y; s.pz = in.position.z;
        s.nx = in.normal.x;  s.ny = in.normal.y;  s.nz = in.normal.z;
        s.r = in.color.r;    s.g = in.color.g;    s.b = in.color.b; s.a = in.color.a;
    }

    if (count > m_instanceCapacity) {
        // Kapazitaet in 2er-Potenzen halten, um haeufiges Neuallokieren zu vermeiden.
        if (m_vboInstance != 0) rlUnloadVertexBuffer(m_vboInstance);
        m_instanceCapacity = 16;
        while (m_instanceCapacity < count) m_instanceCapacity *= 2;
        m_vboInstance = rlLoadVertexBuffer(nullptr,
            static_cast<int>(m_instanceCapacity * sizeof(PackedInstance)), true);
        configureAttributes();  // Attribut-Pointer zeigen auf den neuen Puffer
    }

    rlUpdateVertexBuffer(m_vboInstance, m_scratch.data(),
        static_cast<int>(count * sizeof(PackedInstance)), 0);
}

void ParticleBillboardRenderer::draw(const Instance* instances, size_t count) {
    if (instances == nullptr || count == 0) return;
    ensureResources();
    if (!m_ready) return;

    uploadInstances(instances, count);

    // Vor dem Instanced-Draw evtl. angesammelte Immediate-Geometrie flushen,
    // damit unser Draw nicht von einem noch offenen Batch ueberdeckt wird.
    rlDrawRenderBatchActive();

    rlEnableShader(m_shader.id);
    rlSetUniformMatrix(m_locModelView, rlGetMatrixModelview());
    rlSetUniformMatrix(m_locProjection, rlGetMatrixProjection());
    rlSetUniform(m_locLift, &m_lift, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(m_locSize, &m_size, RL_SHADER_UNIFORM_FLOAT, 1);

    // Billboards sind doppelseitige Flächen: Backface-Culling wuerde die
    // Rueckseite ausblenden, dadurch schimmerte die Oberflaeche durch.
    rlDisableBackfaceCulling();
    rlEnableVertexArray(m_vao);
    rlDrawVertexArrayElementsInstanced(0, static_cast<int>(kIndexCount), nullptr, static_cast<int>(count));
    rlDisableVertexArray();
    rlEnableBackfaceCulling();
}