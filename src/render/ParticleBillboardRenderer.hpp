#pragma once

#include <glm/glm.hpp>
#include <raylib.h>
#include <cstddef>
#include <cstdint>
#include <vector>

// Renderer fuer kamerafeste Partikel-Billboards.
//
// Jedes Partikel wird als regulaeres Achteck gezeichnet, das im Vertex-Shader
// im View-Raum aufgespannt wird (Offset in der Bildebene): Dadurch bleibt das
// Board unabhaengig von der Kameraorientierung immer frontal zur Kamera, ohne
// dass die CPU eine inverse Kameramatrix oder per-Partikel-Matrizen berechnen
// muss. Pro Instanz werden nur die Partikelgeometriedaten (Position, Normale)
// plus Farbe als Instanz-Attribute uebergeben; pro Frame laufen nur die globale
// Model-View- und Projektionsmatrix als Uniform in den Shader.
//
// Gegen Z-Buffer-Overdraw durch den darunter liegenden Mesh wird das
// Partikelzentrum im Vertex-Shader entlang der uebergebenen Oberflaechennormale
// um setLift() nach aussen verschoben. Die Geometrie (ein Achteck aus 8
// Dreiecken) liegt einmalig als VAO/VBO auf der GPU; alle Partikel werden mit
// einem einzigen glDrawElementsInstanced()-Aufruf gezeichnet.
class ParticleBillboardRenderer {
public:
    static constexpr float kDefaultSize = 0.004f;   // Achteck-Halbkante (Bildschirm-Offset)
    static constexpr float kDefaultLift = 0.004f;   // Anhebung entlang der Normalen

    // Geometriedaten fuer ein Partikel (eine "Instanz").
    struct Instance {
        glm::vec3 position;   // Partikelposition auf der SDF-Oberflaeche
        glm::vec3 normal;     // Oberflaechennormale (fuer Anhebung + Orientierung)
        Color color;          // Partikelfarbe
    };

    ParticleBillboardRenderer() = default;
    ~ParticleBillboardRenderer();
    ParticleBillboardRenderer(const ParticleBillboardRenderer&) = delete;
    ParticleBillboardRenderer& operator=(const ParticleBillboardRenderer&) = delete;

    // Zeichnet count Instanzen als kamerafeste Achteck-Billboards. Erwartet
    // einen offenen 3D-Render-Kontext (BeginMode3D), dessen Model-View- und
    // Projektionsmatrix uebernommen werden.
    void draw(const Instance* instances, size_t count);

    void setLift(float lift) { m_lift = lift; }
    void setSize(float size) { m_size = size; }

    // Gibt Shader, VAO und VBOs frei. Benoetigt einen gueltigen GL-Kontext.
    void unload();

private:
    // GPU-seitig kopakte, gepackte Instanzdaten (Alignment 4). Die 4 Farbbytes
    // werden als GL_UNSIGNED_BYTE (normalisiert) gebunden; addrAlignment: Offset
    // 0/12/24 sind alle 4er-ausgerichtet, Stride = 28.
    struct PackedInstance {
        float px, py, pz;
        float nx, ny, nz;
        unsigned char r, g, b, a;
    };
    static_assert(sizeof(PackedInstance) == 28, "PackedInstance expected tightly packed");

    void ensureResources();
    void configureAttributes();
    void uploadInstances(const Instance* instances, size_t count);

    bool m_ready = false;
    float m_lift = kDefaultLift;
    float m_size = kDefaultSize;

    ::Shader m_shader = {};
    int m_locModelView = -1;
    int m_locProjection = -1;
    int m_locLift = -1;
    int m_locSize = -1;

    unsigned int m_vao = 0;
    unsigned int m_vboCorner = 0;    // Oktaeder-Ecken je Vertex (vec2)
    unsigned int m_vboIndex = 0;     // Indexbuffer (8 Dreiecke, unsigned short)
    unsigned int m_vboInstance = 0;  // Instanzdaten (dynamisch)
    size_t m_instanceCapacity = 0;

    std::vector<PackedInstance> m_scratch;
};