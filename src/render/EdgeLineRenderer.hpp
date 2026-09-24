#pragma once

#include <glm/glm.hpp>
#include <raylib.h>
#include <cstddef>
#include <cstdint>
#include <vector>

// Shaderbasierter Renderer fuer Kanten-/Drahtgitter-Linien.
//
// Alt: glLineWidth (auf macOS Core-Profile auf 1px gedeckelt). Neu: Jedes
// Segment wird als Quad im Screen-Space aufgespannt (Geometrie: 4 Ecken mit
// a/b-Endpunkten + Corner-Info, als 2 Dreiecke). Der Vertex-Shader projiziert
// beide Endpunkte, berechnet die Senkrechte der Kante in Bildschirmpixeln und
// verschiebt die Ecken um uHaloHalfPx — die Dicke ist dadurch in Pixeln konstant
// und unabhaengig von Zoom/Abstand.
//
// Kontrast-Halo: Der Fragment-Shader zeichnet die schmale helle Kernlinie
// (Segmentfarbe) auf einer dunklen Unterlage (Halo-Farbe). So heben sich die
// Kanten sowohl von dunklen (Halo unsichtbar, Kern hell) als auch von hellen
// Flaechen (dunkler Umriss) ab.
//
// Instancing (Billboard-Pattern): Die Quad-Geometrie (4 Ecken, 6 Indizes)
// liegt einmalig statisch auf der GPU. Pro Kante werden nur noch die
// Segmentdaten (Endpunkte a/b + Farbe = 28 Bytes) als Instanz-Attribute
// uebertragen. Ein einziger glDrawElementsInstanced()-Aufruf zeichnet alle
// Kanten — gleiches Muster wie die Partikel-Billboards.
class EdgeLineRenderer {
public:
    struct Segment {
        glm::vec3 a;
        glm::vec3 b;
        Color color;
    };

    EdgeLineRenderer() = default;
    ~EdgeLineRenderer();
    EdgeLineRenderer(const EdgeLineRenderer&) = delete;
    EdgeLineRenderer& operator=(const EdgeLineRenderer&) = delete;

    // Zeichnet count Segmente als Screen-Space-Quads. Erwartet einen offenen
    // 3D-Render-Kontext (BeginMode3D), dessen Model-View- und Projektionsmatrix
    // uebernommen werden. haloHalfPx = halbe Gesamtbreite des Halos,
    // coreHalfPx = halbe Breite der hellen Kernlinie (<= haloHalfPx).
    void draw(const Segment* segments, size_t count,
              float haloHalfPx, float coreHalfPx, Color haloColor);

    void unload();

private:
    // Instanzdaten je Kante: Endpunkt a, Endpunkt b, Farbe (ubyte norm.).
    // Stride 28 (alle Offsets 4er-ausgerichtet).
    struct PackedInstance {
        float ax, ay, az;
        float bx, by, bz;
        unsigned char r, g, b, a;
    };
    static_assert(sizeof(PackedInstance) == 28, "PackedInstance expected tightly packed");

    void ensureResources();
    void configureAttributes();
    void uploadInstances(const Segment* segments, size_t count);

    bool m_ready = false;

    ::Shader m_shader = {};
    // Locations werden ueber rlSetUniform/rlSetUniformMatrix gesetzt (wie im
    // ParticleBillboardRenderer), nicht ueber die raylib-Shader-Pflege.
    int m_locModelView  = -1;
    int m_locProjection = -1;
    int m_locScreenSize = -1;
    int m_locHaloHalfPx = -1;
    int m_locCoreHalfPx = -1;
    int m_locHaloColor  = -1;

    unsigned int m_vao = 0;
    unsigned int m_vboCorner = 0;    // Quad-Ecken je Vertex (vec2, statisch)
    unsigned int m_vboIndex = 0;     // Indexbuffer (2 Dreiecke, unsigned short)
    unsigned int m_vboInstance = 0;  // Instanzdaten (dynamisch)
    size_t m_instanceCapacity = 0;

    std::vector<PackedInstance> m_scratch;
    std::vector<float> m_haloColorScratch;
};