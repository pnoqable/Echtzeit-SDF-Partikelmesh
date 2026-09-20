#pragma once

#include "Triangulation.hpp"
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

// Zentroid-Dual (diskrete Voronoi-Tesselation) der Oberflaechentriangulation.
//
// Zu jedem primalen Dreieck wird ein dualer Vertex an dessen Schwerpunkt
// erzeugt (ein Vertex je Dreieck, Reihenfolge = Dreiecksindex). Um jeden
// Oberflaechenpartikel wird die Zelle als Ring der Dualen Vertices seiner
// inzidenten Dreiecke gebildet (Sortierung nach Winkel in der Tangentebene
// der Partikel-Normalen). Jede primale Gitterkante (u,v) ist die Grenze
// zwischen genau zwei Dreiecken -> die verbindende Dual-Kante gehoert zu den
// Zellen von u und v. Ergebnis: genaue Zelle je Partikel, die Oberflaeche
// wird partitioniert.
//
// Build ist seriell und nur bei Topologie-Aenderung noetig (einmalig pro
// Triangulation, nicht pro Frame).
class VoronoiDual {
public:
    struct Cell {
        uint32_t particle;                 // Partikel-Index der Zelle
        std::vector<uint32_t> corners;     // duale Vertex-Indizes (= Dreiecks-IDs), Ring
    };

    void build(
        const std::vector<glm::vec3>& positions,
        const std::vector<glm::vec3>& normals,
        const std::vector<Triangle>& triangles
    );
    void clear() { m_vertices.clear(); m_cells.clear(); m_edges.clear(); m_vertexNormals.clear(); m_faceTriangles.clear(); m_fillVertices.clear(); }

    // Duale Vertex-Positionen: eine je primales Dreieck (an dessen Schwerpunkt).
    const std::vector<glm::vec3>& vertices() const { return m_vertices; }
    // Normale je Dual-Vertex = orientierte Face-Normale des Dreiecks (nach
    // aussen fuer geschlossene, orientierte Meshes). Dient dem Renderer, die
    // Zellgrenzkanten leicht ueber die gefuellte Oberflaeche anzuheben.
    const std::vector<glm::vec3>& vertexNormals() const { return m_vertexNormals; }
    const std::vector<Cell>& cells() const { return m_cells; }

    // Eindeutige Zellgrenzkanten (Paar von Dual-Vertex-Indizes), je einmal.
    const std::vector<std::pair<uint32_t, uint32_t>>& edges() const { return m_edges; }

// Flaechentriangulierung des Duals fuer gerenderte Zellen: jede Zelle wird
// als Triangle-Fan um die urspruengliche Partikel-Position aufgespannt, die
// als zentraler Extra-Vertex im Vertex-Pool der Zellflaeche liegt. Dadurch
// bildet das Zell-Mesh Kruemmungen der SDF-Oberflaeche ab (statt nur der
// flachen Dual-Eckpunkt-Scheiben). Die Indizes beziehen sich auf
// fillVertices(), nicht auf vertices().
const std::vector<Triangle>& faceTriangles() const { return m_faceTriangles; }

// Vertex-Pool der gefuellten Zell-Meshes: zuerst alle Dual-Vertex (Zentroide),
// dann je Zelle die Partikel-Position als Fan-Zentralvertex.
const std::vector<glm::vec3>& fillVertices() const { return m_fillVertices; }

    void rebuildEdges();
    void rebuildFaces(const std::vector<glm::vec3>& positions);

private:
    std::vector<glm::vec3> m_vertices;
    std::vector<glm::vec3> m_vertexNormals;
    std::vector<Cell> m_cells;
    std::vector<std::pair<uint32_t, uint32_t>> m_edges;
    std::vector<Triangle> m_faceTriangles;
    std::vector<glm::vec3> m_fillVertices;
};