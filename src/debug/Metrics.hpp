#pragma once

#include "../simulation/ParticleSystem.hpp"
#include "../simulation/SDF.hpp"
#include "../mesh/Triangulation.hpp"
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

namespace debug {

// Abstandsstatistik ueber die (via SpatialHash) naechsten Nachbarn.
struct DistributionMetrics {
    float minDist = 0.0f;
    float avgDist = 0.0f;
    float maxDist = 0.0f;
    float stdDev = 0.0f;
    int underCount = 0;      // d < 0.85 h
    int okCount = 0;         // 0.85 h <= d <= 1.15 h
    int overCount = 0;       // d > 1.15 h
};

struct SDFMetrics {
    float avgAbsPhi = 0.0f;
    float maxAbsPhi = 0.0f;
};

struct MeshQualityMetrics {
    int poorTriangles = 0;   // min. Innenwinkel < 20° od. Aspect-Ratio > 4
    float minAngleDeg = 180.0f;
    float maxAspectRatio = 1.0f;
};

struct SimulationMetrics {
    DistributionMetrics distribution;
    SDFMetrics sdf;
    MeshQualityMetrics mesh;
    float avgSpeed = 0.0f;
    float maxSpeed = 0.0f;
};

// Erwartet: system.buildSpatialHash() wurde vor dem Aufruf ausgefuehrt.
SimulationMetrics evaluate(const ParticleSystem& system, const SDF& sdf, float targetSpacing);

// Histogramm der naechsten Nachbarabstaende: bins zwischen 0 und maxDistRatio*h
// (letzter Bin faengt Werte >= kOverRatio h, aber unter maxDistRatio h ab).
// Fuer ImGui::PlotHistogram: normalize=false liefert Zaehler.
std::vector<float> spacingHistogram(const ParticleSystem& system, float targetSpacing, int bins, float maxDistRatio = 2.0f);

// Abstaende zu den naechsten Nachbarn (pro Partikel ein Wert).
std::vector<float> nearestDistances(const ParticleSystem& system);

}