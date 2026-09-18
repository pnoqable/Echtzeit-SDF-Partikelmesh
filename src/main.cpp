#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <fstream>
#include <string>

#include "rlImGui.h"
#include "imgui.h"

#include "simulation/SDF.hpp"
#include "simulation/PrimitiveSDF.hpp"
#include "simulation/ParticleSystem.hpp"
#include "mesh/Triangulation.hpp"
#include "mesh/VoronoiDual.hpp"
#include "render/SceneRenderer.hpp"
#include "platform/SystemTheme.hpp"
#include "core/Profiler.hpp"
#include "debug/Metrics.hpp"
#include <chrono>
#include <memory>

namespace {

// Fenstergroesse/-Position im aktuellen Ausfuehrungsverzeichnis
// persistieren (plain-text "window.txt").
const char* kWindowStateFile = "window.txt";

bool loadWindowState(int& w, int& h, int& x, int& y) {
    std::ifstream in(kWindowStateFile);
    if (!in.is_open()) return false;
    in >> w >> h >> x >> y;
    if (!in || w <= 0 || h <= 0) return false;
    return true;
}

void saveWindowState() {
    std::ofstream out(kWindowStateFile);
    if (!out.is_open()) return;
    out << GetScreenWidth() << " " << GetScreenHeight() << " "
        << GetWindowPosition().x << " " << GetWindowPosition().y << "\n";
}

} // namespace

int main() {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_WINDOW_HIGHDPI);
    InitWindow(screenWidth, screenHeight, "SDF Particle Mesh");
    int w = 0, h = 0, x = 0, y = 0;
    if (loadWindowState(w, h, x, y)) {
        SetWindowSize(w, h);
        SetWindowPosition(x, y);
    }
    SetTargetFPS(60);
    rlImGuiSetup(true);

    SystemTheme::Theme theme = SystemTheme::currentTheme();
    if (theme == SystemTheme::Theme::Dark)
        ImGui::StyleColorsDark();
    else
        ImGui::StyleColorsLight();

    Camera3D camera = {
        .position = { 2.0f, 1.5f, 2.0f },
        .target = { 0.0f, 0.0f, 0.0f },
        .up = { 0.0f, 1.0f, 0.0f },
        .fovy = 45.0f,
        .projection = CAMERA_PERSPECTIVE,
    };

    Vector3 camTarget = { 0.0f, 0.0f, 0.0f };
    Vector3 camOffset = Vector3Subtract(camera.position, camTarget);
    float camDist  = Vector3Length(camOffset);
    float camYaw   = atan2f(camOffset.x, camOffset.z);
    float camPitch = asinf(camOffset.y / camDist);
    constexpr float kRotateSpeed = 0.003f;
    constexpr float kKeySpeed    = 2.0f;
    constexpr float kZoomSpeed   = 0.1f;
    constexpr float kMinDist     = 0.3f;

    SphereSDF defaultSphere({0.0f, 0.0f, 0.0f}, 1.0f);
    int particleCount = 1000;
    std::unique_ptr<SDF> activeSDF = std::make_unique<SphereSDF>(defaultSphere);
    ParticleSystem system;
    system.parameters.targetSpacing = 0.1f;
    system.initialize(particleCount, activeSDF->boundsMin(), activeSDF->boundsMax(), 42);
    system.projectToSDF(*activeSDF);
    system.buildSpatialHash();

    const SimulationParameters referenceParams = system.parameters;

    SceneRenderer renderer;
    renderer.setTheme(theme);
    Triangulation::Parameters triParams;
    float maxEdgeMul = triParams.maxEdgeLength;
    std::vector<glm::vec3> meshPositions;
    MeshStats triStats;

    bool showAxes = true;
    bool showBounds = true;
    bool paused = true;
    bool showMesh = true;
    bool showParticles = true;
    bool showHeatmap = true;
    bool singleStep = false;
    bool wireframe = true;
    bool enableLighting = true;
    float lightKeyIntensity = 1.0f;    // Key-Licht (warm-weiss, oben rechts)
    float lightFillIntensity = 0.4f;   // Fill-Licht (kuehl, unten links)
    float lightAmbient = 0.12f;        // Umgebungslicht fuer die Schattenseiten
    bool meshReady = false;
    int topologyRevision = 0;
    long long topologyAliveFrames = 0;

    // Debug-Overlays (M3)
    bool showSelectionGrid = true;
    bool showSelectionNeighbors = true;
    bool showSelectionForces = true;
    bool showSelectionNormal = true;
    bool showSpatialGrid = false;
    bool showSDFProjection = false;
    bool showQuality = false;
    bool showVoronoi = false;
    float viewShiftPx = 150.0f;    // Hauptansicht nach rechts verschieben (off-center)
    float poorAngleDeg = 20.0f;
    int selectedParticle = -1;
    std::vector<glm::vec3> trail;
    VoronoiDual voronoiDual;
    std::vector<float> distHistogram;

    float actualSpacing = std::sqrt(activeSDF->surfaceArea() / static_cast<float>(system.particles.size()));

    debug::SimulationMetrics simMetrics;
    bool simMetricsValid = false;
    float simMs = 0.0f, gridMs = 0.0f;

    // Auto-Rebuild: Triangulation waehrend der Simulation automatisch neu erzeugen
    // (reines Debug-Feature; unterbricht bewusst die persistente Topologie).
    bool autoRebuild = true;
    int rebuildInterval = 60; // Frames
    int rebuildTicker = 0;

    auto rebuildTopology = [&]() {
        auto _t = prof::Profiler::instance().scoped("triangulate");
        std::vector<glm::vec3> pos;
        std::vector<glm::vec3> nrm;
        pos.reserve(system.particles.size());
        nrm.reserve(system.particles.size());
        for (const auto& p : system.particles) {
            pos.push_back(p.position);
            nrm.push_back(p.normal);
        }
        Triangulation tri;
        triParams.maxEdgeLength = maxEdgeMul;
        // Mittlere Punktdichte: h = sqrt(A / N). Die Hex-Formel
        // sqrt(2A/(sqrt(3) N)) ergibt bei relaxierten Verteilungen Randkanten.
        float spacingNow = std::sqrt(activeSDF->surfaceArea() / static_cast<float>(system.particles.size()));
        tri.build(pos, nrm, spacingNow, *activeSDF, triParams, &system.pool());
        system.triangles = tri.triangles();
        triStats = tri.stats();
        voronoiDual.build(pos, nrm, system.triangles);
        meshReady = !system.triangles.empty();
        topologyRevision++;
        topologyAliveFrames = 0;
    };

    // SDF-Form (M6): Auswahl + Parameter; Wechsel initialisiert Partikel neu.
    // 0=Kugel, 1=Ellipsoid, 2=Torus, 3=Hantel (konkav), 4=Metaball (weich),
    // 5=Kugel-minus-Kugel (CSG), 6=Felsbrocken (fbm-displaced)
    int sdfShape = 0;
    float shapeR = 1.0f;           // Kugelradius / Dumbbell-/Metaball-Radius / Basis-Radius (CSG)
    float shapeRx = 1.5f, shapeRy = 0.8f, shapeRz = 1.0f; // Ellipsoid
    float shapeMajor = 1.2f, shapeMinor = 0.45f;          // Torus
    float shapeHalfSep = 0.5f;     // Dumbbell/Metaball: halber Mittelpunktsabstand
    float shapeSmoothK = 0.5f;     // Metaball: Smooth-Min-Parameter (Wärme)
    float shapeCutR = 0.4f;        // CSG: Radius der abgezogenen Kugel
    float shapeCutOff = 0.9f;      // CSG: Versatz der abgezogenen Kugel (+x)
    float shapeRockRx = 1.0f, shapeRockRy = 0.85f, shapeRockRz = 1.15f; // Felsbrocken: Halbachsen
    float shapeRockAmp = 0.28f;    // Felsbrocken: Displacement-Amplitude
    float shapeRockFreq = 1.6f;    // Felsbrocken: Noise-Frequenz (niedrig = detailarm)
    int shapeRockOct = 3;          // Felsbrocken: fbm-Oktaven
    int shapeRockSeed = 0;         // Felsbrocken: Seed (0..49 Rasterstellungen)

    auto applySDFForm = [&]() {
        switch (sdfShape) {
            case 0: activeSDF = std::make_unique<SphereSDF>(glm::vec3(0.0f), shapeR); break;
            case 1: activeSDF = std::make_unique<EllipsoidSDF>(glm::vec3(0.0f), shapeRx, shapeRy, shapeRz); break;
            case 2: activeSDF = std::make_unique<TorusSDF>(glm::vec3(0.0f), shapeMajor, shapeMinor); break;
            case 3: activeSDF = std::make_unique<DumbbellSDF>(glm::vec3(0.0f), shapeR, shapeHalfSep); break;
            case 4: activeSDF = std::make_unique<MetaballSDF>(glm::vec3(0.0f), shapeR, shapeHalfSep, shapeSmoothK); break;
            case 5: activeSDF = std::make_unique<SphereMinusSphereSDF>(glm::vec3(0.0f), shapeR, shapeCutR, shapeCutOff, shapeSmoothK); break;
            case 6: activeSDF = std::make_unique<RockSDF>(glm::vec3(0.0f), shapeRockRx, shapeRockRy, shapeRockRz, shapeRockAmp, shapeRockFreq, shapeRockOct, shapeRockSeed); break;
        }
        system.initialize(particleCount, activeSDF->boundsMin(), activeSDF->boundsMax(), 42);
        system.projectToSDF(*activeSDF);
        system.buildSpatialHash();
        system.triangles.clear();
        voronoiDual.clear();
        meshReady = false;
        topologyRevision++;
        topologyAliveFrames = 0;
        selectedParticle = -1;
        trail.clear();
        actualSpacing = std::sqrt(activeSDF->surfaceArea() / static_cast<float>(particleCount));
    };

    const char* shapeNames[] = { "Kugel", "Ellipsoid", "Torus", "Hantel", "Metaball", "Kugel-minus-Kugel", "Felsbrocken" };

    while (!WindowShouldClose()) {
        rlImGuiBegin();

        if (IsKeyPressed(KEY_F11)) {
            if (IsWindowMaximized())
                RestoreWindow();
            else
                MaximizeWindow();
        }

        // Simulation pausieren/fortsetzen mit Leertaste (nicht, wenn ein
        // ImGui-Widget den Tastatur-Fokus hat).
        if (!ImGui::GetIO().WantCaptureKeyboard && IsKeyPressed(KEY_SPACE))
            paused = !paused;

        float dt = GetFrameTime();

        // Simulation
        bool simRan = (!paused || singleStep) && dt > 0.0f;
        if (simRan) {
            auto t0 = std::chrono::steady_clock::now();
            {
                auto _t = prof::Profiler::instance().scoped("sim");
                system.relax(dt, *activeSDF);
            }
            auto t1 = std::chrono::steady_clock::now();
            simMs = std::chrono::duration<float, std::milli>(t1 - t0).count();
            if (singleStep) {
                paused = true;
                singleStep = false;
            }
        }

        // Auto-Rebuild: waehrend der laufenden Simulation periodisch neu triangulieren
        if (autoRebuild && simRan && rebuildInterval > 0) {
            if (++rebuildTicker >= rebuildInterval) {
                rebuildTopology();
                rebuildTicker = 0;
            }
        } else if (!autoRebuild) {
            rebuildTicker = 0;
        }

        // Metriken (nur gelegentlich neu berechnen; Nachbarsuche ist O(N) über Grid)
        {
            static int metricTicker = 0;
            if (metricTicker++ % 10 == 0) {
                auto t0s = std::chrono::steady_clock::now();
                system.buildSpatialHash();
                {
                    auto _t = prof::Profiler::instance().scoped("evaluate");
                    simMetrics = debug::evaluate(system, *activeSDF, actualSpacing);
                }
                auto t1s = std::chrono::steady_clock::now();
                gridMs = std::chrono::duration<float, std::milli>(t1s - t0s).count();
                simMetrics = debug::evaluate(system, *activeSDF, actualSpacing);
                simMetricsValid = !system.particles.empty();
            }
        }

        // Kamera-Steuerung
        if (!ImGui::GetIO().WantCaptureMouse && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 delta = GetMouseDelta();
            camYaw   -= delta.x * kRotateSpeed;
            camPitch += delta.y * kRotateSpeed;
        }
        if (!ImGui::GetIO().WantCaptureKeyboard) {
            if (IsKeyDown(KEY_LEFT)  || IsKeyDown(KEY_A)) camYaw   += kKeySpeed * dt;
            if (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D)) camYaw   -= kKeySpeed * dt;
            if (IsKeyDown(KEY_UP)    || IsKeyDown(KEY_W)) camPitch -= kKeySpeed * dt;
            if (IsKeyDown(KEY_DOWN)  || IsKeyDown(KEY_S)) camPitch += kKeySpeed * dt;
        }
        camPitch = Clamp(camPitch, -1.55f, 1.55f);

        if (!ImGui::GetIO().WantCaptureMouse) {
            camDist -= GetMouseWheelMove() * kZoomSpeed;
            if (IsKeyDown(KEY_EQUAL) || IsKeyDown(KEY_KP_ADD)) camDist -= kZoomSpeed * dt * 60.0f;
            if (IsKeyDown(KEY_MINUS) || IsKeyDown(KEY_KP_SUBTRACT)) camDist += kZoomSpeed * dt * 60.0f;
        }
        camDist = fmaxf(camDist, kMinDist);

        camera.target = { camTarget.x, camTarget.y, camTarget.z };
        camera.position = {
            camTarget.x + camDist * cosf(camPitch) * sinf(camYaw),
            camTarget.y + camDist * sinf(camPitch),
            camTarget.z + camDist * cosf(camPitch) * cosf(camYaw),
        };

        // Partikel-Auswahl per Rechtsklick (Raycast auf Kugelmitte)
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && !ImGui::GetIO().WantCaptureMouse) {
            Vector2 mouse = GetMousePosition();
            mouse.x -= viewShiftPx; // Off-Center-Versatz der Ansicht kompensieren
            Ray ray = GetMouseRay(mouse, camera);
            int best = -1;
            float bestDenom = std::numeric_limits<float>::max();
            for (size_t i = 0; i < system.particles.size(); ++i) {
                Vector3 p = Vector3Subtract({system.particles[i].position.x, system.particles[i].position.y, system.particles[i].position.z}, ray.position);
                float t = Vector3DotProduct(p, ray.direction);
                if (t < 0) continue;
                Vector3 closest = Vector3Add(ray.position, Vector3Scale(ray.direction, t));
                float dist = Vector3Distance(closest, {system.particles[i].position.x, system.particles[i].position.y, system.particles[i].position.z});
                if (dist < bestDenom) { bestDenom = dist; best = static_cast<int>(i); }
            }
            selectedParticle = (best >= 0 && bestDenom < 0.05f) ? best : -1;
            trail.clear();
        }

        if (selectedParticle >= 0) {
            trail.push_back(system.particles[selectedParticle].position);
            if (trail.size() > 120) trail.erase(trail.begin());
        }

        BeginDrawing();
        ClearBackground(renderer.backgroundColor());
        auto _render = prof::Profiler::instance().scoped("render");

        BeginMode3D(camera);

        // Off-Center-Verschiebung: Die Szene wird um viewShiftPx nach rechts
        // gerendert, damit sie rechts neben der Debug-View zentriert ist.
        // Realisiert ueber ein schraeges Frustum (m8 = (r+l)/(r-l)); die
        // Verschiebung ist dadurch tiefenkonstant statt perspektivisch.
        {
            Matrix proj = rlGetMatrixProjection();
            proj.m8 -= 2.0f * viewShiftPx / static_cast<float>(GetScreenWidth());
            rlSetMatrixProjection(proj);
        }

        if (showAxes) renderer.drawAxes(2.0f);
        if (showBounds) renderer.drawSDFBounds(*activeSDF);
        renderer.setLighting(enableLighting);
        renderer.setLightIntensities(lightKeyIntensity, lightFillIntensity);
        renderer.setAmbient(lightAmbient);

        if (showMesh && meshReady) {
            meshPositions.resize(system.particles.size());
            for (size_t i = 0; i < system.particles.size(); ++i)
                meshPositions[i] = system.particles[i].position;
            renderer.drawMesh(meshPositions, system.triangles, wireframe, topologyRevision);
            ++topologyAliveFrames;
        }
        if (showQuality && meshReady) {
            renderer.drawMeshQuality(meshPositions, system.triangles, poorAngleDeg);
        }
        if (showVoronoi && meshReady) renderer.drawVoronoiDual(voronoiDual);
        if (showParticles) {
            if (showHeatmap) renderer.drawParticlesHeatmap(system, actualSpacing);
            else             renderer.drawParticles(system);
        }
        renderer.drawTrail(trail);
        if (showSpatialGrid) renderer.drawSpatialGrid(system);
        if (showSDFProjection) renderer.drawSDFProjections(system);
        renderer.drawParticleSelection(system, selectedParticle, showSelectionGrid, showSelectionNeighbors, showSelectionForces, showSelectionNormal);

        EndMode3D();

        {
            const ImGuiViewport* vp = ImGui::GetMainViewport();
            if (vp) {
                // Fenster an die Viewport-Arbeitsflaeche begrenzen: Ueberlaeuft der
                // Inhalt die Hoehe, zeigt ImGui eine Scrollbar statt den unteren
                // Teil abzuschneiden (z. B. unter Windows/Different-DPI).
                ImGui::SetNextWindowSizeConstraints(
                    ImVec2(220.0f, 80.0f),
                    ImVec2(vp->WorkSize.x - 8.0f, vp->WorkSize.y - 8.0f));
            }
        }
        ImGui::Begin("Debug");
        ImGui::Text("FPS: %d", GetFPS());
        ImGui::Text("Partikel: %zu", system.particles.size());
        ImGui::Text("Paare: %zu", system.spatialHash().pairs().size());

        if (ImGui::CollapsingHeader("SDF-Form", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Combo("Primitiv", &sdfShape, shapeNames, 7)) {
                applySDFForm();
            }
            bool paramsChanged = false;
            switch (sdfShape) {
                case 0:
                    paramsChanged |= ImGui::SliderFloat("Radius", &shapeR, 0.2f, 2.0f);
                    break;
                case 1:
                    paramsChanged |= ImGui::SliderFloat("Ra", &shapeRx, 0.2f, 2.0f);
                    paramsChanged |= ImGui::SliderFloat("Rb", &shapeRy, 0.2f, 2.0f);
                    paramsChanged |= ImGui::SliderFloat("Rc", &shapeRz, 0.2f, 2.0f);
                    break;
                case 2:
                    paramsChanged |= ImGui::SliderFloat("Major-Radius", &shapeMajor, 0.3f, 2.0f);
                    paramsChanged |= ImGui::SliderFloat("Minor-Radius", &shapeMinor, 0.05f, 1.0f);
                    break;
                case 3:
                    paramsChanged |= ImGui::SliderFloat("Kugelradius", &shapeR, 0.2f, 2.0f);
                    paramsChanged |= ImGui::SliderFloat("Ueberlappung", &shapeHalfSep, 0.05f, 2.0f);
                    break;
                case 4:
                    // Metaball: Ueberlappung darf >= R sein (auch getrennte Blobs).
                    paramsChanged |= ImGui::SliderFloat("Kugelradius", &shapeR, 0.2f, 2.0f);
                    paramsChanged |= ImGui::SliderFloat("Ueberlappung", &shapeHalfSep, 0.05f, 2.0f);
                    paramsChanged |= ImGui::SliderFloat("Smooth k", &shapeSmoothK, 0.0f, 2.0f);
                    break;
                case 5:
                    // Echte Ausnehmung: |R−r| < offset < R+r (Clamp wie in applySDFForm).
                    paramsChanged |= ImGui::SliderFloat("Basis-Radius", &shapeR, 0.2f, 2.0f);
                    paramsChanged |= ImGui::SliderFloat("Schnitt-Radius", &shapeCutR, 0.05f, 2.0f);
                    paramsChanged |= ImGui::SliderFloat("Versatz", &shapeCutOff, 0.05f, 3.0f);
                    paramsChanged |= ImGui::SliderFloat("Smooth k", &shapeSmoothK, 0.0f, 4.0f);
                    break;
                case 6:
                    paramsChanged |= ImGui::SliderFloat("Halbachse X", &shapeRockRx, 0.2f, 2.0f);
                    paramsChanged |= ImGui::SliderFloat("Halbachse Y", &shapeRockRy, 0.2f, 2.0f);
                    paramsChanged |= ImGui::SliderFloat("Halbachse Z", &shapeRockRz, 0.2f, 2.0f);
                    paramsChanged |= ImGui::SliderFloat("Rauheit", &shapeRockAmp, 0.0f, 0.5f);
                    paramsChanged |= ImGui::SliderFloat("Frequenz", &shapeRockFreq, 0.5f, 4.0f);
                    paramsChanged |= ImGui::SliderInt("Oktaven", &shapeRockOct, 1, 4);
                    paramsChanged |= ImGui::SliderInt("Seed", &shapeRockSeed, 0, 49);
                    ImGui::TextDisabled("Felsbrocken: Ellipsoid mit fraktalem Displacement.\n"
                                        "Seed 0..49 waehlt verschiedene Brocken (gleiche Rauheit).");
                    break;
            }
            if (paramsChanged) {
                applySDFForm();
            }
            ImGui::TextDisabled("Form-Wechsel oder Parameter-Aenderung setzt die\nPartikel mit Seed 42 neu auf und projiziert sie.");
        }

        if (ImGui::CollapsingHeader("Steuerung", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button(paused ? "Weiter" : "Pause"))
                paused = !paused;
            ImGui::SameLine();
            if (ImGui::Button("Einzelschritt")) {
                if (paused) {
                    paused = true;
                    singleStep = true;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset"))
                applySDFForm();
        }

        if (ImGui::CollapsingHeader("Simulationsparameter", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::SliderInt("Partikel", &particleCount, 1000, 10000)) {
                applySDFForm();
            }
            ImGui::SliderFloat("Repulsionsradius", &system.parameters.repulsionRadius, 0.01f, 0.15f);
            ImGui::SliderFloat("Staerke", &system.parameters.repulsionStrength, 0.01f, 1.0f);
            ImGui::SliderFloat("Daempfung", &system.parameters.damping, 0.f, 1.0f);
            ImGui::SliderInt("Substeps", &system.parameters.substeps, 1, 5);
            ImGui::SliderFloat("MaxSchritt", &system.parameters.maxStepLength, 0.01f, 0.3f);
            if (ImGui::Button("Parameter zuruecksetzen")) {
                system.parameters = referenceParams;
            }
        }

        if (ImGui::CollapsingHeader("Triangulation", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Triangulation erzeugen")) {
                rebuildTopology();
            }
            ImGui::SliderFloat("Max Kantenlaenge", &maxEdgeMul, 1.0f, 2.0f);
            ImGui::Separator();
            ImGui::Checkbox("Auto-Rebuild", &autoRebuild);
            if (autoRebuild) {
                ImGui::SliderInt("Intervall", &rebuildInterval, 1, 100);
            }
        }

        if (ImGui::CollapsingHeader("Ansicht", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Partikel anzeigen", &showParticles);
            ImGui::Checkbox("Partikel-Heatmap", &showHeatmap);
            ImGui::Checkbox("Achsen", &showAxes);
            ImGui::Checkbox("Bounding Box", &showBounds);
            ImGui::Checkbox("Grid (alle Zellen)", &showSpatialGrid);
            ImGui::Checkbox("SDF-Projektion", &showSDFProjection);
            ImGui::Checkbox("Mesh anzeigen", &showMesh);
            ImGui::Checkbox("Beleuchtung (2 Lichtquellen)", &enableLighting);
            if (enableLighting) {
                ImGui::SliderFloat("Key-Licht", &lightKeyIntensity, 0.0f, 2.0f);
                ImGui::SliderFloat("Fill-Licht", &lightFillIntensity, 0.0f, 2.0f);
                ImGui::SliderFloat("Ambient", &lightAmbient, 0.0f, 0.5f);
            }
            ImGui::Checkbox("Wireframe", &wireframe);
            ImGui::Checkbox("Voronoi-Dual", &showVoronoi);
            ImGui::SliderFloat("View-Versatz", &viewShiftPx, 0.0f, 400.0f);
        }

        if (ImGui::CollapsingHeader("Auswahl", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (selectedParticle >= 0) {
                ImGui::Checkbox("Grid-Zelle", &showSelectionGrid);
                ImGui::SameLine();
                ImGui::Checkbox("Nachbarn", &showSelectionNeighbors);
                ImGui::SameLine();
                ImGui::Checkbox("Kraefte", &showSelectionForces);
                ImGui::SameLine();
                ImGui::Checkbox("Normale", &showSelectionNormal);
                if (selectedParticle < static_cast<int>(system.particles.size())) {
                    const Particle& p = system.particles[selectedParticle];
                    ImGui::Text("Partikel #%d", selectedParticle);
                    ImGui::Text("  pos (%.3f, %.3f, %.3f)", p.position.x, p.position.y, p.position.z);
                    ImGui::Text("  phi = %.2e", activeSDF->sample(p.position).distance);
                }
                int neighbors = 0;
                for (const auto& pair : system.spatialHash().pairs())
                    if (pair.i == static_cast<uint32_t>(selectedParticle) || pair.j == static_cast<uint32_t>(selectedParticle)) ++neighbors;
                ImGui::Text("  Nachbarn: %d", neighbors);
            } else {
                ImGui::TextDisabled("Rechtsklick auf einen Partikel, um ihn auszuwaehlen.");
            }
        }

        if (ImGui::CollapsingHeader("Stats", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (meshReady) {
                ImGui::Text("Dreiecke: %d  (degen: %d, orient: %d)", triStats.totalTriangles, triStats.degenerate, triStats.wrongOrientation);
                ImGui::Text("Kanten: abgelehnt (laenge %d, manifold %d)",
                    triStats.rejectedLength, triStats.rejectedManifold);
                ImGui::Text("Topologie: persistent seit %lld Frames", topologyAliveFrames);
            }
            if (simMetricsValid) {
                ImGui::Text("Zeit: Sim %.2f ms, Grid %.2f ms", simMs, gridMs);
                ImGui::Text("phi: avg %.2e  max %.2e", simMetrics.sdf.avgAbsPhi, simMetrics.sdf.maxAbsPhi);
                ImGui::Text("Abstand: min %.4f  avg %.4f  max %.4f", simMetrics.distribution.minDist,
                    simMetrics.distribution.avgDist, simMetrics.distribution.maxDist);
                ImGui::Text("  StdAbw %.4f   unter/ok/ueber %d/%d/%d", simMetrics.distribution.stdDev,
                    simMetrics.distribution.underCount, simMetrics.distribution.okCount,
                    simMetrics.distribution.overCount);
                ImGui::Text("v: avg %.2e  max %.2e", simMetrics.avgSpeed, simMetrics.maxSpeed);
                if (meshReady) {
                    ImGui::Text("Mesh-Qualitaet: minWinkel %.1f°  maxAspect %.2f  poor %d",
                        simMetrics.mesh.minAngleDeg, simMetrics.mesh.maxAspectRatio, simMetrics.mesh.poorTriangles);
                }
            }
            distHistogram = debug::spacingHistogram(system, actualSpacing, 24, 3.0f);
            ImGui::Text("Abstands-Verteilung");
            ImGui::PlotHistogram("##dist", distHistogram.data(), static_cast<int>(distHistogram.size()), 0, nullptr,
                0.0f, std::numeric_limits<float>::max(), ImVec2(0, 60));
            ImGui::Checkbox("Mesh-Qualitaet", &showQuality);
            ImGui::SliderFloat("Poor-Winkel", &poorAngleDeg, 5.0f, 60.0f);
        }

        if constexpr (prof::enabled) {
            if (ImGui::CollapsingHeader("Performance", ImGuiTreeNodeFlags_DefaultOpen)) {
                auto snap = prof::Profiler::instance().stages();
                for (auto& s : snap) {
                    ImGui::Text("%-16s %8.2f ms", s.name,
                        static_cast<double>(s.c.frameNs) * 1e-6);
                }
            }
        }

        ImGui::TextDisabled("Steuerung:\nLeertaste: Pause\nMaus-Drag: Rotieren\nScroll/+/-: Zoom\nWASD/Pfeiltasten: Rotieren\nF11: Maximieren");
        ImGui::End();

        rlImGuiEnd();
        EndDrawing();

        prof::Profiler::instance().endFrame();
    }

    rlImGuiShutdown();
    saveWindowState();
    CloseWindow();
    return 0;
}