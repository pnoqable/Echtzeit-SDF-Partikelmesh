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
#include "render/SceneRenderer.hpp"
#include "platform/SystemTheme.hpp"
#include "debug/Metrics.hpp"

namespace {

// Fenstergroesse/-Position/Maximiert-Zustand im aktuellen Ausfuehrungs-
// verzeichnis persistieren (plain-text "window.txt").
// Format: "<Breite> <Hoehe> <PosX> <PosY> [Maximiert]"  (5. Feld optional).
const char* kWindowStateFile = "window.txt";

int gRestoredW = 1280;   // "normale" (nicht maximierte) Fenstergroesse, logische Pixel
int gRestoredH = 720;
int gWinX = 0;
int gWinY = 0;
bool gMaximized = false;

// Logische (Bildschirm-)Groesse des Fensters, unabhaengig von der
// HiDPI-Semantik der gebauten raylib-Version:
//   - gepatchte raylib (RaylibHiDPIResizeFix): GetScreenWidth() == logisch,
//     GetRenderWidth() == physisch
//   - ungepatchte raylib (vor dem HiDPI-Fix gebaut): GetScreenWidth() liefert
//     physisch
// In beiden Faellen ist logisch == GetRenderWidth() / GetWindowScaleDPI().
// SetWindowSize() erwartet genau diese logische Groesse.
void updateLogicalWindowSize() {
    const Vector2 scale = GetWindowScaleDPI();
    const int renderW = GetRenderWidth(), renderH = GetRenderHeight();
    gRestoredW = (scale.x > 0.0f)
        ? static_cast<int>(std::lround(static_cast<float>(renderW) / scale.x))
        : renderW;
    gRestoredH = (scale.y > 0.0f)
        ? static_cast<int>(std::lround(static_cast<float>(renderH) / scale.y))
        : renderH;
}

bool loadWindowState() {
    std::ifstream in(kWindowStateFile);
    if (!in.is_open()) return false;
    int w = 0, h = 0, x = 0, y = 0, m = 0;
    in >> w >> h >> x >> y;
    const bool haveMax = static_cast<bool>(in >> m); // altes 4-Werte-Format: false
    if (in.bad() || w <= 0 || h <= 0) return false;
    gRestoredW = w;
    gRestoredH = h;
    gWinX = x;
    gWinY = y;
    gMaximized = haveMax && (m != 0);
    return true;
}

void saveWindowState() {
    std::ofstream out(kWindowStateFile);
    if (!out.is_open()) return;
    // Letzte normale Groesse (laufend per updateLogicalWindowSize getrackt,
    // logische Pixel) + Maximiert-Flag. So interpretiert SetWindowSize() beim
    // Laden die Groesse korrekt, auch wenn die getrackte Groesse physisch
    // (skaliert) waere.
    out << gRestoredW << " " << gRestoredH << " "
        << gWinX << " " << gWinY << " "
        << (IsWindowMaximized() ? 1 : 0) << "\n";
}

} // namespace

int main() {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_WINDOW_HIGHDPI);
    InitWindow(screenWidth, screenHeight, "SDF Particle Mesh");
    if (loadWindowState()) {
        SetWindowSize(gRestoredW, gRestoredH);
        SetWindowPosition(gWinX, gWinY);
        if (gMaximized)
            MaximizeWindow();
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

    SphereSDF sphere({0.0f, 0.0f, 0.0f}, 1.0f);
    ParticleSystem system;
    system.parameters.targetSpacing = 0.1f;
    system.initialize(1000, sphere.boundsMin(), sphere.boundsMax(), 42);
    system.projectToSDF(sphere);
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
    bool meshReady = false;
    int topologyRevision = 0;

    // Debug-Overlays (M3)
    bool showSelectionGrid = true;
    bool showSelectionNeighbors = true;
    bool showSelectionForces = true;
    bool showSelectionNormal = true;
    bool showSpatialGrid = false;
    bool showSDFProjection = false;
    bool showQuality = true;
    float poorAngleDeg = 20.0f;
    int selectedParticle = -1;
    std::vector<glm::vec3> trail;
    std::vector<float> distHistogram;

    float radius = 0.5f * (sphere.boundsMax().x - sphere.boundsMin().x);
    float actualSpacing = std::sqrt(4.0f * glm::pi<float>() * radius * radius / static_cast<float>(system.particles.size()));

    debug::SimulationMetrics simMetrics;
    bool simMetricsValid = false;

    while (!WindowShouldClose()) {
        rlImGuiBegin();

        if (IsKeyPressed(KEY_F11)) {
            if (IsWindowMaximized())
                RestoreWindow();
            else
                MaximizeWindow();
        }

        // Restore-Groesse/-Position laufend nachfuehren, solange das Fenster
        // normal (nicht maximiert) ist. So wird auch eine Maximierung ueber
        // den OS-Titelbutton erkannt, ohne die letzte normale Groesse zu
        // verlieren; beim Beenden im maximierten Zustand persistiert
        // saveWindowState() diese normale Groesse plus das Maximiert-Flag.
        if (!IsWindowMaximized()) {
            updateLogicalWindowSize();
            const Vector2 wp = GetWindowPosition();
            gWinX = static_cast<int>(wp.x);
            gWinY = static_cast<int>(wp.y);
        }

        float dt = GetFrameTime();

        // Simulation
        if (!paused || singleStep) {
            system.relax(dt, sphere);
            if (singleStep) {
                paused = true;
                singleStep = false;
            }
        }

        simMetrics = debug::evaluate(system, sphere, actualSpacing);
        simMetricsValid = !system.particles.empty();

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
            Ray ray = GetMouseRay(GetMousePosition(), camera);
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

        BeginMode3D(camera);

        if (showAxes) renderer.drawAxes(2.0f);
        if (showBounds) renderer.drawSDFBounds(sphere);
        if (showMesh && meshReady) {
            meshPositions.resize(system.particles.size());
            for (size_t i = 0; i < system.particles.size(); ++i)
                meshPositions[i] = system.particles[i].position;
            renderer.drawMesh(meshPositions, system.triangles, wireframe, topologyRevision);
        }
        if (showQuality && meshReady) {
            renderer.drawMeshQuality(meshPositions, system.triangles, poorAngleDeg);
        }
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
        ImGui::Separator();

        if (ImGui::CollapsingHeader("Triangulation", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Triangulation erzeugen")) {
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
                glm::vec3 bmin = sphere.boundsMin(), bmax = sphere.boundsMax();
                float radius = 0.5f * (bmax.x - bmin.x);
                // Mittlere Punktdichte: h = sqrt(A / N). Die Hex-Formel
                // sqrt(2A/(sqrt(3) N)) ergibt bei relaxierten Verteilungen Randkanten.
                float area = 4.0f * glm::pi<float>() * radius * radius;
                float actualSpacing = std::sqrt(area / static_cast<float>(system.particles.size()));
                tri.build(pos, nrm, actualSpacing, sphere, triParams);
                system.triangles = tri.triangles();
                triStats = tri.stats();
                meshReady = !system.triangles.empty();
                topologyRevision++;
            }
            ImGui::SetNextItemWidth(150.0f);
            ImGui::SliderFloat("Max Kantenlaenge (x h)", &maxEdgeMul, 1.0f, 3.0f);
        }
        ImGui::Separator();

        if (ImGui::CollapsingHeader("Ansicht", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Mesh anzeigen", &showMesh);
            ImGui::Checkbox("Wireframe", &wireframe);
            ImGui::Checkbox("Partikel anzeigen", &showParticles);
            ImGui::Checkbox("Partikel-Heatmap", &showHeatmap);
            ImGui::Checkbox("Achsen", &showAxes);
            ImGui::Checkbox("Bounding Box", &showBounds);
            ImGui::Checkbox("Grid (besetzte Zellen)", &showSpatialGrid);
            ImGui::Checkbox("SDF-Projektion", &showSDFProjection);
        }
        ImGui::Separator();

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
                    ImGui::Text("  phi = %.2e", sphere.sample(p.position).distance);
                }
                int neighbors = 0;
                for (const auto& pair : system.spatialHash().pairs())
                    if (pair.i == static_cast<uint32_t>(selectedParticle) || pair.j == static_cast<uint32_t>(selectedParticle)) ++neighbors;
                ImGui::Text("  Nachbarn: %d", neighbors);
            } else {
                ImGui::TextDisabled("Rechtsklick auf einen Partikel, um ihn auszuwaehlen.");
            }
        }
        ImGui::Separator();

        if (ImGui::CollapsingHeader("Simulationsparameter", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SetNextItemWidth(150.0f);
            ImGui::SliderFloat("Repulsionsradius", &system.parameters.repulsionRadius, 0.01f, 0.5f);
            ImGui::SetNextItemWidth(150.0f);
            ImGui::SliderFloat("Staerke", &system.parameters.repulsionStrength, 0.01f, 5.0f);
            ImGui::SetNextItemWidth(150.0f);
            ImGui::SliderFloat("Daempfung", &system.parameters.damping, 0.f, 1.0f);
            ImGui::SetNextItemWidth(150.0f);
            ImGui::SliderInt("Substeps", &system.parameters.substeps, 1, 16);
            ImGui::SetNextItemWidth(150.0f);
            ImGui::SliderFloat("MaxSchritt", &system.parameters.maxStepLength, 0.01f, 0.5f);
            if (ImGui::Button("Parameter zuruecksetzen")) {
                system.parameters = referenceParams;
            }
        }
        ImGui::Separator();

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
            if (ImGui::Button("Reset")) {
                system.initialize(1000, sphere.boundsMin(), sphere.boundsMax(), 42);
                system.projectToSDF(sphere);
                system.buildSpatialHash();
                system.triangles.clear();
                paused = true;
                meshReady = false;
                topologyRevision++;
                selectedParticle = -1;
                trail.clear();
            }
        }
        ImGui::Separator();

        if (ImGui::CollapsingHeader("Stats", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (meshReady) {
                ImGui::Text("Dreiecke: %d  (degen: %d, orient: %d)", triStats.totalTriangles, triStats.degenerate, triStats.wrongOrientation);
                ImGui::Text("Kanten: abgelehnt (laenge %d, normal %d, mid %d, manifold %d)",
                    triStats.rejectedLength, triStats.rejectedNormal, triStats.rejectedMidpoint, triStats.rejectedManifold);
            }
            if (simMetricsValid) {
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
            ImGui::SetNextItemWidth(150.0f);
            ImGui::SliderFloat("Poor-Winkel", &poorAngleDeg, 5.0f, 60.0f);
        }
        ImGui::Separator();

        ImGui::TextDisabled("Steuerung:\nMaus-Drag: Rotieren\nScroll/+/-: Zoom\nWASD/Pfeiltasten: Rotieren\nF11: Maximieren");
        ImGui::End();

        rlImGuiEnd();
        EndDrawing();
    }

    rlImGuiShutdown();
    saveWindowState();
    CloseWindow();
    return 0;
}
