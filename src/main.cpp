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
#include <chrono>

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

    SphereSDF sphere({0.0f, 0.0f, 0.0f}, 1.0f);
    ParticleSystem system;
    system.parameters.targetSpacing = 0.1f;
    system.initialize(1000, sphere.boundsMin(), sphere.boundsMax(), 42);
    system.projectToSDF(sphere);

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

    float radius = 0.5f * (sphere.boundsMax().x - sphere.boundsMin().x);
    float actualSpacing = std::sqrt(4.0f * glm::pi<float>() * radius * radius / static_cast<float>(system.particles.size()));

    debug::SimulationMetrics simMetrics;
    bool simMetricsValid = false;
    float simMs = 0.0f, gridMs = 0.0f;

    while (!WindowShouldClose()) {
        rlImGuiBegin();

        if (IsKeyPressed(KEY_F11)) {
            if (IsWindowMaximized())
                RestoreWindow();
            else
                MaximizeWindow();
        }

        float dt = GetFrameTime();

        // Simulation
        if (!paused || singleStep) {
            auto t0 = std::chrono::steady_clock::now();
            system.relax(dt, sphere);
            auto t1 = std::chrono::steady_clock::now();
            simMs = std::chrono::duration<float, std::milli>(t1 - t0).count();
            if (singleStep) {
                paused = true;
                singleStep = false;
            }
        }

        // Metriken (nur gelegentlich neu berechnen; Nachbarsuche ist O(N) über Grid)
        {
            static int metricTicker = 0;
            if (metricTicker++ % 10 == 0) {
                auto t0s = std::chrono::steady_clock::now();
                system.buildSpatialHash();
                auto t1s = std::chrono::steady_clock::now();
                gridMs = std::chrono::duration<float, std::milli>(t1s - t0s).count();
                simMetrics = debug::evaluate(system, sphere, actualSpacing);
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
        if (showParticles) {
            if (showHeatmap) renderer.drawParticlesHeatmap(system, actualSpacing);
            else             renderer.drawParticles(system);
        }

        EndMode3D();

        ImGui::Begin("Debug");
        ImGui::Text("FPS: %d", GetFPS());
        ImGui::Text("Partikel: %zu", system.particles.size());
        ImGui::Text("Paare: %zu", system.spatialHash().pairs().size());
        ImGui::Separator();
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
            system.triangles.clear();
            paused = true;
            meshReady = false;
            topologyRevision++;
        }
        ImGui::Separator();
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
if (meshReady) {
            ImGui::Text("Dreiecke: %d  (degen: %d, orient: %d)", triStats.totalTriangles, triStats.degenerate, triStats.wrongOrientation);
            ImGui::Text("Kanten: abgelehnt (laenge %d, normal %d, mid %d, manifold %d)",
                triStats.rejectedLength, triStats.rejectedNormal, triStats.rejectedMidpoint, triStats.rejectedManifold);
        }
        ImGui::Checkbox("Mesh anzeigen", &showMesh);
        ImGui::Checkbox("Wireframe", &wireframe);
        ImGui::Checkbox("Partikel anzeigen", &showParticles);
        ImGui::Checkbox("Partikel-Heatmap", &showHeatmap);
        ImGui::Separator();
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
        ImGui::Separator();
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
        ImGui::Separator();
        ImGui::Checkbox("Achsen", &showAxes);
        ImGui::Checkbox("Bounding Box", &showBounds);
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