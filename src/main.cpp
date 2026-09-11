#include <raylib.h>
#include <rlgl.h>
#include <glm/glm.hpp>
#include <fstream>
#include <string>

#include "simulation/SDF.hpp"
#include "simulation/PrimitiveSDF.hpp"
#include "simulation/ParticleSystem.hpp"
#include "render/SceneRenderer.hpp"

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

    Camera3D camera = {
        .position = { 2.0f, 1.5f, 2.0f },
        .target = { 0.0f, 0.0f, 0.0f },
        .up = { 0.0f, 1.0f, 0.0f },
        .fovy = 45.0f,
        .projection = CAMERA_PERSPECTIVE,
    };

    SphereSDF sphere({0.0f, 0.0f, 0.0f}, 1.0f);
    ParticleSystem system;
    system.parameters.targetSpacing = 0.1f;
    system.initialize(1000, sphere.boundsMin(), sphere.boundsMax(), 42);
    system.projectToSDF(sphere);

    SceneRenderer renderer;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_F11)) {
            if (IsWindowMaximized())
                RestoreWindow();
            else
                MaximizeWindow();
        }

        UpdateCamera(&camera, CAMERA_ORBITAL);

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        renderer.drawAxes(2.0f);
        renderer.drawSDFBounds(sphere);
        renderer.drawParticles(system);

        EndMode3D();

        DrawFPS(10, 10);

        EndDrawing();
    }

    saveWindowState();
    CloseWindow();
    return 0;
}