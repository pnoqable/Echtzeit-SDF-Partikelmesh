#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <glm/glm.hpp>
#include <fstream>
#include <string>

#include "simulation/SDF.hpp"
#include "simulation/PrimitiveSDF.hpp"
#include "simulation/ParticleSystem.hpp"
#include "render/SceneRenderer.hpp"

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

    while (!WindowShouldClose()) {
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


        // Mouse drag
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 delta = GetMouseDelta();
            camYaw   -= delta.x * kRotateSpeed;
            camPitch += delta.y * kRotateSpeed;
        }
        // Arrow keys / WASD
        float dt = GetFrameTime();
        if (IsKeyDown(KEY_LEFT)  || IsKeyDown(KEY_A)) camYaw   += kKeySpeed * dt;
        if (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D)) camYaw   -= kKeySpeed * dt;
        if (IsKeyDown(KEY_UP)    || IsKeyDown(KEY_W)) camPitch -= kKeySpeed * dt;
        if (IsKeyDown(KEY_DOWN)  || IsKeyDown(KEY_S)) camPitch += kKeySpeed * dt;
        camPitch = Clamp(camPitch, -1.55f, 1.55f);

        // Scroll + +/- zoom
        camDist -= GetMouseWheelMove() * kZoomSpeed;
        if (IsKeyDown(KEY_EQUAL) || IsKeyDown(KEY_KP_ADD)) camDist -= kZoomSpeed * dt * 60.0f;
        if (IsKeyDown(KEY_MINUS) || IsKeyDown(KEY_KP_SUBTRACT)) camDist += kZoomSpeed * dt * 60.0f;
        camDist = fmaxf(camDist, kMinDist);

        // Update camera position from angles
        camera.target = { camTarget.x, camTarget.y, camTarget.z };
        camera.position = {
            camTarget.x + camDist * cosf(camPitch) * sinf(camYaw),
            camTarget.y + camDist * sinf(camPitch),
            camTarget.z + camDist * cosf(camPitch) * cosf(camYaw),
        };

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
