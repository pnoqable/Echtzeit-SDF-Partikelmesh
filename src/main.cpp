#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <fstream>
#include <string>
#include <queue>
#include <algorithm>

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
#include <memory>

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
    bool showHeatmap = false;
    bool singleStep = false;
    bool wireframe = true;
    // Auto-Ende bei Simulationsstillstand: pausiert, sobald die Partikel
    // lange genug praktisch stillstehen (Kriterium unten im Simulationsloop).
    bool autoConverge = true;
    int convStableFrames = 0;
    constexpr float kConvDispFrac   = 0.002f; // mittlere Verschiebung/Frame < 0.2% von h
    constexpr float kConvMaxFrac    = 0.004f;  // max |v| < 0.4% h/dt
    constexpr int   kConvStableNeed = 1800;   // 30s Stillstand bei 60fps
    bool enableLighting = true;
    bool cameraLighting = false;      // kamera-feste Lichtrichtung (View-Raum) statt objektfester (Weltraum)
    bool smoothShading = false;       // weiche Vertex-Normalen statt flacher Face-Normalen
    bool roughTexture = false;        // raue Fraktal-Textur im Smooth-Modus
    float roughness = 0.25f;          // Kipp-Amplitude der fraktalen Normalentoerung
    float roughFreq = 20.0f;          // Detailgroesse des Fraktal-Noises
    float lightKeyIntensity = 1.5f;    // Key-Licht (warm-weiss, oben rechts)
    float lightFillIntensity = 0.1f;   // Fill-Licht (kuehl, unten links)
    float lightAmbient = 0.01f;        // Umgebungslicht fuer die Schattenseiten
    bool meshReady = false;
    int topologyRevision = 0;
    long long topologyAliveFrames = 0;
    int selectedCell = -1;  // per Linksklick ausgewaehlte Voronoi-Zelle (-1 = keine)
    Vector2 clickDownMouse = {};   // Klick-Position beim Maeuse-Druck (Linksklick)
    bool leftClickArmed = false;   // Linksklick scharf, solange vor dem Loslassen nicht gedragt wird
    // Kuerzester Pfad durch den Zell-Nachbarschaftsgraphen (Rechtsklick-Ziel).
    // Inhalt: Zell-Indizes von selectedCell bis zum Ziel (-1 = kein Pfad).
    std::vector<int> pathCells;
    Vector2 clickDownMouseR = {};  // Klick-Position beim Maeuse-Druck (Rechtsklick)
    bool rightClickArmed = false;  // Rechtsklick scharf, solange vor dem Loslassen nicht gedragt wird

    // Debug-Overlays (M3)
    bool showSpatialGrid = false;
    bool showSDFProjection = false;
    bool showVoronoiFill = false;  // Zellflaechen des Duals (mit Flat-Shading)
    bool showVoronoiWire = false;  // Zellgrenzkanten des Duals (unabhaengig)
    float viewShiftPx = 150.0f;    // Hauptansicht nach rechts verschieben (off-center)
    VoronoiDual voronoiDual;
    std::vector<float> distHistogram;

    float actualSpacing = std::sqrt(activeSDF->surfaceArea() / static_cast<float>(system.particles.size()));

    debug::SimulationMetrics simMetrics;
    bool simMetricsValid = false;

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
    // 5=Kugel-minus-Kugel (CSG), 6=Felsbrocken (fbm-displaced),
    // 7=Fels-Torus (fbm-displaced Torus)
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
    float shapeRockTmajor = 1.2f, shapeRockTminor = 0.5f; // Fels-Torus: Ring-Radien (Minor etwas dicker fuer stabilere Projektion)

    auto applySDFForm = [&]() {
        switch (sdfShape) {
            case 0: activeSDF = std::make_unique<SphereSDF>(glm::vec3(0.0f), shapeR); break;
            case 1: activeSDF = std::make_unique<EllipsoidSDF>(glm::vec3(0.0f), shapeRx, shapeRy, shapeRz); break;
            case 2: activeSDF = std::make_unique<TorusSDF>(glm::vec3(0.0f), shapeMajor, shapeMinor); break;
            case 3: activeSDF = std::make_unique<DumbbellSDF>(glm::vec3(0.0f), shapeR, shapeHalfSep); break;
            case 4: activeSDF = std::make_unique<MetaballSDF>(glm::vec3(0.0f), shapeR, shapeHalfSep, shapeSmoothK); break;
            case 5: activeSDF = std::make_unique<SphereMinusSphereSDF>(glm::vec3(0.0f), shapeR, shapeCutR, shapeCutOff, shapeSmoothK); break;
            case 6: activeSDF = std::make_unique<RockSDF>(glm::vec3(0.0f), shapeRockRx, shapeRockRy, shapeRockRz, shapeRockAmp, shapeRockFreq, shapeRockOct, shapeRockSeed); break;
            case 7: activeSDF = std::make_unique<RockTorusSDF>(glm::vec3(0.0f), shapeRockTmajor, shapeRockTminor, shapeRockAmp, shapeRockFreq, shapeRockOct, shapeRockSeed); break;
        }
        system.initialize(particleCount, activeSDF->boundsMin(), activeSDF->boundsMax(), 42);
        system.projectToSDF(*activeSDF);
        system.buildSpatialHash();
        system.triangles.clear();
        voronoiDual.clear();
        meshReady = false;
        topologyRevision++;
        topologyAliveFrames = 0;
        selectedCell = -1;
        pathCells.clear();
        actualSpacing = std::sqrt(activeSDF->surfaceArea() / static_cast<float>(particleCount));
    };

    const char* shapeNames[] = { "Kugel", "Ellipsoid", "Torus", "Hantel", "Metaball", "Kugel-minus-Kugel", "Felsbrocken", "Fels-Torus" };

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

        // Simulation pausieren/fortsetzen mit Leertaste (nicht, wenn ein
        // ImGui-Widget den Tastatur-Fokus hat).
        if (!ImGui::GetIO().WantCaptureKeyboard && IsKeyPressed(KEY_SPACE))
            paused = !paused;

        float dt = GetFrameTime();

        // Simulation
        if (!paused || singleStep) {
            {
                auto _t = prof::Profiler::instance().scoped("sim");
                system.relax(dt, *activeSDF);
                singleStep = false;
            }

            // Auto-Rebuild: waehrend der laufenden Simulation periodisch neu triangulieren
            if (autoRebuild && ++rebuildTicker >= rebuildInterval) {
                rebuildTopology();
                rebuildTicker = 0;
            }
        }

        // Auto-Ende bei Simulationsstillstand: Solange die mittlere (und
        // maximale) Geschwindigkeit winzig ist relativ zur Partikelweite h
        // und zur Frameratedauer dt, bewegt sich praktisch nichts mehr an der
        // Verteilung -> Simulation pausieren und einmalig final genau
        // triangulieren, damit sich die Topologie danach nicht mehr aendert.
        if (autoConverge && !paused) {
            double sumV = 0.0, maxV = 0.0;
            for (const auto& p : system.particles) {
                float sp = glm::length(p.velocity);
                sumV += sp;
                maxV = std::max(maxV, static_cast<double>(sp));
            }
            const float h = actualSpacing;
            const float avgV = static_cast<float>(sumV / std::max<size_t>(1, system.particles.size()));
            const bool still = avgV < kConvDispFrac * (h / dt) && maxV < kConvMaxFrac * (h / dt);
            convStableFrames = still ? convStableFrames + 1 : 0;
            if (convStableFrames >= kConvStableNeed) {
                paused = true;
                convStableFrames = 0;
                rebuildTopology();
            }
        }

        {
            auto _t = prof::Profiler::instance().scoped("evaluate");
            simMetrics = debug::evaluate(system, *activeSDF, actualSpacing);
            simMetricsValid = !system.particles.empty();
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

        // Linksklick: Voronoi-Zelle unter dem Cursor auswaehlen (Farblackzent).
        // Die Auswahl wird erst beim Loslassen ausgefuehrt - und nur dann, wenn
        // nicht zwischenzeitig gedragt wurde (Kamera-Rotation nutzt denselben
        // Linksklick). Klick ausserhalb des Meshes setzt die Auswahl zurueck
        // (-1). Die Off-Center-Verschiebung (viewShiftPx) wird kompensiert.
        const Vector2 mousePos = GetMousePosition();
        if (!ImGui::GetIO().WantCaptureMouse && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            clickDownMouse = mousePos;
            leftClickArmed = true;
        } else if (leftClickArmed && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            leftClickArmed = false;
            Vector2 delta = Vector2Subtract(mousePos, clickDownMouse);
            const bool wasClick = (delta.x * delta.x + delta.y * delta.y) < 25.0f;  // ~5px
            if (wasClick) {
                Vector2 pick = clickDownMouse;
                pick.x -= viewShiftPx;
                selectedCell = meshReady
                    ? renderer.pickSelectedCell(voronoiDual, GetMouseRay(pick, camera))
                    : -1;
                // Neuer Start-Zellindex: bestehender Pfad ist ungueltig.
                if (!pathCells.empty()) pathCells.clear();
            }
        }

        // Rechtsklick: kuerzesten Weg durch den Nachbargraph der Voronoi-Zellen
        // zum Ziel-Zellindex suchen und als Pfad hervorheben. Gleiche Klick-
        // erkennung wie links (Ausloesung beim Loslassen, kein Drag).
        if (!ImGui::GetIO().WantCaptureMouse && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            clickDownMouseR = mousePos;
            rightClickArmed = true;
        } else if (rightClickArmed && IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
            rightClickArmed = false;
            Vector2 delta = Vector2Subtract(mousePos, clickDownMouseR);
            const bool wasClick = (delta.x * delta.x + delta.y * delta.y) < 25.0f;  // ~5px
            if (wasClick && meshReady && selectedCell >= 0) {
                Vector2 pick = clickDownMouseR;
                pick.x -= viewShiftPx;
                const int target = renderer.pickSelectedCell(voronoiDual, GetMouseRay(pick, camera));
                pathCells.clear();
                if (target >= 0 && target != selectedCell) {
                    // Adjazenz direkt aus der Triangulation: Eine Kante (i,j)
                    // eines Dreiecks meint, dass die Zellen der Partikel i und j
                    // benachbart sind (Zellindex = Partikelindex bei separaten
                    // Zellverzeichnissen). Kantengewicht = raeumliche Distanz der
                    // Zentren; Dijkstra minimiert die Summe der Gewichte statt
                    // der Anzahl der Hopfen (BFS).
                    struct WEdge { int to; float w; };
                    const auto& parts = system.particles;
                    std::vector<std::vector<WEdge>> adj(parts.size());
                    for (const auto& t : system.triangles) {
                        const uint32_t e0[3] = { t.i0, t.i1, t.i2 };
                        for (int k = 0; k < 3; ++k) {
                            const uint32_t a = e0[k], b = e0[(k + 1) % 3];
                            const float w = glm::length(parts[a].position - parts[b].position);
                            adj[a].push_back({ static_cast<int>(b), w });
                        }
                    }
                    std::vector<float> dist(parts.size(), std::numeric_limits<float>::max());
                    std::vector<int> prev(parts.size(), -1);
                    // Min-Heap: (Distanz, Knoten).
                    using QItem = std::pair<float, int>;
                    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
                    dist[selectedCell] = 0.0f;
                    pq.push({ 0.0f, selectedCell });
                    while (!pq.empty()) {
                        const auto [d, u] = pq.top(); pq.pop();
                        if (d > dist[u]) continue;   // veralteter Heap-Eintrag
                        if (u == target) break;
                        for (const auto& e : adj[u]) {
                            const float nd = dist[u] + e.w;
                            if (nd < dist[e.to]) {
                                dist[e.to] = nd;
                                prev[e.to] = u;
                                pq.push({ nd, e.to });
                            }
                        }
                    }
                    if (dist[target] < std::numeric_limits<float>::max()) {
                        for (int c = target; c != -1; c = prev[c]) pathCells.push_back(c);
                        std::reverse(pathCells.begin(), pathCells.end());
                    }
                }
            }
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
        renderer.setCameraLighting(cameraLighting);
        renderer.setSmoothShading(smoothShading);
        renderer.setRoughTexture(roughTexture);
        renderer.setRoughness(roughness, roughFreq);
        renderer.setLightIntensities(lightKeyIntensity, lightFillIntensity);
        renderer.setAmbient(lightAmbient);

        if (meshReady && (showMesh || wireframe)) {
            meshPositions.resize(system.particles.size());
            for (size_t i = 0; i < system.particles.size(); ++i)
                meshPositions[i] = system.particles[i].position;
            renderer.syncMesh(meshPositions, system.triangles, topologyRevision);
            ++topologyAliveFrames;
        }
        if (meshReady && (showVoronoiFill || showVoronoiWire)) {
            renderer.syncVoronoiDual(voronoiDual, topologyRevision);
        }
        // Erst alle gefuellten Flaechen, dann beide Drahtgitter: so liegen die
        // spaeter gezeichneten Linien ueber allen Fill-Paessen (Z-Buffer &
        // Transparenz zeigen sonst Glitches durch gezeichnete Voronoi-Flaeche
        // ueber dem Triangulations-Wireframe).
        if (meshReady && showMesh) renderer.drawMeshFill(meshPositions);
        if (meshReady && showVoronoiFill) renderer.drawDualFill(voronoiDual);
        // Ausgewaehlte Zelle als Akzent-Flaechenpass: ueber der gefuellten
        // Dual-Flaeche, aber unter beiden Drahtgittern, damit die Zellgrenzen
        // lesbar bleiben.
        if (meshReady && selectedCell >= 0) renderer.drawSelectedCell(voronoiDual, selectedCell, topologyRevision);
        if (meshReady && wireframe) renderer.drawMeshWireframe(meshPositions, system.triangles);
        if (meshReady && showVoronoiWire) renderer.drawDualWireframe(voronoiDual);
        // Kuerzester Pfad: Linie durch die Zentren (Partikelpositionen) der
        // Pfad-Zellen, groessten Teils ueber allen anderen Paessen sichtbar.
        if (meshReady && pathCells.size() >= 2) {
            const auto& cells = voronoiDual.cells();
            std::vector<glm::vec3> pts, nrm;
            pts.reserve(pathCells.size());
            nrm.reserve(pathCells.size());
            bool valid = true;
            for (const int ci : pathCells) {
                if (ci < 0 || ci >= static_cast<int>(cells.size())) { valid = false; break; }
                const uint32_t p = cells[ci].particle;
                if (p >= system.particles.size()) { valid = false; break; }
                pts.push_back(system.particles[p].position);
                nrm.push_back(system.particles[p].normal);
            }
            if (valid) renderer.drawPathPolyline(pts, nrm);
        }
        if (showParticles) {
            if (showHeatmap) renderer.drawParticlesHeatmap(system, actualSpacing);
            else             renderer.drawParticles(system);
        }
        if (showSpatialGrid) renderer.drawSpatialGrid(system);
        if (showSDFProjection) renderer.drawSDFProjections(system);

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
            if (ImGui::Combo("Primitiv", &sdfShape, shapeNames, 8)) {
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
                case 7:
                    paramsChanged |= ImGui::SliderFloat("Major-Radius", &shapeRockTmajor, 0.3f, 2.0f);
                    paramsChanged |= ImGui::SliderFloat("Minor-Radius", &shapeRockTminor, 0.05f, 1.0f);
                    paramsChanged |= ImGui::SliderFloat("Rauheit", &shapeRockAmp, 0.0f, 0.5f);
                    paramsChanged |= ImGui::SliderFloat("Frequenz", &shapeRockFreq, 0.5f, 4.0f);
                    paramsChanged |= ImGui::SliderInt("Oktaven", &shapeRockOct, 1, 4);
                    paramsChanged |= ImGui::SliderInt("Seed", &shapeRockSeed, 0, 49);
                    ImGui::TextDisabled("Fels-Torus: fbm-displaced Torus.\nDie Lochmitte bleibt offen (Amplitude begrenzt).");
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
            ImGui::Checkbox("Automatisches Simulationsende", &autoConverge);
            if (autoConverge) {
                ImGui::SameLine();
                if (paused)
                    ImGui::TextDisabled("Ende");
                else
                    ImGui::TextDisabled("%d/%d Stillstand", convStableFrames, kConvStableNeed);
            }
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
            ImGui::Checkbox("Grid (besetzte Zellen)", &showSpatialGrid);
            ImGui::Checkbox("SDF-Projektion", &showSDFProjection);
            ImGui::Checkbox("Mesh anzeigen", &showMesh);
            ImGui::Checkbox("Wireframe", &wireframe);
            ImGui::Checkbox("Voronoi-Flaeche", &showVoronoiFill);
            ImGui::Checkbox("Voronoi-Kanten", &showVoronoiWire);
            ImGui::Checkbox("Beleuchtung", &enableLighting);
            if (enableLighting) {
                ImGui::SliderFloat("Key-Licht", &lightKeyIntensity, 0.0f, 2.0f);
                ImGui::SliderFloat("Fill-Licht", &lightFillIntensity, 0.0f, 2.0f);
                ImGui::SliderFloat("Ambient", &lightAmbient, 0.0f, 0.5f);
                ImGui::Checkbox("Kamera-feste Beleuchtung", &cameraLighting);
                ImGui::Checkbox("Weiche Beleuchtung", &smoothShading);
                if (smoothShading) {
                    ImGui::Checkbox("Raue Fraktal-Textur", &roughTexture);
                    if (roughTexture) {
                        ImGui::SliderFloat("Rauigkeit", &roughness, 0.0f, 1.0f);
                        ImGui::SliderFloat("Fraktal-Frequenz", &roughFreq, 1.0f, 40.0f);
                    }
                }
            }
            ImGui::Separator();
            ImGui::SliderFloat("View-Versatz", &viewShiftPx, 0.0f, 400.0f);
        }

        if (ImGui::CollapsingHeader("Stats", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (meshReady) {
                ImGui::Text("Dreiecke: %d  (degen: %d, orient: %d)", triStats.totalTriangles, triStats.degenerate, triStats.wrongOrientation);
                ImGui::Text("Kanten: abgelehnt (laenge %d, manifold %d)",
                    triStats.rejectedLength, triStats.rejectedManifold);
                ImGui::Text("Topologie: persistent seit %lld Frames", topologyAliveFrames);
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
