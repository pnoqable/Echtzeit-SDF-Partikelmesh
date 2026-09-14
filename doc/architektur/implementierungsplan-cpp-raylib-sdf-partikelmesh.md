# Implementierungsplan: C++20, raylib und Live-Debugging für SDF-Partikel-Meshes

## Ziel

Dieser Plan beschreibt eine verständliche und gleichzeitig leistungsfähige technische Basis für den Echtzeit-Prototypen:

- **C++20** für Simulation und Datenstrukturen;
- **raylib** für Fenster, Kamera, Eingabe und 3D-Rendering;
- **GLM** für Vektor- und Matrixmathematik;
- **Dear ImGui** für Parameter, Metriken und Debug-Ansichten.

Die erste Version arbeitet vollständig auf der CPU. Sie visualisiert die Partikelrelaxation, die SDF-Projektion und das persistente Triangle Mesh live. Eine GPU-Umsetzung wird erst nach belastbarer CPU-Messung ergänzt.

---

## 1. Technische Entscheidungen

| Baustein | Wahl | Aufgabe |
|---|---|---|
| Sprache | C++20 | Datenorientierte Simulation, Speicher- und Laufzeitkontrolle |
| Rendering | raylib | Fenster, Kamera, Mesh, Linien, Eingabe |
| Mathematik | GLM | `vec3`, Matrizen, Skalarprodukte, Kreuzprodukte |
| Debug-UI | Dear ImGui | Live-Regler, Pausieren, Einzelschritt, Metrikdiagramme |
| Build | CMake | reproduzierbare Konfiguration und Abhängigkeiten |
| Tests | Catch2 oder doctest | SDF-, Projektion- und Topologie-Tests |

### Warum diese Kombination?

Sie trennt die Fachlogik sauber von Darstellung und UI. raylib bleibt eine kleine Darstellungsschicht statt einer großen Engine; die Algorithmen für SDF, Grid, Repulsion und Triangulation sind dadurch unmittelbar lesbar und unabhängig testbar. Gleichzeitig kann der Renderer später durch einen GPU-Pfad ergänzt werden, ohne die Simulation fachlich neu zu entwerfen.

---

## 2. Projektstruktur

```text
sdf-particle-mesh/
├── CMakeLists.txt
├── external/                  # Paketmanager oder Git-Submodule, keine Fachlogik
├── src/
│   ├── main.cpp
│   ├── app/
│   │   ├── Application.hpp
│   │   └── Application.cpp
│   ├── simulation/
│   │   ├── SDF.hpp
│   │   ├── PrimitiveSDF.cpp
│   │   ├── ParticleSystem.hpp
│   │   ├── ParticleSystem.cpp
│   │   ├── SpatialHash.hpp
│   │   └── SpatialHash.cpp
│   ├── mesh/
│   │   ├── Triangulation.hpp
│   │   ├── Triangulation.cpp
│   │   ├── MeshTopology.hpp
│   │   └── MeshValidation.cpp
│   ├── render/
│   │   ├── SceneRenderer.hpp
│   │   ├── SceneRenderer.cpp
│   │   └── DebugDraw.hpp
│   ├── debug/
│   │   ├── ControlPanel.cpp
│   │   ├── Metrics.hpp
│   │   └── Metrics.cpp
│   └── util/
│       ├── Random.hpp
│       └── Timer.hpp
├── tests/
└── assets/
```

Die Abhängigkeitsrichtung bleibt bewusst einseitig:

```text
simulation ───► mesh
     │             │
     └────► debug ◄┘
                 │
                 ▼
              render
                 │
                 ▼
                app
```

`simulation/` kennt weder raylib noch ImGui. Dadurch können SDF-Projektion, Grid und Kräfte ohne Grafikkontext getestet werden.

---

## 3. Kern-Datenlayout

Für den ersten Prototypen sind einfache, zusammenhängende Arrays vorzuziehen. Statt einer Klasse pro Partikel werden parallel oder kompakt gespeicherte Daten verwendet.

```cpp
struct Particle {
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 normal;
    uint32_t  id;
};

struct Triangle {
    uint32_t i0, i1, i2;
};

struct SimulationParameters {
    float targetSpacing;
    float repulsionRadius;
    float repulsionStrength;
    float damping;
    float maxStepLength;
    float sdfTolerance;
    int   substeps;
    int   projectionIterations;
};
```

```cpp
class ParticleSystem {
public:
    std::vector<Particle> particles;
    std::vector<Triangle> triangles; // nach Initialisierung unverändert
    SimulationParameters parameters;

    void initialize(uint32_t count, const Bounds& bounds, uint32_t seed);
    void relax(float dt);
    void buildInitialTopology();
};
```

Die Partikel-ID ist zugleich Vertex-ID. Dadurch besitzt das Mesh keinen unabhängigen Positionszustand: Die Renderposition von Vertex `i` ist immer `particles[i].position`.

---

## 4. Ablauf eines Frames

```text
Eingabe / ImGui
      │
      ▼
Simulation: 0 bis N Substeps
      ├── Spatial Hash bauen
      ├── lokale Nachbarn suchen
      ├── tangentiale Repulsion berechnen
      ├── integrieren
      └── auf das SDF projizieren
      │
      ▼
Metriken aktualisieren
      │
      ▼
Rendern: Partikel, Mesh, Debug-Geometrie, UI
```

Pseudocode:

```cpp
while (!WindowShouldClose()) {
    float dt = GetFrameTime();

    controls.update(params);
    if (controls.resetRequested())
        system.initialize(params.particleCount, bounds, params.seed);

    if (controls.rebuildTopologyRequested())
        system.buildInitialTopology();

    if (!controls.paused()) {
        int steps = controls.singleStepRequested() ? 1 : params.substeps;
        for (int i = 0; i < steps; ++i)
            system.relax(dt / std::max(1, steps));
    }

    metrics.update(system);

    BeginDrawing();
    renderer.draw(system, metrics, debugSettings);
    controls.draw(params, metrics, debugSettings);
    EndDrawing();
}
```

Bei der initialen Relaxation bleibt die Topologie leer. Der Button **„Triangulation erzeugen“** bleibt deaktiviert, bis der Konvergenzindikator ausreichend gut ist. Danach bleibt der Indexbuffer persistent, bis ein bewusster Reset oder Neubau ausgelöst wird.

---

## 5. Entwicklungsphase 1: Projektgerüst und Kamera

### Aufgaben

1. CMake-Projekt erstellen und raylib, GLM, Dear ImGui einbinden.
2. Fenster, FPS-Anzeige und frei steuerbare 3D-Kamera einrichten.
3. Eine einfache Kugel-SDF, eine Bounding Box und die Weltachsen rendern.
4. Kamera-Reset und Screenshot-Funktion als kleine Qualitäts-of-Life-Funktionen ergänzen.

### Abnahme

- Anwendung startet ohne weitere Laufzeitabhängigkeiten.
- Kugel, Achsen und Bounding Box sind räumlich nachvollziehbar sichtbar.
- Kamera kann rotieren, verschoben und zurückgesetzt werden.

---

## 6. Entwicklungsphase 2: Partikel und SDF-Projektion

### Aufgaben

1. Reproduzierbare Zufallsinitialisierung über einen expliziten Seed implementieren.
2. Partikel zunächst als kleine, einfache Kugeln oder Punkte instanziiert rendern.
3. `sampleSDF(position)` mit Distanz und Gradient implementieren.
4. Newton-artige Projektion auf `φ(p) = 0` implementieren.
5. Partikel nach der Projektion anhand von `abs(φ(p))` einfärben.

### Debug-Ansicht

- Vorher-/Nachher-Projektion als kurzlebige Linie darstellen.
- Normale des ausgewählten Partikels als Pfeil zeigen.
- Punkte mit zu großem SDF-Fehler deutlich markieren.

### Abnahme

- Alle Partikel liegen nach dem Projektionsschritt innerhalb der SDF-Toleranz auf der Oberfläche.
- Ein Auswahlklick zeigt Partikel-ID, Position, `φ(p)` und Normalenvektor.

---

## 7. Entwicklungsphase 3: Spatial Hash und Nachbarn

### Aufgaben

1. `cellSize ≈ targetSpacing` verwenden.
2. Partikel pro Frame einer ganzzahligen 3D-Zelle zuordnen.
3. Nur die eigene und angrenzende Zellen für Nachbarkandidaten durchsuchen.
4. Doppelte Paarverarbeitung verhindern.

### Debug-Ansicht

Beim selektierten Partikel anzeigen:

- seine Grid-Zelle;
- die 26 angrenzenden Zellen;
- tatsächlich gefundene Nachbarn als Linien;
- Distanz und Nachbaranzahl im UI.

### Abnahme

- Der Grid-Pfad liefert im Referenztest dieselben Nachbarn wie eine langsame O(N²)-Vergleichssuche.
- Die gemessene Suchzeit wächst für gleichmäßige Verteilung näherungsweise linear mit der Partikelzahl.

---

## 8. Entwicklungsphase 4: Tangentiale Relaxation

### Aufgaben

1. Begrenzte, glatte Repulsionskraft innerhalb des Interaktionsradius implementieren.
2. Kraft in die Tangentialebene projizieren:

   ```cpp
   tangentForce = force - glm::dot(force, normal) * normal;
   ```

3. Gedämpfte Integration, Schrittlimit und SDF-Projektion pro Substep ergänzen.
4. Ruhe- und Konvergenzkriterium aus Geschwindigkeit und Nachbarabständen berechnen.

### Debug-Ansicht

- Kraftpfeil: gesamte Kraft und tangentialer Anteil verschieden darstellen.
- Bewegungs-Trail für den selektierten Partikel; optional eine kleine Stichprobe.
- Partikelfarbe nach nächstem Nachbarabstand:
  - Unterabstand: zu dicht;
  - Zielbereich: passend;
  - Überabstand: zu weit.
- Live-Histogramm der nächsten Nachbarabstände.

### Abnahme

- Auf der Kugel sinkt die Streuung der Nachbarabstände sichtbar.
- Der maximale SDF-Fehler bleibt klein.
- Das System bleibt bei mehreren Minuten Laufzeit stabil.

---

## 9. Entwicklungsphase 5: Initialtriangulation und Mesh-Rendering

### Aufgaben

1. Nach Konvergenz für jeden Partikel eine Tangentialbasis aufbauen.
2. Nahe Nachbarn in die lokale Ebene projizieren.
3. Kanten und Dreiecke aus einer lokalen Delaunay-orientierten Strategie erzeugen.
4. Kandidaten über Kantenlänge, Normalenwinkel und SDF-Test am Kantenmittelpunkt filtern.
5. Dreiecksorientierung mit SDF-Normalen vereinheitlichen.
6. Doppelte, degenerierte und nichtmanifold Elemente erkennen.

### Renderpfad

- Flächen: halbtransparent oder normal schattiert;
- Wireframe: immer optional darüber;
- Partikel: optional über dem Mesh, damit die Vertexbindung sichtbar bleibt;
- Vertexpositionen: pro Frame aus `particles[i].position` erzeugen bzw. direkt referenzieren;
- Indizes: nach erfolgreicher Triangulation nicht mehr ändern.

### Debug-Ansicht

- Dreiecksfarbe nach Minimalwinkel bzw. Aspect Ratio.
- Ungültige Dreiecke leuchtend hervorheben.
- Beim selektierten Partikel inzidente Dreiecke und Kanten zeichnen.
- Eine Ansicht „nur Mesh“, „nur Partikel“ und „Overlay“ anbieten.

### Abnahme

- Ein geschlossenes Kugelmesh ohne degenerierte und nichtmanifold Kanten.
- Der Indexbuffer bleibt beim normalen Lauf unverändert.
- Der Mesh-Vertex `i` folgt sichtbar und korrekt Partikel `i`.

---

## 10. Debug-UI: notwendige Regler und Anzeigen

### Steuerung

| Gruppe | Elemente |
|---|---|
| Simulation | Start/Pause, Einzelschritt, Reset, fester Seed |
| Partikel | Anzahl, Zielabstand, Substeps, maximale Schrittweite |
| Kräfte | Radius, Stärke, Dämpfung |
| SDF | Primitive, Formparameter, Projektionen, Toleranz |
| Topologie | Triangulation erzeugen, Wireframe, Quality-Threshold |
| Ansicht | Partikel, Mesh, Grid, Normalen, Kräfte, Trails, Heatmap |

### Permanente Metriken

| Metrik | Bedeutung |
|---|---|
| Framezeit / FPS | Gesamtbudget und sichtbare Einbrüche |
| Grid-Zeit | Aufwand der Nachbarsuche |
| Simulationszeit | Kräfte, Integration, Projektion |
| `max / avg |φ(p)|` | Qualität der SDF-Bindung |
| Abstand: Min / Mittel / Max / StdAbw. | Qualität der Verteilung |
| schlechte Dreiecke | Topologischer Gesundheitszustand |
| min. Innenwinkel / max. Aspect Ratio | Geometrische Mesh-Qualität |

Die UI sollte eine „sichere“ Schaltfläche **Reset auf Referenzparameter** enthalten. So lassen sich Experimente jederzeit mit identischer Ausgangslage wiederholen.

---

## 11. Testablauf

1. **Kugel, 1.000 Partikel:** Algorithmus nachvollziehen und Debug-Ansichten überprüfen.
2. **Kugel, 10.000 Partikel:** Grid- und Framezeiten messen.
3. **Ellipsoid:** wechselnde Krümmung validieren.
4. **Torus:** lokale Nachbarschaft und nichttriviale Topologie testen.
5. **Konkave SDF:** falsche Querverbindungen erkennen und Filter schärfen.
6. **Langzeittest:** 10 s, 60 s und 5 min mit Metrik-Log; Indexbuffer prüfen.

Für jeden Test werden Parameter, Seed, mittlere Framezeit und Qualitätsmetriken gespeichert. Screenshots oder kurze Aufzeichnungen derselben Kameraansicht erleichtern Vergleiche nach Änderungen.

---

## 12. Option: Edge-Flips als lokaler Reparaturpfad

Diese Phase beginnt erst, wenn die persistente Topologie tatsächlich Qualitätsverlust zeigt.

### Aufgaben

1. Statischen Indexbuffer durch Half-Edge-Topologie ergänzen.
2. Schlechte Dreiecke in eine begrenzte Arbeitsliste eintragen.
3. Nur die Gegenkanten benachbarter Dreiecke prüfen.
4. Flip nur anwenden, wenn der Qualitätswert klar steigt und die lokale Orientierung korrekt bleibt.
5. Geänderte Nachbarschaft im Debug-Overlay animiert markieren.

Ein per-Frame-Budget verhindert, dass eine große Korrektur die Renderzeit dominiert. Edge-Splits und -Collapses bleiben weiterhin außerhalb des ersten Ausbaus.

---

## 13. Weg zur GPU-Version

Eine GPU-Umsetzung ist erst gerechtfertigt, wenn die CPU-Profilierung zeigt, dass Grid-Aufbau oder Kraftberechnung das relevante Bottleneck sind.

### Reihenfolge

1. CPU-Referenzpfad behalten und Metriken vergleichen.
2. Partikelpositionen, Geschwindigkeiten und Normalen in zusammenhängende GPU-Buffer überführen.
3. Erst Repulsion und Integration als Compute-Pass portieren.
4. Danach Grid-Aufbau und Nachbarlisten portieren.
5. Vertexshader direkt aus dem Partikelpositionsbuffer lesen lassen.
6. Debugdaten nur für selektierte Partikel oder Stichproben zurücklesen; kein Voll-Readback pro Frame.

### Technische Optionen

- **OpenGL Compute Shader:** geringste zusätzliche Komplexität, wenn raylib-/OpenGL-Kontext genügt.
- **Vulkan oder bgfx:** später sinnvoll für explizitere Synchronisierung und mehrere Grafik-Backends, aber kein Startpunkt für den Algorithmusprototyp.

Die Datenidentität bleibt unverändert: Auch auf der GPU ist der Partikelbuffer die einzige Positionsquelle; Mesh-Indizes bleiben persistent.

---

## 14. Meilensteine

| Meilenstein | Ergebnis |
|---|---|
| M1 | Kamera, Kugel-SDF, projizierte Partikel |
| M2 | Grid, selektierbare Nachbarschaft, stabile Relaxation |
| M3 | Live-Metriken und nachvollziehbare Debug-Overlays |
| M4 | einmaliges, validiertes Kugelmesh |
| M5 | persistente Topologie im Langzeittest |
| M6 | Torus und konkave Testform mit dokumentierten Grenzen |
| M7 | optional: budgetierte Edge-Flips |
| M8 | optional: GPU-Compute-Pfad mit CPU-Referenzvergleich |

---

## 15. Statusübersicht (Stand 2026-09-14)

Die folgenden Tabellen dokumentieren den Ist-Stand des CPU-Prototypen. Offene Punkte aus den Abnahme-Kriterien sind nur aufgenommen, wenn sie sachlich relevant und nicht rein kosmetisch sind.

### Umsetzungsstand nach Phase

| Phase | Status | Details und offene Punkte |
|---|---|---|
| 1 Projektgerüst und Kamera | ✅ umgesetzt | CMake + FetchContent (raylib 5.5, GLM 1.0.1, ImGui 1.92.7, rlImGui), Debug-Presets (debug/asan, Ninja), VSCode-Launch mit `preLaunchTask`; Fenster resizable + F11; Kugel-SDF, Bounding Box, Achsen; Kamera via Maus (Y-invertiert), WASD/Pfeiltasten, Scroll/Zoom |
| 2 Partikel und SDF-Projektion | ◐ weitgehend | Reproduzierbarer Seed, Partikel als kleine Kugeln (`DrawSphereEx` 4×4), Newton-Projektion auf `φ=0`; `max/avg |φ(p)|` in Metriken umgesetzt, farbige Markierung nach `abs(φ(p))` (Plan-Abnahme) optional; **exakter Fusspunkt fuer Ellipsoide** (normierte Distanz statt Blinn-Naeherung, `e236314`) |
| 3 Spatial Hash und Nachbarn | ✅ umgesetzt | `cellSize = repulsionRadius`, 27 Nachbarzellen, deduplizierte Paare `j>i`; Paaranzahl im UI, min/avg/max Abstand + StdAbw. als Metrik; Referenz-Verifikation gegen O(N²)-Suche (`tests/test_spatialhash_reference.cpp`), Nachbarlinien-Overlay für selektierten Partikel; **vollständiges Grid-Overlay aller belegten Zellen** (`SceneRenderer::drawSpatialGrid`, Toggle „Grid (alle Zellen)“) |
| 4 Tangentiale Relaxation | ◐ weitgehend | Tangentiale Repulsion `f(d)=k(1-d/R)²/d`, gedämpfte Integration, Verschiebungs-Clamp (`maxStepLength·h`), SDF-Projektion pro Substep; Einzelschritt, Partikel-Abstands-Heatmap, Live-Abstands-Histogramm, Kraftpfeil (gesamt=gelb, tangential=magenta) für selektierten Partikel, Bewegungs-Trail, `avg/max v` als Konvergenz-Indikator; **Vorher-/Nachher-Projektionslinien umgesetzt** (`SceneRenderer::drawSDFProjections`, Cyan, Toggle „SDF-Projektion“); Konvergenzkriterium aus Abstands-Streuung und Trails für Stichprobe fehlen |
| 5 Initialtriangulation und Mesh-Rendering | ✅ Abnahme erfüllt | Fan-Triangulation (Tangentialprojektion + `atan2`-Sortierung), Kantenfilter (Länge, Normalenwinkel, SDF-Midpoint), Orientierungsvereinheitlichung, Dedup + Manifold-Prüfung (Kante max. 2×), finale Validierung, abschließender **Boundary-Loop-Fill** (schließt 3-/4-er Randkanten-Loops); **Ueberarbeitung** (`6f6482c`): Normalen-/Midpoint-Vorfilter entfernt, Ear-Clipping-Fehlerbehandlung statt Vorfilter fuer grosse Loops; Mesh flächengefüllt + Wireframe-overlay + Partikel-Toggle, **Mesh-Qualitäts-Overlay** (Dreiecksfarbe nach Minimalwinkel, Poor rot, Schwellwert einstellbar), Vertices folgen pro Frame `particles[i].position`. Spacing-Formel `sqrt(area/N)`. Nach 60 s Relaxation: `F=2V−4`, `χ=2`, **0 Randkanten** über 10 Seeds (App-Defaults) |

### Debug-UI

Umgesetzt: Pause/Weiter, **Einzelschritt**, Reset, „Triangulation erzeugen", **Auto-Rebuild (Debug)** mit Frames-Intervall (trianguliert während der laufenden Simulation periodisch neu; unterbricht bewusst die persistente Topologie), FPS, Partikelzahl, Paaranzahl, Dreiecks-Stats; Toggles Mesh/Wireframe/Partikel/**Partikel-Heatmap**/**Mesh-Qualität**/Achsen/Bounding Box/**Grid (alle Zellen)**/**SDF-Projektion**; Slider für Max Kantenlänge, **Poor-Winkel (Quality-Threshold)**, Repulsion, Stärke, Dämpfung, Substeps, MaxSchritt (Breite begrenzt, damit Labels passen), **Parameter zurücksetzen** bei den Reglern; **Off-Center-Ansicht** (`3a61828`): 3D-Hauptansicht wird um einstellbare Pixel nach rechts verschoben (schraeges Frustum ueber `m8` der Projektionsmatrix, tiefenkonstant), Slider „View-Versatz (px)" 0–400; Rechtsklick-Raycast kompensiert den Versatz. Permanente Metriken (`src/debug/Metrics`): Sim/Grid-Zeit, `max/avg |φ(p)|`, Abstand min/avg/max/StdAbw + unter/ok/über `h`, `avg/max v`, Mesh-Qualität (min. Innenwinkel, max. Aspect Ratio, „poor“-Zähler), **Live-Histogramm der Nachbarabstände**, **Topologie-Persistenz** („persistent seit N Frames“). **Partikel-Auswahl per Rechtsklick**: ID, Position, `φ(p)`, Nachbaranzahl; Overlays Grid-Zelle, Nachbarlinien, Kraftpfeil (gesamt/tangential), Normale, Bewegungs-Trail.

Fehlend aus Plan-Abschnitt 10: Konvergenzkriterium aus Abstands-Streuung und Trails für eine Stichprobe; M7 Edge-Flips (nach M5-Messung voraussichtlich nicht nötig).

### Strukturelle Abweichungen vom Plan

| Plan | Ist |
|---|---|
| `app/`, `debug/` (ControlPanel, Metrics), `util/` | nicht vorhanden; UI direkt in `main.cpp`, kein eigenes Metrics-Modul, kein Timer/Random-Helper |
| `system.buildInitialTopology()` in `ParticleSystem` | als eigenständiges Modul `mesh/Triangulation` umgesetzt; Topologie lebt in `ParticleSystem::triangles` (persistent, Partikel-ID = Vertex-ID) |
| `src/platform/` | neu hinzugekommen (nicht im Plan): `SystemTheme` für OS-Dark-Mode-Erkennung (macOS CFPreferences, Windows Registry) |
| Tests (Plan: Catch2/doctest) | `tests/test_triangulation.cpp` (Euler-Test, Fibonacci-Sphäre), `tests/test_spacing_regression.cpp`, `tests/test_closed_mesh.cpp` (10 Seeds geschlossen, `F=2V−4`), `tests/test_metrics.cpp` (Verteilungs-/SDF-/Mesh-Metriken plausibel), `tests/test_spatialhash_reference.cpp` (Grid-Pfad == O(N²)-Referenz), `tests/test_persistent_topology.cpp` (M5: persistenter Indexbuffer über +10 s/+60 s/+5 min, Vertex-Drift-Monitor), `tests/test_sdf_primitives.cpp` (M6 Primitiva: Kugel/Ellipsoid/Torus/Hantel, Fächentreue, `tests/test_closed_shapes.cpp` (M6/Metaball: triangulierte Formen geschlossen/bewertet); kein Test-Framework eingebunden |

### Meilensteine

| Meilenstein | Status | Anmerkung |
|---|---|---|
| M1 | ✅ | Kamera, Kugel-SDF, projizierte Partikel |
| M2 | ✅ | Grid, Nachbarschaft, stabile Relaxation (Kern; Debug-Overlays fehlen) |
| M3 | ✅ | Panel-Metriken, Einzelschritt, Partikel-Heatmap, Live-Histogramm, Mesh-Qualitäts-Overlay, Partikel-Auswahl mit Overlays (Grid-Zelle, Nachbarn, Kräfte, Normale, Trail), Ansichten-Combo, Referenz-Test Grid vs. O(N²); vollständiges Grid-Overlay aller Zellen + Vorher-/Nachher-Projektionslinien als Toggles |
| M4 | ✅ | Kugelmesh erzeugbar und nach Relaxation geschlossen (Spacing-Korrektur `sqrt(A/N)`) |
| M5 | ✅ | Persistente Topologie im Langzeittest: Indexbuffer bleibt während des normalen Laufs bit-identisch, nur Vertexpositionen werden pro Frame aktualisiert (`renderer.drawMesh` → `updateMeshVertices` bei unveränderter Revision); UI zeigt „Topologie: persistent seit N Frames“; neuer Langzeittest `tests/test_persistent_topology.cpp` (60 s Vorkonvergenz, dann +10 s/+60 s/+5 min): `F=1996` unverändert, Vertex-Drift ≤ 0.03·h, Qualität stabil (minWinkel 35°, poor 0) – persistente Topologie trägt, Edge-Flips (M7) nicht nötig |
| M6 | ✅ | Torus-, Hantel- und **Metaball-SDF** (Smooth-Min, Quílez-Formel, analytischer Gradient via Kettenregel) verfügbar via Debug-UI (SDF-Form-Combo: Kugel/Ellipsoid/Torus/Hantel/Metaball, Formparameter-Slider); `surfaceArea()` im SDF-Interface für generisches Spacing `sqrt(A/N)`, `computeProfile()` (Rotationsintegral als Radiusverlauf + Bounds); Metaball-Ueberlappung darf `>= R` sein (getrennte Blobs, Gap-Erkennung im Profil); `tests/test_sdf_primitives.cpp` PASS (Flächen exakt bis 1,2 %, inkl. Metaball k→0 = Hantel); `tests/test_closed_shapes.cpp` PASS: Kugel exakt F=2V−4 (Rand=0), Metaball k=0.8 delta −1 F=2V−4 (Rand=3); **dokumentierte Grenzen**: Torus F=2V−1 (Rand=5), Hantel F=(2V−4)−1 (Rand=5) — Boundary-Loop-Fill schließt nur 3er/4er-Loops; konkave/negativ gekrümmte Stellen bleiben mit ≤5 Randkanten offen (Hantel minWinkel 3,5°, poor=8 — geometrische Grenze der Delaunay-Filterung); Metaball k=2.5 erzeugt Randkanten an der konkaven Taille (dokumentierte Grenze der Fan-Triangulation) |
| M7 | — | offen (Edge-Flips nach M5-Messung voraussichtlich nicht nötig) |

### Bekannte Einschränkung: Mesh-Lücken nach Relaxation

**Gelöst (2026-09-12):** Zwei Ursachen:
1. **Spacing-Formel:** Die App nutzte `sqrt(2A/(√3·N))` (Hexagon-Ringabstand) statt `sqrt(A/N)` (mittlere Punktdichte). Mit korrekter Formel und `maxEdgeLength=1.4` sanken die Randkanten drastisch.
2. **Loch-Schließung:** Die verbleibenden kleinen Lücken (1–2 fehlende Dreiecke, als spitze Löcher sichtbar) entstanden durch die Manifold-Rejection konkurrierender Fan-Kandidaten. Ein abschließender **Boundary-Loop-Fill** in `Triangulation::closeBoundaryLoops()` verdrahtet Randkanten zu Loops und füllt 3- und 4-er Loops gefiltert (F = 2V−4, `χ=2`).

Ergebnis mit den echten App-Defaults (seed 42–31415, 60 s Relaxation, 1000 Partikel): **0 Randkanten in 10/10 Seeds**, jeder Test exakt `F=1996`. Regressionstest: `tests/test_spacing_regression.cpp`.

---

## Abschlusskriterium

Der erste Prototyp ist erfolgreich, wenn er auf einer Kugel und einem Torus reproduzierbar zeigt:

1. zufällige Partikel konvergieren zu einer gleichmäßigen Verteilung auf `φ = 0`;
2. eine initiale Triangulation ein gültiges Mesh erzeugt;
3. der Indexbuffer über lange Laufzeiten unverändert bleibt;
4. ausschließlich Vertexpositionen aktualisiert werden;
5. die Debug-Ansicht jede relevante Entscheidung – Nachbarn, Kräfte, Projektion und Triangle-Qualität – sichtbar macht;
6. die Messwerte eindeutig zeigen, ob persistente Topologie ausreicht oder Edge-Flips nötig werden.
