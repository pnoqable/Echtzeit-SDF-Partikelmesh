# Performanztest unter Windows: ThreadPool-Chunking & Sim-Pipeline

## Ziel

Den auf dem Mac (Apple Silicon, M3 Pro) durchgefuehrten Performanzvergleich auf
einem Windows-System reproduzieren und dokumentieren. Zwei Ebenen:

1. **ThreadPool-Mikrobenchmark** (`bench/bench_threadpool.cpp`) — vergleicht die
   Index-Zuteilung des ThreadPools: Einzel-Index (`chunk=1`, altes Verhalten)
   gegen feste Chunks und adaptives Chunking (Default seit `72173d1`).
2. **Sim-Pipeline-Profil** (`tests/bench_profile.cpp`) — headless Messung der
   Stage-Zeiten eines Relax-Frames (grid / forces / integrate / project) und der
   einmaligen Kosten (evaluate, triangulate).

Damit wird insbesondere geprueft, ob die Optimierungen aus der Windows-FPS-Beobachtung
(Optimum bei Repulsionsradius 0.08, Abfall darunter/ darueber) die erwartete Wirkung
zeigen: feinkoernige Lasten (`light`, `integrate`) sollen ~20x schneller sein,
die Kraftschleife (`force`) ~2.5–3x, der SpatialHash-Build bleibt nahezu unveraendert.

---

## 1. Voraussetzungen

- Windows 10/11 (x86-64), Zielsystem z. B. AMD Ryzen 7 8845HS (8 C/16 T)
- **CMake ≥ 3.20** und ein Generator mit C++20:
  - Visual Studio 2022 (MSVC, Generator `Visual Studio 17 2022`), oder
  - Ninja + Clang/GCC (Generator `Ninja`), oder
  - ein C++-Compiler fuer `bench_profile.cpp` (GCC/Clang mit `-DSDFPROFILING=1`
    oder MSVC mit `/DSDFPROFILING=1`)
- Git-Befehle in PowerShell/Konsole ausfuehrbar

Hinweis: Der Pool erzeugt `hardware_concurrency() - 1` Worker-Threads plus den
Haupt-Thread. Auf dem 8845HS sind das 16 Teilnehmer (SMT an), auf dem M3 Pro 10.
Die relative Betrachtung (Speedup gegenuber `chunk=1`) ist dadurch direkt
vergleichbar; absolute Zeiten NICHT (verschiedene Kerne/Takt).

---

## 2. Build

### 2.1 Benchmark `bench_threadpool` (Option `SDFBENCH=ON`)

```powershell
# Aus dem Repo-Wurzelverzeichnis:
cmake -S . -B build/windows -DSDFBENCH=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/windows --config Release --target bench_threadpool
```

Ausfuehrbare Datei:

```text
build/windows/Release/bench_threadpool.exe   # MSVC/VS-Generator
build/windows/bench_threadpool               # Ninja/single-config
```

### 2.2 Pipeline-Profil `bench_profile`

Das Profil ist bewusst headless und nicht ins CMake-Build eingebunden
(selbst kompilieren, vgl. Kopfkommentar in `tests/bench_profile.cpp`):

```powershell
# MinGW (GCC):  -pthread ist bei MinGW-w64 enthalten
g++ -std=c++20 -O2 -DSDFPROFILING=1 -I build/windows/_deps/glm-src -I src `
    src/core/ThreadPool.cpp src/simulation/PrimitiveSDF.cpp `
    src/simulation/SpatialHash.cpp src/simulation/ParticleSystem.cpp `
    src/mesh/Triangulation.cpp src/debug/Metrics.cpp `
    tests/bench_profile.cpp -o bench_profile.exe -lm

# MSVC (Developer PowerShell):
cl /std:c++20 /O2 /EHsc /DSDFPROFILING=1 /I build/windows/_deps/glm-src /I src `
   src/core/ThreadPool.cpp src/simulation/PrimitiveSDF.cpp src/simulation/SpatialHash.cpp `
   src/simulation/ParticleSystem.cpp src/mesh/Triangulation.cpp src/debug/Metrics.cpp `
   tests/bench_profile.cpp /Fe:bench_profile.exe
```

---

## 3. Messprotokoll

### 3.1 ThreadPool-Mikrobenchmark

```powershell
.\build\windows\Release\bench_threadpool.exe | Tee-Object -FilePath .\bench-chunking-windows.txt
```

Mehrfach laufen lassen (3–5 unabhaengige Lauefe) und das **Minimum** je Konfiguration
nehmen; Zwischenwerte sind CPU-Takt/Frequenz-Scaling geschuldet.

Referenz (Mac, M3 Pro, 10 Worker, N = 20000 — Minima ueber mehrere Laeufe, als Erwartungsanker):

| Konfiguration | light | force | grid R=0.08 | grid R=0.05 |
|---|---|---|---|---|
| chunk = 1 (alt) | 1.452 ms | 1.958 ms | 25.71 ms | 14.87 ms |
| chunk = 128 | 0.068 ms | 0.949 ms | 23.39 ms | 14.41 ms |
| adaptiv d=8 (Default) | 0.074 ms | 0.942 ms | 24.33 ms | 13.80 ms |
| Speedup adaptiv vs. chunk=1 | **~20x** | **~2–3x** | ~1.0x | ~1.1x |

Windows-Tabelle zum Ausfuellen:

| Konfiguration | light | force | grid R=0.08 | grid R=0.05 |
|---|---|---|---|---|
| chunk = 1 (alt) | | | | |
| chunk = 128 | | | | |
| adaptiv d=8 (Default) | | | | |
| Speedup adaptiv vs. chunk=1 | | | | |

Erwartung: `light` deutlich schneller (Atomik-Overhead pro Index war dominant),
`force` ~2.5–3x, `grid` nahezu unveraendert (dort dominieren serielle Anteile).

### 3.2 Sim-Pipeline-Profil

```powershell
.\bench_profile.exe 20000 100 500   # N=20000, 100 Frames Warmup, 500 Mess-Frames
```

Liefert je Stage `mean/p95/min` in ms sowie die einmaligen Kosten (`evaluate`,
`triangulate`). Referenz (Mac) und Windows-Daten jeweils ablegen:

| Stage | Mac mean | Windows mean | Windows p95 |
|---|---|---|---|
| grid | | | |
| forces | | | |
| integrate | | | |
| project | | | |
| sim (Relax-Frame gesamt) | | | |

---

## 4. Rahmenbedingungen fuer belastbare Messung

- **Release-Build** (MSVC `/O2` oder CMake `Release`). Debug-Builds sind fuer den
  Vergleich wertlos (STL- und Inline-Unterschiede).
- **Energieprofil „Hoechstleistung"** bzw. deaktiviertes Frequenz-Scaling
  (`powercfg /setactive SCHEME_MIN` als Administrator) verkleinert die Streuung.
- Keine Hintergrundlast (Indexierung, Game-Overlays); idealerweise Messung aus
  einer PowerShell-Session ohne VS-Just-In-Time-Ablaeufe.
- Mehrere Lauefe, Minimum notieren (Minimum ist invers zum garantierten Takt am
  stabilsten; Mittelwerte sind durch Turst-Boost verfaelscht).
- Bei 8845HS (16 Threads) ist der Effekt der Chunk-Optimierung tendenziell
  groesser als auf dem Mac, da mehr Teilnehmer am `fetch_add` drum herum streiten.

---

## 5. Interpretation / Abnahmekriterien

- **Adaptiv d=8 auf Windows**: `light`-Speedup deutlich > 1 (Ziel: ~10–20x),
  `force` ~2.5–3x gegenuber `chunk=1`. `grid` darf um ±10 % schwanken (Rauschen).
- **Pipeline**: `forces` sollte je Frame messbar (ca. ×2–3) sinken, `grid` bleibt.
  Der beobachtete FPS-Einbruch *unterhalb* von R=0.08 entstammt dem Grid-Build
  (CSR-Umstellung `2281c12`), nicht der Zuteilung — darum im Verlauf der
  Repulsionsradius-Sweeps die Stage-Zeiten, nicht nur das FPS, loggen.
- **Regression**: P95-Streuung der Stages darf sich nicht erhoehen
  (Chunking darf kein Lastbalancing verschlechtern).