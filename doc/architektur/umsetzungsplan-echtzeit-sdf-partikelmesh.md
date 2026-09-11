# Umsetzungsplan: Echtzeit-Prototyp für SDF-gebundene Partikel und persistentes Triangle Mesh

## 1. Zielbild

Dieser Prototyp erzeugt aus einer konstant großen Menge von Partikeln auf der Oberfläche eines **analytischen, zeitlich invarianten Signed-Distance-Fields (SDF)** ein Triangle Mesh. Die Partikel werden zunächst zufällig in einer Bounding Box erzeugt, auf die SDF-Oberfläche projiziert und dort durch tangentiale Repulsion zu einer gleichmäßigen Verteilung relaxiert.

Nach dieser Initialisierungsphase wird **einmalig** eine Triangle-Topologie erzeugt. Zur Laufzeit ändern sich anschließend nur die Vertexpositionen; der Indexbuffer bleibt unverändert. Dadurch wird aus einem allgemeinen „Point Cloud → Mesh pro Frame“-Problem ein wesentlich günstigeres Problem: eine bewegliche, aber persistent diskretisierte Oberfläche.

Die zentrale Hypothese des Prototyps lautet:

> Eine gleichmäßig relaxierte, auf einem statischen SDF gebundene Partikelmenge kann mit einer einmal erzeugten Topologie über lange Zeit stabil als Mesh dargestellt werden. Lokale Edge-Flips reichen als spätere, optionale Korrektur aus.

Nicht Ziel der ersten Version sind globale Rekonstruktion pro Frame, variable Partikelzahl, dynamische SDF-Topologie oder vollständiges Remeshing.

---

## 2. Rahmenbedingungen und Annahmen

| Eigenschaft | Annahme |
|---|---|
| SDF | analytisch, statisch, mit Gradient verfügbar |
| Partikelzahl | konstant: `N` |
| Oberfläche | zusammenhängend; zu Beginn bevorzugt geschlossen |
| Partikel | bleiben durch Projektion nahe `φ(p) = 0` |
| Verteilung | durch Repulsion annähernd gleichmäßig |
| Mesh | ein Vertex pro Partikel, feste Identität und feste Indizes |
| Laufzeit | Partikeldynamik + Projektion + Positionsupdate; keine globale Triangulation |

Diese Einschränkungen sind bewusst. Sie erlauben, die schwierige globale Topologiebestimmung aus dem Hot Path zu entfernen.

---

## 3. Architektur

```text
                         statisches SDF φ(p), ∇φ(p)
                                   │
              ┌────────────────────┴────────────────────┐
              │                                         │
              ▼                                         ▼
    Projektion und Normalen                    einmalige Initialtriangulation
              │                                         │
              ▼                                         ▼
     Partikelpositionen P(t) ───────────────► Indexbuffer T
              │                                         │
              └─────────────────┬───────────────────────┘
                                ▼
                    persistentes Triangle Mesh
                    Positionen dynamisch, Indizes fest
```

Es gibt genau eine maßgebliche Positionsrepräsentation: den Partikelbuffer. Das Mesh liest diese Positionen direkt oder erhält sie per linearem Update. Eine separate, driftende Vertexsimulation wird vermieden.

---

## 4. Datenmodell

### 4.1 SDF-Schnittstelle

```cpp
struct SDFSample {
    float distance;   // φ(p), negativ innen, positiv außen
    Vec3  gradient;   // ∇φ(p)
};

SDFSample sampleSDF(Vec3 p);
```

Für den Start werden drei primitive SDFs implementiert:

- Kugel: Referenzfall mit konstanter Krümmung
- Achsenausgerichtete Box mit abgerundeten Kanten oder Ellipsoid: wechselnde Krümmung
- Torus: nichttriviale Genus-1-Topologie

Später folgen konkave Kompositionen (z. B. Differenz zweier SDFs). Für alle Formen muss der Gradient zuverlässig sein; bei CSG-Übergängen kann eine numerische Gradientenschätzung als Diagnose dienen.

### 4.2 Partikel

```cpp
struct Particle {
    Vec3 position;
    Vec3 velocity;
    Vec3 normal;       // normalisierter SDF-Gradient
    uint32_t id;
};
```

### 4.3 Mesh und Nachbarschaft

```cpp
struct Triangle { uint32_t i0, i1, i2; };

struct Mesh {
    std::vector<Triangle> triangles;  // nach Initialisierung persistent
};
```

Für die optionale Reparaturphase wird eine Half-Edge- oder Winged-Edge-Struktur ergänzt. Sie erlaubt benachbarte Dreiecke, Gegenkanten und Edge-Flips in konstanter lokaler Zeit. Für die erste, rein persistente Fassung reicht ein statischer Indexbuffer.

---

## 5. Zielabstand und Skalierung

Die gewünschte mittlere Kantenlänge bzw. der Partikelabstand `h` bestimmt alle räumlichen Parameter. Bei geschätzter Oberfläche `A` und `N` Partikeln eignet sich als Startwert:

```text
h ≈ sqrt(A / N)
```

Der Faktor hängt von der gewünschten hexagonal ähnlichen Packung ab; wichtig ist zunächst eine konsistente Skalierung, nicht der perfekte Vorfaktor. Für bekannte Formen kann `A` analytisch vorgegeben werden. Beispielsweise gilt für einen Torus mit großem Radius `R` und kleinem Radius `r`:

```text
A = 4π²Rr
```

Wenn `A` nicht analytisch verfügbar ist, wird `h` zunächst manuell gewählt und nach der gemessenen Nachbarabstandsverteilung nachgeregelt.

---

## 6. Initialisierung

### 6.1 Zufällige Punkte in der Bounding Box

Erzeuge `N` gleichverteilte Zufallspunkte in einer Bounding Box, die die Null-Isosurface sicher enthält. Verwende einen festen Seed für reproduzierbare Tests.

```cpp
for (uint32_t i = 0; i < N; ++i) {
    particles[i].position = randomUniform(bounds);
    particles[i].velocity = {0, 0, 0};
    particles[i].id = i;
}
```

Bei Formen mit mehreren nahe beieinander liegenden Oberflächenbereichen kann die reine Projektion Punkte auf eine unerwünschte nahe Oberfläche ziehen. Für die ersten Testformen ist das akzeptabel. Später können mehrfaches Resampling, Vorzeichenregeln oder surface-nahe Startverteilungen ergänzt werden.

### 6.2 Projektion auf die Null-Isosurface

Projektion erfolgt als Newton-artiger Schritt:

```cpp
bool projectToSDF(Vec3& p) {
    for (int iteration = 0; iteration < projectionIterations; ++iteration) {
        SDFSample s = sampleSDF(p);
        float g2 = dot(s.gradient, s.gradient);
        if (g2 < gradientEpsilon) return false;
        p -= s.distance * s.gradient / g2;
        if (abs(s.distance) < sdfTolerance) break;
    }
    return true;
}
```

Bei einem echten Distance Field mit normiertem Gradient ist die vereinfachte Form `p -= φ(p) * normalize(∇φ(p))` ausreichend. Die Division durch `|∇φ|²` ist robuster, wenn die Implementierung nur ein implizites Feld liefert.

Nach jeder Projektion werden Normalen aktualisiert:

```cpp
particle.normal = normalize(sampleSDF(particle.position).gradient);
```

### 6.3 Uniform Grid / Spatial Hash

Die Repulsion benötigt nur lokale Nachbarn. Statt einer globalen O(N²)-Suche wird ein Uniform Grid mit Zellgröße nahe `h` verwendet.

```text
cellSize = h
interactionRadius = 1.5h bis 2.0h
```

Ein Partikel untersucht nur seine eigene und die angrenzenden Zellen. Bei ungefähr gleichmäßiger Oberflächenbelegung bleibt die mittlere Nachbarzahl beschränkt; die Suche ist praktisch O(N).

Pseudocode:

```cpp
clear(grid);
for (const Particle& p : particles)
    grid.insert(cellOf(p.position), p.id);

for (Particle& p : particles)
    for (uint32_t j : grid.pointsInNeighborCells(cellOf(p.position)))
        accumulatePair(p, particles[j]);
```

Paare müssen eindeutig verarbeitet werden (`j > i`), falls Kräfte symmetrisch akkumuliert werden.

---

## 7. Tangentiale Repulsion und Relaxation

### 7.1 Kraftmodell

Für einen Nachbarn `q` mit `r = q.position - p.position`, `d = |r|` und Interaktionsradius `R` kann zunächst eine glatte, begrenzte Abstoßung verwendet werden:

```text
f(d) = k · (1 - d/R)² / max(d, ε),  für d < R
f(d) = 0,                             sonst
```

Die räumliche Kraft ist `F = -f(d) · r`. Entscheidend ist ihre Projektion in die Tangentialebene des aktuellen Partikels:

```cpp
Vec3 tangentForce(Vec3 force, Vec3 normal) {
    return force - dot(force, normal) * normal;
}
```

Die SDF-Projektion behandelt den Normalanteil als Zwangsbedingung; die Repulsion soll ausschließlich die Parameterisierung auf der Oberfläche verbessern.

### 7.2 Integration

Für den Prototypen eignen sich gedämpftes explizites Euler oder semi-implizites Euler. Ein überdämpfter Relaxationsmodus ist einfacher und stabiler als eine physikalisch realistische Bewegung:

```cpp
for (Particle& p : particles) {
    Vec3 force = tangentForce(repulsion(p), p.normal);
    p.velocity = damping * p.velocity + mobility * force;
    p.position += dt * p.velocity;
}

for (Particle& p : particles) {
    projectToSDF(p.position);
    p.normal = normalize(sampleSDF(p.position).gradient);
}
```

Nutze zu Beginn mehrere Substeps pro dargestelltem Frame. Begrenze optional `|velocity|` oder die maximale Verschiebung auf einen kleinen Anteil von `h`, damit Nachbarschaften nicht in einem Schritt übersprungen werden.

### 7.3 Konvergenzkriterium

Die Initialrelaxation endet nicht nach einer festen Zahl von Frames allein, sondern wenn mindestens die folgenden Größen ausreichend stabil sind:

- mittlere Geschwindigkeitsnorm unter einem Schwellwert;
- Standardabweichung der nächsten Nachbarabstände klein;
- maximaler SDF-Fehler unter der Toleranz;
- keine großen Cluster oder offensichtlich unbedeckten Oberflächenbereiche.

Eine feste Obergrenze schützt vor Endlosschleifen. Für visuelle Tests darf der Zwischenzustand schlecht aussehen; maßgeblich ist die konvergierte Verteilung.

---

## 8. Einmalige Initialtriangulation

Die Triangulation wird erst nach ausreichender Relaxation ausgeführt. Dieser Schritt darf deutlich teurer als ein Frame sein und wird außerhalb der Runtime-Hot-Loop betrachtet.

### 8.1 Empfohlener Prototypansatz: lokale tangentiale Delaunay-Nachbarschaft

Für jeden Partikel `p`:

1. Bestimme `n = normalize(∇φ(p))`.
2. Konstruiere eine orthonormale Tangentialbasis `(u, v)` mit `u ⟂ n`, `v = n × u`.
3. Sammle 8–16 räumlich nahe Nachbarn im Spatial Grid.
4. Projiziere jeden Nachbarn `q` in die lokale Ebene:

   ```text
   x = dot(q - p, u)
   y = dot(q - p, v)
   ```

5. Berechne eine lokale 2D-Delaunay-Triangulation oder zunächst eine winkelfolgende Heuristik.
6. Überführe vorgeschlagene Kanten in eine globale, deduplizierte Kantenmenge.
7. Erzeuge Dreiecke nur aus gegenseitig konsistenten Kanten und orientiere sie mit dem SDF-Gradienten.

Die Winkelheuristik ist für einen ersten Debug-Prototypen zulässig, aber nicht der langfristige Qualitätsstandard. Sie kann über konkave Lücken hinweg falsche Verbindungen vorschlagen. Die lokale Delaunay-Variante muss daher mit Radien, Normalenkompatibilität und globaler Konsistenz gefiltert werden.

### 8.2 Filter gegen falsche Verbindungen

Eine Kandidatenkante `(i,j)` wird abgewiesen, wenn eine der folgenden Bedingungen gilt:

- `|p_j - p_i| > maxEdgeLength`;
- die Normalen zu stark abweichen (`dot(n_i, n_j) < normalThreshold`);
- die Mitte der Kante deutlich von der SDF-Oberfläche abweicht;
- die Kante einen lokal erkannten bestehenden Rand kreuzt;
- sie zu einer nichtmanifold Kante führen würde.

Der Midpoint-Test ist besonders wertvoll bei dünnen, konkaven oder nahe beieinander liegenden SDF-Bereichen:

```cpp
if (abs(sampleSDF(0.5f * (a + b)).distance) > edgeSurfaceTolerance)
    rejectEdge();
```

### 8.3 Alternativen für die Initialisierung

Falls eine robuste eigene Triangulation den Prototypen blockiert, kann einmalig eine externe oder vorgefertigte Methode eingesetzt werden:

- Ball Pivoting: passend bei gleichmäßigem Sampling, gute Debug-Referenz;
- globale Surface-Reconstruction: nur als Offline-Vergleich;
- direkte SDF-Isoflächenextraktion (Marching Cubes/Dual Contouring): Referenzmesh, wenn die Partikel nicht die primäre Darstellung sein müssen.

Diese Verfahren gehören nicht in den normalen Runtime-Pfad. Das Prototypziel bleibt eine an die Partikelidentitäten gebundene Topologie.

### 8.4 Validierung und Orientierung

Für jedes Dreieck `(a,b,c)` wird die Orientierung mit der lokalen SDF-Normale kontrolliert:

```cpp
Vec3 faceNormal = cross(b - a, c - a);
Vec3 surfaceNormal = normalize(sampleSDF((a + b + c) / 3.0f).gradient);
if (dot(faceNormal, surfaceNormal) < 0.0f)
    swap(i1, i2);
```

Zusätzlich werden degenerierte Dreiecke, doppelte Indizes, doppelte Dreiecke und nichtmanifold Kanten gezählt. Die persistente Phase startet nur mit einem akzeptierten Mesh.

---

## 9. Persistente Laufzeitphase

Nach der Initialisierung bleibt `triangles[]` unverändert. Jede Partikel-ID entspricht einem Vertexindex.

```cpp
void update(float frameDt) {
    for (int substep = 0; substep < substeps; ++substep) {
        buildSpatialHash(particles);
        computeTangentialRepulsion(particles);
        integrate(particles, frameDt / substeps);

        for (Particle& p : particles) {
            projectToSDF(p.position);
            p.normal = normalize(sampleSDF(p.position).gradient);
        }
    }

    // Indexbuffer unverändert.
    uploadOrAliasVertexPositions(particles);
}
```

Im Idealfall referenziert der Renderer den Partikelpositionsbuffer direkt, etwa als SSBO/Structured Buffer im Vertexshader. Andernfalls wird nur ein zusammenhängender Vertexpositionsbuffer mit `N` Einträgen aktualisiert. Der Aufwand ist dann O(N), ohne Triangle-Rebuild oder Index-Upload.

---

## 10. Optionale spätere Edge-Flips

Edge-Flips sind die erste sinnvolle Topologiereparatur, wenn die ursprüngliche Konnektivität lokal unvorteilhaft wird. Sie ändern weder Partikelzahl noch Positionen, sondern nur die Diagonale zweier angrenzender Dreiecke.

```text
vorher:       nachher:
    A             A
   /|\           / \
  / | \         /   \
 B--+--C  →    B-----C
  \ | /         \   /
   \|/           \ /
    D             D
```

Eine gemeinsame Kante `A-D` wird durch `B-C` ersetzt, wenn beide Dreiecke existieren, das Viereck lokal gültig ist und eine Qualitätsmetrik steigt.

Geeignete Kriterien:

- Delaunay-artig: Summe der gegenüberliegenden Winkel;
- Verbesserung des kleinsten Innenwinkels;
- Verringerung von `longestEdge / shortestEdge`;
- Verringerung eines kombinierten Triangle-Quality-Scores.

Vorgehen:

1. Markiere Dreiecke mit schlechter Qualität.
2. Prüfe nur ihre Gegenkanten und maximal ein begrenztes Reparaturbudget pro Frame.
3. Führe nur Flips aus, die eine klare Verbesserung liefern.
4. Markiere die lokalen Nachbarn erneut.

Da `N` konstant ist und die SDF statisch bleibt, sind Edge-Splits und -Collapses zunächst nicht erforderlich. Sie werden erst relevant, wenn die gewünschte Dichte räumlich variieren soll oder die Verteilung trotz Relaxation dauerhaft starke Längenfehler erzeugt.

---

## 11. Metriken und Debug-Visualisierung

Metriken sind kein nachträglicher Komfort: Sie entscheiden, ob die persistente Topologie tatsächlich tragfähig ist.

| Bereich | Metrik | Erwartung |
|---|---|---|
| SDF-Zwang | `max_i |φ(p_i)|`, Mittelwert | nahe 0 |
| Verteilung | nächster Nachbar: Min/Mittel/Max/StdAbw. | enge Verteilung um `h` |
| Bewegung | Mittel/Maximum `|v|` | sinkt bei Relaxation |
| Dreiecke | Anzahl degenerierter Flächen | 0 |
| Form | min. Innenwinkel, Aspect Ratio | keine extremen Ausreißer |
| Kanten | Längenhistogramm | konzentriert um `h` |
| Orientierung | Dreiecke gegen SDF-Normale | 0 |
| Topologie | Randkanten, nichtmanifold Kanten, Komponenten | erwartete Werte |
| Kosten | Grid, Kräfte, Projektion, Upload, Gesamtdauer | separat messen |

Nützliche Ansichten:

- Partikel farbcodiert nach nächstem Nachbarabstand;
- Dreiecke farbcodiert nach Aspect Ratio oder Minimalwinkel;
- Kanten farbcodiert nach Länge;
- fehlerhafte SDF-Projektionen und abgelehnte Kanten als Marker;
- Normallinien für Partikel und Faces;
- Zeitreihe der obigen Metriken.

Die Qualität sollte vor allem während der langen Stabilitätstests beobachtet werden. Nicht jede Zwischeniteration muss ein korrektes Mesh liefern; ein dauerhafter Trend zu Degeneration ist hingegen ein klarer Fehlschlag des Ansatzes oder seiner Parameter.

---

## 12. Startparameter

Die folgenden Werte sind dimensionslose Startpunkte und müssen an `h` sowie die Zeitskala angepasst werden.

| Parameter | Startwert | Zweck |
|---|---:|---|
| Partikelzahl | 10 000 | erster Performanz-/Qualitätstest |
| Zielabstand `h` | `sqrt(A/N)` | räumliche Referenz |
| Grid-Zellgröße | `h` | lokale Nachbarsuche |
| Repulsionsradius `R` | `1.5h` | genügend Nachbarn, lokal bleiben |
| erwartete Nachbarn | 8–16 | triangulierbare lokale Umgebung |
| Substeps | 2–8 pro Frame | Stabilität |
| Projektionen | 1–2 je Substep | SDF-Zwang |
| Dämpfung | 0.8–0.98 | verhindert Oszillation |
| max. Schrittweite | `0.05h–0.2h` | verhindert Nachbarschaftssprünge |
| max. Kantenlänge | `1.5h–2.0h` | Filter bei Triangulation |
| SDF-Toleranz | skalenabhängig, z. B. `1e-4h` | Konvergenz |

Ändere nie mehrere Parametergruppen gleichzeitig. Zuerst die reine Partikelverteilung auf der Kugel stabilisieren, dann Triangulation, dann komplexere Geometrie.

---

## 13. Testfälle

### Test 1: Kugel

Der Referenztest. Erwartet werden gleichmäßige Abstände, fast gleichseitige Dreiecke und keine schwierigen topologischen Fälle. Fehlschläge hier sind meist Probleme in Projektion, Kraftskalierung, Grid oder Orientierung.

### Test 2: Ellipsoid oder deformierte Kugel

Testet wechselnde Krümmung bei einfacher Topologie. Prüft, ob Tangentialprojektion und Nachbarsuche auch bei ungleichförmiger Einbettungsgeometrie funktionieren.

### Test 3: Torus

Testet nichttriviale Topologie und die Gefahr, gegenüberliegende Bereiche des dünnen Rings irrtümlich zu verbinden. Der Midpoint-SDF-Test und eine ausreichend lokale Nachbarschaft sind hier besonders wichtig.

### Test 4: Konkave CSG-Form

Zum Beispiel Kugel minus kleinerer versetzter Kugel oder eine weiche U-Form. Prüft, ob Kandidatenkanten quer über Einbuchtungen entstehen.

### Test 5: Dünne Nähe / hohe Krümmung

Zwei nahe, aber getrennte Oberflächenbereiche bzw. ein enger Kanal. Dieses Szenario definiert die Grenzen der lokalen Heuristik und der gewählten Samplingdichte.

### Test 6: Langzeittest

Auf Kugel, Torus und konkaver Form 10 Sekunden, 60 Sekunden und 5 Minuten simulieren. Logge Metriken und prüfe, ob Triangle-Qualität, Orientierung oder SDF-Fehler langsam wegdriften.

---

## 14. Entwicklungsphasen und Abnahmekriterien

### Phase A — SDF und Projektion

Implementiere SDF-Schnittstelle, Kugel und Projektion. Visualisiere Punkte vor/nach Projektion.

**Abnahme:** `max |φ(p)|` liegt nach Projektion zuverlässig unter der Toleranz.

### Phase B — Surface-Relaxation

Implementiere Grid, Nachbarsuche, tangentiale Repulsion und Dämpfung. Noch kein Mesh rendern.

**Abnahme:** Auf der Kugel konvergiert die Standardabweichung der Nachbarabstände sichtbar; keine instabilen Cluster.

### Phase C — Initialtriangulation

Implementiere Tangentialbasis, Kandidatenkanten, globale Deduplizierung, Triangle-Aufbau, Orientierung und Validierung. Starte mit Kugel.

**Abnahme:** Geschlossenes, orientiertes Mesh ohne degenerierte oder nichtmanifold Kanten; Kantenlängen um `h`.

### Phase D — Persistentes Runtime-Mesh

Kopple Partikel-ID und Vertex-ID. Halte Indizes fest und aktualisiere nur Positionen.

**Abnahme:** Indexbuffer bleibt über den Langzeittest bit-identisch; Render-Mesh folgt dem Partikelbuffer ohne separate Positionsdrift.

### Phase E — Robustheit auf komplexen SDFs

Führe Torus- und Konkavitätstests aus. Ergänze Kantenfilter, Normalenprüfung und Fehlerdiagnosen.

**Abnahme:** Keine offensichtlichen Brücken durch den Torus bzw. über konkave Einbuchtungen; bekannte Grenzfälle sind messbar dokumentiert.

### Phase F — Lokale Reparatur (optional)

Führe Half-Edge-Datenstruktur und budgetierte Edge-Flips ein. Beginne nur mit klaren Qualitätsschwellen.

**Abnahme:** Flips verbessern nachweisbar die lokale Qualitätsmetrik und erzeugen keine nichtmanifold Topologie.

### Phase G — Optimierung / GPU (optional)

Übertrage Grid-Aufbau, Nachbarsuche, Kräfte und Vertexzugriff schrittweise auf Compute/Graphics-Hardware. Halte die CPU-Referenz als Validierungsmodus.

**Abnahme:** Gleiche Qualitätsmetriken innerhalb vereinbarter Toleranzen; getrennt gemessene Performancegewinne.

---

## 15. Risiken und Gegenmaßnahmen

| Risiko | Symptom | Gegenmaßnahme |
|---|---|---|
| Instabile Repulsion | Oszillation oder Cluster | mehr Dämpfung, kleinere Schritte, Substeps |
| Schlechte Projektion | sichtbarer Off-Surface-Drift | Newton-Form, Gradient prüfen, mehr Iterationen |
| Falsche Kanten über Lücken | Brücken in konkaven/dünnen Bereichen | kleinerer Radius, Normalenfilter, Midpoint-SDF-Test |
| Schlechte Anfangstopologie | früh extrem schmale Dreiecke | bessere lokale Delaunay-Konsistenz, Mesh validieren |
| Langsame Degeneration | Quality-Score verschlechtert sich | Schrittlimit, stärkere Relaxation, später Edge-Flips |
| Nichtmanifold Mesh | Kanten mit >2 Faces | globale Kantenverwaltung und strikte Validierung |
| Performancebruch | Nachbarsuche dominiert | Zellgröße und Paarzählung prüfen, Grid/GPU optimieren |

---

## 16. Nicht in den ersten Prototyp aufnehmen

- Poisson-, BPA- oder globale Delaunay-Rekonstruktion pro Frame;
- globale Remeshing-Pässe;
- Edge-Splits und -Collapses;
- adaptive Partikelzahl;
- zeitvariable SDFs oder topologische Übergänge;
- GPU-Optimierung vor einer validierten CPU-Referenz.

Diese Funktionen erhöhen die Komplexität deutlich und beantworten nicht die erste Kernfrage: Ob die persistente Topologie unter den gegebenen, günstigen Randbedingungen schon genügt.

---

## 17. Erwartetes Minimalergebnis

Am Ende des ersten Prototyps sollen folgende Ergebnisse vorliegen:

1. Eine Kugel-SDF mit reproduzierbarer Partikelinitialisierung und stabiler Relaxation.
2. Ein einmal trianguliertes, geschlossenes, validiertes Mesh.
3. Ein Laufzeitpfad, der ausschließlich `N` Partikelpositionen aktualisiert und denselben Indexbuffer weiterverwendet.
4. Ein Metrik-Dashboard bzw. Log für SDF-Fehler, Nachbarabstände, Triangle-Qualität und Laufzeit.
5. Ein dokumentierter Langzeittest auf Kugel und Torus.
6. Eine klare Entscheidung auf Basis der Messungen: persistente Topologie genügt, oder Edge-Flips werden als nächster minimaler Eingriff ergänzt.

Damit ist die Architekturfrage früh und belastbar beantwortet, bevor Aufwand in vollständiges dynamisches Remeshing investiert wird.
