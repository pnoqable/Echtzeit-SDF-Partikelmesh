#pragma once

#include "SDF.hpp"
#include <glm/glm.hpp>
#include <cmath>

class SphereSDF : public SDF {
public:
    SphereSDF(glm::vec3 center, float radius);

    SDFSample sample(glm::vec3 p) const override;
    glm::vec3 boundsMin() const override;
    glm::vec3 boundsMax() const override;
    float surfaceArea() const override;

private:
    glm::vec3 m_center;
    float m_radius;
};

// SDF für ein Ellipsoid mit Halbachsen (rx, ry, rz). Gradient über die
// implizite Gleichung (x/rx)² + (y/ry)² + (z/rz)² = 1 abgeleitet; Fläche via
// Knud-Thomsen-Näherung.
class EllipsoidSDF : public SDF {
public:
    EllipsoidSDF(glm::vec3 center, float rx, float ry, float rz);

    SDFSample sample(glm::vec3 p) const override;
    glm::vec3 boundsMin() const override;
    glm::vec3 boundsMax() const override;
    float surfaceArea() const override;

private:
    glm::vec3 m_center;
    float m_rx, m_ry, m_rz;
};

// Torus in der xz-Ebene: major radius R (Ringmitte→Röhrenmitte), minor radius r.
class TorusSDF : public SDF {
public:
    TorusSDF(glm::vec3 center, float majorRadius, float minorRadius);

    SDFSample sample(glm::vec3 p) const override;
    glm::vec3 boundsMin() const override;
    glm::vec3 boundsMax() const override;
    float surfaceArea() const override;

private:
    glm::vec3 m_center;
    float m_major, m_minor;
};

// KONKAVE Testform: Hantel aus zwei gleich großen Kugeln (Radius R), deren
// Mittelpunkte um 2a überlappen (Modell: min(S1, S2)). Zwischen beiden entsteht
// eine konkave Sattelfläche („Hals“). Das ist der Standard-Test für falsche
// Querverbindungen über eine Konkavität hinweg; die zwei Gratkreise, auf denen
// die Kugeln zusammenstoßen, erzeugen zusätzliche Gratkanten.
//
// Konfigurierbarer Überlappungsparameter `a` mit auf
// [0, R) begrenzter Öffnung (a → R: Kugeln berühren sich nur noch).
//
// Die sichtbare (Nicht-überlappte) Fläche ist exakt:
//   A = 2 · (4πR² − 2πR·(R−a))             [je Kugel die sichtbare Kappe]
//     = 4πR·(R + a)
class DumbbellSDF : public SDF {
public:
    DumbbellSDF(glm::vec3 center, float radius, float halfSeparation);

    SDFSample sample(glm::vec3 p) const override;
    glm::vec3 boundsMin() const override;
    glm::vec3 boundsMax() const override;
    float surfaceArea() const override;

private:
    glm::vec3 m_center;
    float m_radius;
    float m_a;      // halber Mittelpunktsabstand (Überlappungskonfiguration)
    glm::vec3 m_d1; // Mittelpunkt Kugel 1 (Richtung +x)
    glm::vec3 m_d2; // Mittelpunkt Kugel 2 (Richtung −x)
};

// Weiche Testform: Zwei gleich große Kugeln (Radius R), Mittelpunkte bei ±a
// auf der x-Achse (gleiche Geometrie wie DumbbellSDF), aber mit polynomialem
// Smooth Min (Inigo Quílez) verbunden statt hartem min(S1,S2):
//
//   h = clamp(0.5 + 0.5·(φ2 − φ1)/k, 0, 1)
//   φ = mix(φ2, φ1, h) − k·h·(1−h)
//
// Der Parameter k steuert die Wärme der Überblendung: k → 0 entspricht der
// harten konkaven Hantel, wachsendes k lässt die Kugeln zunehmend zu einem
// durchgängigen, weich geschwungenen Blob („Peanut“/Metaball) verschmelzen.
// Nutzt man k ≥ 4(R − a), schließt die Null-Isofläche am Sattelpunkt und es
// entsteht ein einzelner zusammenhängender, geschlossener Körper.
//
// GRADIENT analytisch über die Kettenregel: ∇φ = h·∇φ1 + (1−h)·∇φ2
//   + ∇h·(φ1 − φ2 − 2k·(0.5 − h)), mit ∇h = (0.5/k)·(∇φ2 − ∇φ1) innerhalb der
//   Überblendung (0 < h < 1), sonst ∇h = 0. Achtung: |∇φ| ist im Blendbereich
//   < 1, die Newton-Projektion (normalisiert über |g|²) konvergiert trotzdem.
//
// Fläche: Die Kapsel-Näherung 4πR·(R + a) unterschätzt die aufgeblähte
// Null-Isofläche bei großem k deutlich (Messung: +50 % bei k=2.5, R=1, a=0.5).
// Deshalb wird sie beim Konstruieren numerisch bestimmt. Die Form ist rota-
// tionssymmetrisch um die x-Achse: Das Profil r(x) der Null-Isofläche wird per
// Bisektion gelöst und die Oberfläche als Rotationsintegral A = 2π·∫r·ds
// (exakt für Rotationskörper, deterministisch, einmalig, ~10000 SDF-Samples).
// Gleichzeitig liefert das Profil die exakten Bounding-Box-Grenzen, denn der
// Blend kann die Isofläche über die reine Kugel-Kappe hinauswölben.
class MetaballSDF : public SDF {
public:
    MetaballSDF(glm::vec3 center, float radius, float halfSeparation, float smoothK);

    SDFSample sample(glm::vec3 p) const override;
    glm::vec3 boundsMin() const override;
    glm::vec3 boundsMax() const override;
    float surfaceArea() const override;

private:
    // Berechnet das x-Profil r(x) der ==0-Isofläche, daraus area, boundsMin, boundsMax.
    void computeProfile();

    glm::vec3 m_center;
    float m_radius;
    float m_a;      // halber Mittelpunktsabstand
    float m_k;      // Smooth-Min-Parameter (Wärme der Überblendung)
    glm::vec3 m_d1; // Mittelpunkt Kugel 1 (Richtung +x)
    glm::vec3 m_d2; // Mittelpunkt Kugel 2 (Richtung −x)
    float m_area;   // numerisch bestimmte Oberfläche
    glm::vec3 m_boundsMin, m_boundsMax;
};

// KONKAVER CSG-TESTFALL (Plan §13, Test 4): Kugel A (Radius R, Zentrum c) minus
// einer versetzten Kugel B (Radius r < R, Zentrum um `offset` entlang +x
// verschoben). Anders als eine harte CSG-Differenz (max(φA, −φB), Gratkante
// mit ~116°-Normalensprung, die die Fan-Triangulation offen lässt) wird hier
// das Maximum über den Quílez-Polynom-Smooth-Max weich überblendet („weiche
// U-Form“): die Schnittkante wird gerundet, die Triangulation kann sie schließen.
//
//   h   = clamp(0.5 + 0.5·(a − b)/k, 0, 1),  a = φA, b = −φB
//   φ   = b + h·(a − b) + k·h·(1 − h)      (+k vor k·h(1−h) = Smooth-Max)
//   ∇φ  = h·∇φA + (1 − h)·∇(−φB)
//
// Fläche numerisch: Wie beim Metaball Rotationsintegral um die x-Achse, aber
// per Marching-Squares-Kontur von φ(x,r,0) = 0 im (x,r)-Querschnitt. Notwendig,
// weil der Querschnitt im Überlappungsbereich HOLL ist (zwei Profil-Äste: äußere
// Kugelwand und Innenwand der Ausnehmung, verbunden über die gerundete Kante).
// Die Konturverfolgung liefert exakterweise A = 2π·∫r·ds über alle Kontursegmente.
class SphereMinusSphereSDF : public SDF {
public:
    // Kenngröße: |R−r| < offset < R+r (Kugeln schneiden). `smoothK` = Wärme
    // der Blendung (k → 0 entspricht der harten CSG-Gratkante).
    SphereMinusSphereSDF(glm::vec3 center, float radius, float cutRadius, float offset, float smoothK);

    SDFSample sample(glm::vec3 p) const override;
    glm::vec3 boundsMin() const override;
    glm::vec3 boundsMax() const override;
    float surfaceArea() const override;

private:
    void computeProfile();

    glm::vec3 m_center;
    float m_radius;
    float m_cutRadius;
    float m_offset;  // Versatz des Kugelzentrums B entlang +x
    float m_k;       // Smooth-Max-Parameter (Wärme der Überblendung)
    glm::vec3 m_cutCenter;
    float m_area = 0.0f;
    glm::vec3 m_boundsMin, m_boundsMax;
};

// ORGANISCHER FELSBROCKEN: Ellipsoid mit fraktalem Oberflaechen-Displacement,
// im Stil der Displacement-Shader von Inigo Quilez:
//
//   d      = p − c,   q = d/radii,   k = |q|
//   φ(p)   = (k − 1)·r̄ − amp·(2·fbm(q·freq + seed) − 1)
//
// Das fbm wird auf wenige Oktaven begrenzt ("detailarm"); die Buckel sind
// gross und glatt. Der Gradient ist voll analytisch: die trilineare
// Value-Noise wird mit ihrem exakten Gradienten ausgewertet (Fade-Polynom +
// Produktregel), das fbm summiert die Oktaven mit Frequenz-Skalierung.
//
// Der Seed geht als nicht-ganzzahlige Translation des Noise-Gitters ein;
// 50 Stellungen ergeben 50 verschiedene Brocken.
//
// Zur Vereinfachung ist das Grund-Ellipsoid ueber sein Potential approximiert
// (φ_ell = (k−1)·r̄, ∇φ_ell = r̄·(d_i/r_i²)/k); fuer die organische Form spielt
// das Displacement ohnehin die dominante Rolle.
//
// Flaeche: Monte-Carlo ueber die sternfoermige Flaechengleichung r(ω):
//   A = ∫ r²/|n̂·ω̂| dω,  ausgewertet ueber Fibonacci-Sphere-Richtungen mit
//   Bisektion entlang jedes Strahls (einmalig im Konstruktor).
class RockSDF : public SDF {
public:
    RockSDF(glm::vec3 center, float rx, float ry, float rz,
            float amplitude, float frequency, int octaves, int seed);

    SDFSample sample(glm::vec3 p) const override;
    glm::vec3 boundsMin() const override;
    glm::vec3 boundsMax() const override;
    float surfaceArea() const override;

private:
    void computeData();

    glm::vec3 m_center;
    glm::vec3 m_radii;
    float m_amp;
    float m_freq;
    int m_octaves;
    glm::vec3 m_seedOffset;
    float m_area;
    glm::vec3 m_boundsMin, m_boundsMax;
};

// ORGANISCHER FELS-TORUS: Torus mit fraktalem Oberflaechen-Displacement, im
// selben Stil wie RockSDF (Quilez), aber auf den Torus-SDF bezogen:
//
//   s     = sqrt(x² + z² + r_minor²)   (glatte, nie singulaere Ringkoordinate)
//   lam   = ( x/s, y/R, 0 )             (nahtlos; nur Azimit + Ringachse, kein
//                                        Radialterm, damit ∇D ⊥ Rohrnormale ist)
//   φ(p)  = φ_torus(q) − amp·(2·fbm(lam·freq + seed) − 1)
//
// Die Noise-Koordinate laeuft ueber den Azimit-Winkel (via s, ohne atan-Seam)
// und die Ringachse; dadurch zeigt ∇D tangential, der Gesamtgradient bleibt
// nahe der Rohrnormalen (stabile Partikelprojektion). Die Amplitude wird im
// Konstruktor auf 0.9·(R − r) begrenzt, damit die Lochmitte des Torus offen
// bleibt. Der Gradient ist voll analytisch (Kettenregel ueber die Noise-Koord).
//
// Flaecheninhalt: numerisches Flaechenintegral ueber die parameterisierte,
// displacements-transformierte Tube (Girard, 96×96 Stuetzstellen, einmalig im
// Konstruktor); der star-ray-Ansatz von RockSDF versagt hier, weil der Torus
// von seinem Zentrum aus nicht sternfoermig ist.
class RockTorusSDF : public SDF {
public:
    RockTorusSDF(glm::vec3 center, float majorRadius, float minorRadius,
                 float amplitude, float frequency, int octaves, int seed);

    SDFSample sample(glm::vec3 p) const override;
    glm::vec3 boundsMin() const override;
    glm::vec3 boundsMax() const override;
    float surfaceArea() const override;

private:
    void computeData();
    void evaluateDisplacement(const glm::vec3& q, float& D, glm::vec3& gradD) const;

    glm::vec3 m_center;
    float m_major, m_minor;
    float m_amp;
    float m_freq;
    int m_octaves;
    glm::vec3 m_seedOffset;
    float m_area;
    glm::vec3 m_boundsMin, m_boundsMax;
};