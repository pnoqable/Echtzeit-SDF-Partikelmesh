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