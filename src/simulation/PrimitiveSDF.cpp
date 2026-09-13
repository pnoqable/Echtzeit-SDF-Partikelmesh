#include "PrimitiveSDF.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <cmath>

// ---------- Kugel ----------

SphereSDF::SphereSDF(glm::vec3 center, float radius)
    : m_center(center), m_radius(radius) {}

SDFSample SphereSDF::sample(glm::vec3 p) const {
    glm::vec3 diff = p - m_center;
    float len = glm::length(diff);
    if (len < 1e-6f) return { -m_radius, { 0.0f, 1.0f, 0.0f } };
    return { len - m_radius, diff / len };
}

float SphereSDF::surfaceArea() const {
    return 4.0f * glm::pi<float>() * m_radius * m_radius;
}

glm::vec3 SphereSDF::boundsMin() const {
    return m_center - glm::vec3(m_radius);
}

glm::vec3 SphereSDF::boundsMax() const {
    return m_center + glm::vec3(m_radius);
}

// ---------- Ellipsoid ----------

EllipsoidSDF::EllipsoidSDF(glm::vec3 center, float rx, float ry, float rz)
    : m_center(center), m_rx(rx), m_ry(ry), m_rz(rz) {}

// Distanz = K·(|p|normiert − 1), Gradient analytisch aus der impliziten
// Ellipsoidgleichung; die Normierung faktorisiert den Betrag des Gradienten.
SDFSample EllipsoidSDF::sample(glm::vec3 p) const {
    glm::vec3 q = (p - m_center) / glm::vec3(m_rx, m_ry, m_rz);
    float k = glm::length(q);
    if (k < 1e-6f) return { -1.0f, { 0.0f, 1.0f, 0.0f } };
    float d = k - 1.0f;
    // (q.x², q.y², q.z²)^T / (rx,ry,rz)·(rx,ry,rz) normiert
    glm::vec3 grad = q * glm::vec3(1.0f / (m_rx * m_rx), 1.0f / (m_ry * m_ry), 1.0f / (m_rz * m_rz));
    float len = glm::length(grad);
    if (len < 1e-9f) return { d, { 0.0f, 1.0f, 0.0f } };
    return { d, grad / len };
}

glm::vec3 EllipsoidSDF::boundsMin() const {
    return m_center - glm::vec3(m_rx, m_ry, m_rz);
}

glm::vec3 EllipsoidSDF::boundsMax() const {
    return m_center + glm::vec3(m_rx, m_ry, m_rz);
}

// Knud-Thomsen-Näherung, Fehler < 1.2 % für beliebige Halbachsen. Der exakte
// Wert erfordert elliptische Integrale; für das Spacing h = sqrt(A/N) reicht
// diese Näherung aus.
float EllipsoidSDF::surfaceArea() const {
    const float p = 1.6075f;
    float t = std::pow(m_rx * m_ry, p) + std::pow(m_rx * m_rz, p) + std::pow(m_ry * m_rz, p);
    return 4.0f * glm::pi<float>() * std::pow(t / 3.0f, 1.0f / p);
}

// ---------- Torus ----------

TorusSDF::TorusSDF(glm::vec3 center, float majorRadius, float minorRadius)
    : m_center(center), m_major(majorRadius), m_minor(minorRadius) {}

// Torus in der xz-Ebene: |(len(xz) − R, y)| − r, Gradient analytisch.
SDFSample TorusSDF::sample(glm::vec3 p) const {
    glm::vec3 q = p - m_center;
    glm::vec2 xz(q.x, q.z);
    float d2 = glm::dot(xz, xz);
    if (d2 < 1e-6f) return { m_major - m_minor, { 1.0f, 0.0f, 0.0f } };
    float lenXz = std::sqrt(d2);
    glm::vec2 g(lenXz - m_major, q.y);
    float d = glm::length(g) - m_minor;
    glm::vec2 gdir = g / std::max(1e-6f, glm::length(g));
    glm::vec3 normal(gdir.x * q.x / lenXz, gdir.y, gdir.x * q.z / lenXz);
    return { d, glm::normalize(normal) };
}

glm::vec3 TorusSDF::boundsMin() const {
    float r = m_major + m_minor;
    return m_center - glm::vec3(r, m_minor, r);
}

glm::vec3 TorusSDF::boundsMax() const {
    float r = m_major + m_minor;
    return m_center + glm::vec3(r, m_minor, r);
}

float TorusSDF::surfaceArea() const {
    return 4.0f * glm::pi<float>() * glm::pi<float>() * m_major * m_minor; // 4π²Rr (Guldin)
}

// ---------- Hantel (konkav) ----------

DumbbellSDF::DumbbellSDF(glm::vec3 center, float radius, float halfSeparation)
    : m_center(center), m_radius(radius), m_a(halfSeparation),
      m_d1(center + glm::vec3(halfSeparation, 0.0f, 0.0f)),
      m_d2(center - glm::vec3(halfSeparation, 0.0f, 0.0f)) {}

// min(S1, S2): distanz zum jeweils näheren Mittelpunkt; Gradient des Min.
SDFSample DumbbellSDF::sample(glm::vec3 p) const {
    glm::vec3 v1 = p - m_d1;
    glm::vec3 v2 = p - m_d2;
    float d1 = glm::length(v1);
    float d2 = glm::length(v2);
    float ph1 = d1 - m_radius;
    float ph2 = d2 - m_radius;
    if (ph1 < ph2) {
        if (d1 < 1e-6f) return { ph1, { 0.0f, 1.0f, 0.0f } };
        return { ph1, v1 / d1 };
    }
    if (d2 < 1e-6f) return { ph2, { 0.0f, 1.0f, 0.0f } };
    return { ph2, v2 / d2 };
}

glm::vec3 DumbbellSDF::boundsMin() const {
    float rMax = m_radius + m_a; // extremer +x-Punkt der Kugel 1
    return m_center - glm::vec3(rMax, m_radius, m_radius);
}

glm::vec3 DumbbellSDF::boundsMax() const {
    float rMax = m_radius + m_a;
    return m_center + glm::vec3(rMax, m_radius, m_radius);
}

// Exakt: je Kugel ist nur die außerhalb der jeweils anderen sichtbare Kappe
// belegt (Kappen-Höhe R − a), A = 2·(4πR² − 2πR(R−a)) = 4πR(R + a).
float DumbbellSDF::surfaceArea() const {
    return 4.0f * glm::pi<float>() * m_radius * (m_radius + m_a);
}