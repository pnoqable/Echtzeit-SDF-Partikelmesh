#include "PrimitiveSDF.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

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

// Echte vorzeichenbehaftete Distanz zum Ellipsoid via exaktem Fusspunkt.
// Fusspunkt q erfuellt p = q + lambda*n(q)  =>  q_i = p_i*a_i^2/(a_i^2+lambda).
// Aus der Ellipsoidgleichung ergibt sich die MONOTONE Skalargleichung
//   f(lambda) = sum_i p_i^2 * a_i^2 / (a_i^2 + lambda)^2 = 1.
// Ausserhalb: Lambda>0 (f(0)=|q|^2 > 1, f->0 fuer lambda->inf).
// Innen:       -min(a_i^2) < lambda <= 0 (f->inf am Pol, f(0) < 1).
// Bisektion (60 Schritte) ist robust und liefert |grad phi| = 1 exakt.
SDFSample EllipsoidSDF::sample(glm::vec3 p) const {
    glm::vec3 d = p - m_center;
    float a2 = m_rx * m_rx, b2 = m_ry * m_ry, c2 = m_rz * m_rz;

    auto f = [&](float lambda) {
        float t = d.x * d.x * a2 / ((a2 + lambda) * (a2 + lambda));
        t += d.y * d.y * b2 / ((b2 + lambda) * (b2 + lambda));
        t += d.z * d.z * c2 / ((c2 + lambda) * (c2 + lambda));
        return t; // == 1 gesucht
    };

    float minR2 = std::min(a2, std::min(b2, c2));
    bool outside = f(0.0f) > 1.0f;
    float lo, hi;
    if (outside) {
        lo = 0.0f; hi = 1.0f;
        while (f(hi) > 1.0f) hi *= 2.0f;
    } else {
        lo = -minR2 + 1e-6f * minR2;
        hi = 0.0f;
    }
    // Bisektion: f monoton fallend, lo: f>1, hi: f<1 (lo<hi).
    for (int it = 0; it < 60; ++it) {
        float mid = 0.5f * (lo + hi);
        if (f(mid) > 1.0f) lo = mid; else hi = mid;
    }
    float lambda = 0.5f * (lo + hi);

    glm::vec3 q(d.x * a2 / (a2 + lambda),
                d.y * b2 / (b2 + lambda),
                d.z * c2 / (c2 + lambda));
    glm::vec3 toQ = d - q;
    float len = glm::length(toQ);
    if (len < 1e-6f) {
        // praktisch auf der Oberfläche: Normale aus der Ellipsoidgleichung
        glm::vec3 nrm(q.x / a2, q.y / b2, q.z / c2);
        nrm = glm::normalize(nrm);
        if (glm::length(nrm) < 1e-6f || nrm.x != nrm.x) nrm = glm::vec3(0.0f, 1.0f, 0.0f);
        return { 0.0f, nrm };
    }
    float dist = outside ? len : -len;
    glm::vec3 nrm = outside ? toQ / len : -toQ / len;
    return { dist, nrm };
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

// ---------- Metaball (Smooth Min) ----------

MetaballSDF::MetaballSDF(glm::vec3 center, float radius, float halfSeparation, float smoothK)
    : m_center(center), m_radius(radius), m_a(halfSeparation), m_k(std::max(smoothK, 1e-4f)),
      m_d1(center + glm::vec3(halfSeparation, 0.0f, 0.0f)),
      m_d2(center - glm::vec3(halfSeparation, 0.0f, 0.0f)) {
    computeProfile();
}

// Polynomialer Smooth Min (Quílez):
//   h   = clamp(0.5 + 0.5·(φ2 − φ1)/k, 0, 1)
//   φ   = mix(φ2, φ1, h) − k·h·(1−h)
// Gradient via Kettenregel (k → 0 liefert die harte Dumbbell als Grenzfall):
//   ∇φ = (h − c0)·∇φ1 + (1 − h + c0)·∇φ2
//   c0 = (φ1 − φ2 − k·(1 − 2h)) · 0.5/k, nur für 0 < h < 1 gültig; an den
//   Clamp-Rändern fällt der Korrekturterm weg (dh = 0), es bleibt ∇φ1 bzw. ∇φ2.
SDFSample MetaballSDF::sample(glm::vec3 p) const {
    glm::vec3 v1 = p - m_d1;
    glm::vec3 v2 = p - m_d2;
    float d1 = glm::length(v1);
    float d2 = glm::length(v2);
    float ph1 = d1 - m_radius;
    float ph2 = d2 - m_radius;

    glm::vec3 n1 = (d1 < 1e-6f) ? glm::vec3(0.0f, 1.0f, 0.0f) : v1 / d1;
    glm::vec3 n2 = (d2 < 1e-6f) ? glm::vec3(0.0f, 1.0f, 0.0f) : v2 / d2;

    float h = glm::clamp(0.5f + 0.5f * (ph2 - ph1) / m_k, 0.0f, 1.0f);
    float phi = ph2 + h * (ph1 - ph2) - m_k * h * (1.0f - h);

    glm::vec3 grad;
    if (h <= 0.0f) {
        grad = n2;                       // nur Kugel 2
    } else if (h >= 1.0f) {
        grad = n1;                       // nur Kugel 1
    } else {
        float c0 = (ph1 - ph2 - m_k * (1.0f - 2.0f * h)) * (0.5f / m_k);
        grad = (h - c0) * n1 + (1.0f - h + c0) * n2;
    }
    return { phi, grad };
}

glm::vec3 MetaballSDF::boundsMin() const {
    return m_boundsMin;
}

glm::vec3 MetaballSDF::boundsMax() const {
    return m_boundsMax;
}

float MetaballSDF::surfaceArea() const {
    return m_area;
}

// x-Profil r(x) der ==0-Isofläche: an jedem x-Schnitt wird r im Intervall
// [0, rMax] per Bisektion aus φ(x, r, 0) gelöst. Die Form ist rotations-
// symmetrisch um die x-Achse, daher gilt für die Oberfläche das Rotationsintegral
//   A = 2π·∫ r·ds,  ds = √(dx² + dr²)  (Trapezregel, exakt für Rotationskörper).
// rMax = R + a + k deckt den weitesten möglichen Auswuchs des Blends ab; die
// Bisektion beginnt so weit aussen, dass φ(Hals, rMax) definitiv > 0 ist.
void MetaballSDF::computeProfile() {
    const float rMax = m_radius + m_a + m_k;
    const int slices = 256;
    const float xLo = -(rMax + 1e-3f);
    const float xHi = rMax + 1e-3f;
    const float dx = (xHi - xLo) / slices;

    std::vector<float> xs, rs;
    xs.reserve(slices + 1);
    rs.reserve(slices + 1);

    float minX = xHi, maxX = xLo, maxR = 0.0f;

    for (int i = 0; i <= slices; ++i) {
        float x = xLo + i * dx;
        // phi(x, r, 0) = 0 nach r aufgeloest. Am Hals (phi < 0 bei r=0) und
        // aussen (phi > 0 bei r=rMax) hat die Isoflaeche genau eine Wurzel.
        float lo = 0.0f, hi = rMax;
        SDFSample sl = sample({x, lo, 0.0f});
        SDFSample sh = sample({x, hi, 0.0f});
        if (sl.distance <= 0.0f && sh.distance >= 0.0f) {
            for (int it = 0; it < 50; ++it) {
                float mid = 0.5f * (lo + hi);
                if (sample({x, mid, 0.0f}).distance <= 0.0f)
                    lo = mid;
                else
                    hi = mid;
            }
            float r = 0.5f * (lo + hi);
            xs.push_back(x);
            rs.push_back(r);
            if (r > maxR) maxR = r;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
        }
    }

    // Trapezregel fuer A = 2π ∫ r ds (r̄ = Mittelwert der Segmentradii).
    // Luecke im x-Profil (xs-Sprung > dx, z.B. zwei getrennte Blobs bei
    // a >= R und kleinem k) darf NICHT ueberbrueckt werden: ein kuenstliches
    // Verbindungssegment wuerde sonst die Flaeche faelschlich vergroessern.
    float area2 = 0.0f;
    for (size_t i = 1; i < rs.size(); ++i) {
        if (xs[i] - xs[i - 1] > 1.2f * dx) continue;
        float ds = std::hypot(xs[i] - xs[i - 1], rs[i] - rs[i - 1]);
        area2 += glm::pi<float>() * (rs[i - 1] + rs[i]) * ds; // 2π·r̄·ds
    }

    m_area = area2;
    float halfR = std::max(maxR, m_radius);
    m_boundsMin = m_center - glm::vec3(std::fabs(minX), halfR, halfR);
    m_boundsMax = m_center + glm::vec3(std::fabs(maxX), halfR, halfR);
}