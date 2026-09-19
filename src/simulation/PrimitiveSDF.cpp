#include "PrimitiveSDF.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

// ---------- Value-Noise + fbm (analytischer Gradient) ----------

namespace {

// Integer-Hash (Wang-artig) → [0,1). Seed-frei; der Seed geht als
// Gitter-Translation in die Sample-Position ein.
inline float hash3i(int x, int y, int z) {
    std::uint32_t h = static_cast<std::uint32_t>(x) * 374761393u
                    + static_cast<std::uint32_t>(y) * 668265263u
                    + static_cast<std::uint32_t>(z) * 2147483647u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= (h >> 16);
    return static_cast<float>(h & 0x00FFFFFFu) * (1.0f / 16777216.0f);
}

// Trilineare Value-Noise mit Smoothstep-Fade; `grad` erhaelt ∂n/∂x exakt
// (Ableitung des Fade-Polynoms und der Interpolation ueber die Produktregel).
inline float valueNoiseGrad(glm::vec3 x, glm::vec3& grad) {
    glm::vec3 i = glm::floor(x);
    glm::vec3 f = x - i;
    glm::vec3 u = f * f * (3.0f - 2.0f * f);
    glm::vec3 du = 6.0f * f * (1.0f - f);

    int ix = static_cast<int>(i.x), iy = static_cast<int>(i.y), iz = static_cast<int>(i.z);
    float c000 = hash3i(ix,     iy,     iz);
    float c100 = hash3i(ix + 1, iy,     iz);
    float c010 = hash3i(ix,     iy + 1, iz);
    float c110 = hash3i(ix + 1, iy + 1, iz);
    float c001 = hash3i(ix,     iy,     iz + 1);
    float c101 = hash3i(ix + 1, iy,     iz + 1);
    float c011 = hash3i(ix,     iy + 1, iz + 1);
    float c111 = hash3i(ix + 1, iy + 1, iz + 1);

    float x00 = c000 + (c100 - c000) * u.x;
    float x10 = c010 + (c110 - c010) * u.x;
    float x01 = c001 + (c101 - c001) * u.x;
    float x11 = c011 + (c111 - c011) * u.x;

    float y0 = x00 + (x10 - x00) * u.y;
    float y1 = x01 + (x11 - x01) * u.y;

    float v = y0 + (y1 - y0) * u.z;

    // ∂v/∂u.x: ableiten des Interpolationsbaums entlang x
    float dx0 = (c100 - c000) + ((c110 - c010) - (c100 - c000)) * u.y;
    float dx1 = (c101 - c001) + ((c111 - c011) - (c101 - c001)) * u.y;
    float dvx = (dx0 + (dx1 - dx0) * u.z) * du.x;
    // ∂v/∂u.y
    float dvy = ((x10 - x00) + ((x11 - x01) - (x10 - x00)) * u.z) * du.y;
    // ∂v/∂u.z
    float dvz = (y1 - y0) * du.z;

    grad = glm::vec3(dvx, dvy, dvz);
    return v;
}

// fbm aus `octaves` Oktaven, auf [0,1] normiert; `grad` = ∇fbm.
inline float fbmGrad(glm::vec3 x, int octaves, glm::vec3& grad) {
    float sum = 0.0f, norm = 0.0f, amp = 0.5f, freq = 1.0f;
    grad = glm::vec3(0.0f);
    for (int o = 0; o < octaves; ++o) {
        glm::vec3 g;
        sum += amp * valueNoiseGrad(x * freq, g);
        grad += amp * freq * g;
        norm += amp;
        amp *= 0.5f;
        freq *= 2.0f;
    }
    grad /= norm;
    return sum / norm;
}

} // namespace

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

// ---------- Kugel minus versetzte Kugel (CSG-Differenz, smooth) ----------

SphereMinusSphereSDF::SphereMinusSphereSDF(glm::vec3 center, float radius, float cutRadius, float offset, float smoothK)
    : m_center(center), m_radius(radius), m_cutRadius(cutRadius),
      m_offset(offset), m_k(std::max(smoothK, 1e-4f)), m_cutCenter(center + glm::vec3(offset, 0.0f, 0.0f)) {
    computeProfile();
}

// Weiche CSG-Differenz = Smooth-Max über (φ_A, −φ_B). Quílez-Polynom:
//     h  = clamp(0.5 + 0.5·(a − b)/k, 0, 1)   a = φ_A, b = −φ_B
//     φ  = b + h·(a − b) + k·h·(1 − h)        (+k statt −k = Smooth-MAX)
//     ∇φ = h·∇a + (1 − h)·∇b                  (außerhalb des Blends |∇φ|=1)
// Das Plus vor k·h(1−h) ist entscheidend: es macht aus einem Quílez-Smooth-Min
// einen Smooth-Max (SmoothMax(a,b) = −SmoothMin(−a,−b)), also hier die weiche
// Extremumswahl max(φ_A, −φ_B). Die Gratkante wird zum gerundeten Fillet, das
// die Fan-Triangulation schließen kann (≈116° → weiche U-Form).
SDFSample SphereMinusSphereSDF::sample(glm::vec3 p) const {
    glm::vec3 vA = p - m_center;
    glm::vec3 vB = p - m_cutCenter;
    float dA = glm::length(vA);
    float dB = glm::length(vB);
    float a = dA - m_radius;
    float b = -(dB - m_cutRadius);   // −φ_B

    glm::vec3 nA = (dA < 1e-6f) ? glm::vec3(0.0f, 1.0f, 0.0f) : vA / dA;
    glm::vec3 nB = (dB < 1e-6f) ? glm::vec3(0.0f, 1.0f, 0.0f) : vB / dB;
    glm::vec3 gb = -nB;              // ∇(−φ_B)

    float h = glm::clamp(0.5f + 0.5f * (a - b) / m_k, 0.0f, 1.0f);
    float phi = b + h * (a - b) + m_k * h * (1.0f - h);
    glm::vec3 grad = h * nA + (1.0f - h) * gb;
    return { phi, grad };
}

glm::vec3 SphereMinusSphereSDF::boundsMin() const {
    return m_boundsMin;
}

glm::vec3 SphereMinusSphereSDF::boundsMax() const {
    return m_boundsMax;
}

float SphereMinusSphereSDF::surfaceArea() const {
    return m_area;
}

// Rotationsintegral A = 2π·∫r·ds über die Null-Kontur von φ(x, r, 0) im
// (x, r)-Querschnitt (r = Abstand von der x-Achse). Die Kontur wird per
// Marching Squares auf einem regulären Gitter extrahiert. Anders als die
// Metaball-Bisektion (ein Profilpunkt pro x) erfasst sie den HOLLEN Querschnitt
// der CSG-Ausnehmung: im Überlappungsbereich existieren zwei Wand-Äste, die über
// die gerundete Gratkante verbunden sind — die Kontur schließt beides mit ein.
void SphereMinusSphereSDF::computeProfile() {
    const float rMax = m_radius + m_k + 0.1f;
    const float xLo = -(rMax + 1e-3f);
    const float xHi = m_offset + m_cutRadius + m_k + 0.1f;
    const int nx = 320, nr = 160;
    const float dx = (xHi - xLo) / nx;
    const float dr = rMax / nr;

    // Gitterwerte von φ(x, r, 0)
    std::vector<float> phiGrid(static_cast<size_t>((nx + 1) * (nr + 1)));
    auto at = [&](int ix, int ir) -> float {
        return phiGrid[static_cast<size_t>(ir) * (nx + 1) + ix];
    };
    for (int ir = 0; ir <= nr; ++ir) {
        float r = ir * dr;
        for (int ix = 0; ix <= nx; ++ix) {
            float x = xLo + ix * dx;
            phiGrid[static_cast<size_t>(ir) * (nx + 1) + ix] =
                sample({x, r, 0.0f}).distance;
        }
    }

    auto lerpX = [&](int ix, float t) -> float { return xLo + (ix + t) * dx; };
    auto lerpR = [&](int ir, float t) -> float { return (ir + t) * dr; };

    float area = 0.0f;
    float minX = xHi, maxX = xLo, maxR = 0.0f;
    auto addSegment = [&](float x1, float r1, float x2, float r2) {
        float ds = std::hypot(x2 - x1, r2 - r1);
        area += glm::pi<float>() * (r1 + r2) * ds;   // 2π·r̄·ds
        minX = std::min(minX, std::min(x1, x2));
        maxX = std::max(maxX, std::max(x1, x2));
        maxR = std::max(maxR, std::max(r1, r2));
    };

    // Marching Squares: Jede Gitterzelle mit Vorzeichenwechsel liefert genau ein
    // Kontursegment (Kantenkreuzungen linear interpoliert). Nimmt man beide
    // Verbindungen der jeweiligen Diagonalpaarung im Ambigue-Fall nicht
    // auf, aber die CSG-Kontur ist hier signaturregulär (keine Sattelpunkte).
    for (int ir = 0; ir < nr; ++ir) {
        for (int ix = 0; ix < nx; ++ix) {
            float v00 = at(ix, ir);
            float v10 = at(ix + 1, ir);
            float v11 = at(ix + 1, ir + 1);
            float v01 = at(ix, ir + 1);
            int code = (v00 >= 0.0f ? 1 : 0) | (v10 >= 0.0f ? 2 : 0)
                     | (v11 >= 0.0f ? 4 : 0) | (v01 >= 0.0f ? 8 : 0);
            if (code == 0 || code == 15) continue;

            struct P { float x, r; };
            std::array<P, 4> pts;
            int n = 0;
            // Unterkante (v00→v10)
            if ((v00 >= 0.0f) != (v10 >= 0.0f)) {
                float t = v00 / (v00 - v10);
                pts[n++] = { lerpX(ix, t), ir * dr };
            }
            // Rechte Kante (v10→v11)
            if ((v10 >= 0.0f) != (v11 >= 0.0f)) {
                float t = v10 / (v10 - v11);
                pts[n++] = { (ix + 1) * dx + xLo, lerpR(ir, t) };
            }
            // Oberkante (v11→v01)
            if ((v01 >= 0.0f) != (v11 >= 0.0f)) {
                float t = v01 / (v01 - v11);
                pts[n++] = { lerpX(ix, t), (ir + 1) * dr };
            }
            // Linke Kante (v01→v00)
            if ((v00 >= 0.0f) != (v01 >= 0.0f)) {
                float t = v00 / (v00 - v01);
                pts[n++] = { xLo + ix * dx, lerpR(ir, t) };
            }
            if (n == 2) addSegment(pts[0].x, pts[0].r, pts[1].x, pts[1].r);
        }
    }

    m_area = area;
    float halfR = std::max(maxR, m_radius);
    m_boundsMin = m_center - glm::vec3(std::fabs(minX), halfR, halfR);
    m_boundsMax = m_center + glm::vec3(std::fabs(maxX), halfR, halfR);
}

// ---------- Organischer Felsbrocken (fbm-displaced Ellipsoid) ----------

RockSDF::RockSDF(glm::vec3 center, float rx, float ry, float rz,
                 float amplitude, float frequency, int octaves, int seed)
    : m_center(center), m_radii(rx, ry, rz),
      m_amp(std::max(amplitude, 0.0f)),
      m_freq(std::max(frequency, 1e-3f)),
      m_octaves(std::max(octaves, 1)),
      m_seedOffset(static_cast<float>(seed) * glm::vec3(12.9898f, 78.233f, 37.719f)) {
    computeData();
}

SDFSample RockSDF::sample(glm::vec3 p) const {
    glm::vec3 d = p - m_center;
    glm::vec3 q = d / m_radii;
    float k = glm::length(q);
    float rmean = (m_radii.x + m_radii.y + m_radii.z) / 3.0f;

    // Grund-Ellipsoid als Potential (schnell; Displacement dominiert die Form).
    float phiEll = (k - 1.0f) * rmean;
    glm::vec3 gradEll(0.0f);
    if (k > 1e-6f) {
        glm::vec3 n(d.x / (m_radii.x * m_radii.x),
                    d.y / (m_radii.y * m_radii.y),
                    d.z / (m_radii.z * m_radii.z));
        gradEll = (rmean / k) * n;
    }

    glm::vec3 x = q * m_freq + m_seedOffset;
    glm::vec3 g;
    float f = fbmGrad(x, m_octaves, g);            // [0,1]

    float phi = phiEll - m_amp * (2.0f * f - 1.0f);
    glm::vec3 gradDisp(m_amp * 2.0f * m_freq * g.x / m_radii.x,
                       m_amp * 2.0f * m_freq * g.y / m_radii.y,
                       m_amp * 2.0f * m_freq * g.z / m_radii.z);
    glm::vec3 grad = gradEll - gradDisp;
    if (glm::length(grad) < 1e-8f) grad = glm::vec3(0.0f, 1.0f, 0.0f);
    return { phi, grad };
}

glm::vec3 RockSDF::boundsMin() const { return m_boundsMin; }
glm::vec3 RockSDF::boundsMax() const { return m_boundsMax; }
float RockSDF::surfaceArea() const { return m_area; }

// Sternfoermige Flaeche: jeder Strahl vom Zentrum trifft genau einmal. Die
// Bisektion loest φ(r·ω)=0; das Flaechenelement in Kugelkoordinaten ist
// r²/|n̂·ω̂| dω, das Integral wird ueber Fibonacci-Sphere-Richtungen
// (gleichverteilt) gequaestet.
void RockSDF::computeData() {
    glm::vec3 pad = m_radii + glm::vec3(m_amp);
    m_boundsMin = m_center - pad;
    m_boundsMax = m_center + pad;

    const int M = 16384;
    const float rMax = std::max(m_radii.x, std::max(m_radii.y, m_radii.z)) + 2.0f * m_amp + 0.2f;
    const float golden = glm::pi<float>() * (3.0f - std::sqrt(5.0f));
    double area = 0.0;

    for (int i = 0; i < M; ++i) {
        float z = 1.0f - 2.0f * (static_cast<float>(i) + 0.5f) / M;
        float rr = std::sqrt(std::max(0.0f, 1.0f - z * z));
        float th = golden * static_cast<float>(i);
        glm::vec3 w(std::cos(th) * rr, std::sin(th) * rr, z);

        float lo = 0.0f, hi = rMax;
        if (sample(m_center + w * hi).distance < 0.0f) continue;
        for (int it = 0; it < 40; ++it) {
            float mid = 0.5f * (lo + hi);
            if (sample(m_center + w * mid).distance < 0.0f) lo = mid; else hi = mid;
        }
        float r = 0.5f * (lo + hi);
        SDFSample s = sample(m_center + w * r);
        float gl = glm::length(s.gradient);
        if (gl < 1e-6f) continue;
        float cosA = std::fabs(glm::dot(s.gradient / gl, w));
        if (cosA < 1e-3f) continue;
        area += static_cast<double>(r * r) / cosA;
    }
    m_area = static_cast<float>(area * (4.0 * glm::pi<double>() / M));
}

// ---------- Organischer Fels-Torus (fbm-displaced Torus) ----------

RockTorusSDF::RockTorusSDF(glm::vec3 center, float majorRadius, float minorRadius,
                           float amplitude, float frequency, int octaves, int seed)
    : m_center(center), m_major(majorRadius),
      m_minor(std::max(minorRadius, 1e-4f)),
      // Amplitude begrenzen, damit die Lochmitte (φ0 = R − r) offen bleibt:
      // |amp·D| ≤ amp, also amp < 0.9·(R − r) erhaelt das Loch.
      m_amp(std::min(std::max(amplitude, 0.0f), 0.9f * std::max(majorRadius - m_minor, 0.0f))),
      m_freq(std::max(frequency, 1e-3f)),
      m_octaves(std::max(octaves, 1)),
      m_seedOffset(static_cast<float>(seed) * glm::vec3(12.9898f, 78.233f, 37.719f)) {
    computeData();
}

// Displacement-Feld D ∈ [−1,1] plus analytischer Gradient ∇D am Ring-Offset q.
// Koordinaten: volumetrisch-isotrope 3D-Abbildung λ = q/m_major (wie beim
// Felsbrocken). Im Gegensatz zur frueheren 2D-Tube-Abbildung λ=(x/s, y/R) gibt
// es keinen Spiegelsymmetrie-Defekt (D(x,y,z) != D(x,y,−z) im Allgemeinen) und
// keine azimutale Buendelung: alle drei Raumachsen deformieren gleichmaessig.
void RockTorusSDF::evaluateDisplacement(const glm::vec3& q, float& D, glm::vec3& gradD) const {
    if (m_amp <= 0.0f) { D = 0.0f; gradD = glm::vec3(0.0f); return; }
    glm::vec3 lam = q / m_major;

    glm::vec3 g;
    float f = fbmGrad(lam * m_freq + m_seedOffset, m_octaves, g);   // [0,1], g = ∂f/∂u
    D = 2.0f * f - 1.0f;

    // ∂D/∂q: dλ/dq = I/m_major, u = λ·freq  =>  gradD = 2·freq·g/m_major.
    gradD = m_freq * 2.0f * (g / m_major);
}

SDFSample RockTorusSDF::sample(glm::vec3 p) const {
    glm::vec3 q = p - m_center;
    float x = q.x, y = q.y, z = q.z;
    float r = std::sqrt(x * x + z * z);

    // Basis-Torus exakt (wie TorusSDF): |(r − R, y)| − r_minor.
    glm::vec3 normal(1.0f, 0.0f, 0.0f);
    float phi0;
    if (r < 1e-6f) {
        phi0 = m_major - m_minor;
    } else {
        glm::vec2 gv(r - m_major, y);
        float lenG = std::sqrt(gv.x * gv.x + gv.y * gv.y);
        float lg = std::max(lenG, 1e-6f);
        phi0 = lenG - m_minor;
        normal = { gv.x * x / (r * lg), gv.y / lg, gv.x * z / (r * lg) };
    }

    float D;
    glm::vec3 gradD;
    evaluateDisplacement(q, D, gradD);

    float phi = phi0 - m_amp * D;
    glm::vec3 grad = normal - m_amp * gradD;
    if (glm::length(grad) < 1e-8f) grad = glm::vec3(0.0f, 1.0f, 0.0f);
    return { phi, grad };
}

glm::vec3 RockTorusSDF::boundsMin() const { return m_boundsMin; }
glm::vec3 RockTorusSDF::boundsMax() const { return m_boundsMax; }
float RockTorusSDF::surfaceArea() const { return m_area; }

// Numerisches Flaechenintegral: parametrisiere die Tube ueber Ringwinkel α und
// Rohrwinkel β und integriere |∂p/∂α × ∂p/∂β|. Das Displacement wird an der
// Basis-Tube ausgewertet (Ein-Zug-Naeherung), damit die Summe wohldefiniert ist.
void RockTorusSDF::computeData() {
    float ring = m_major + m_minor + m_amp;
    m_boundsMin = m_center - glm::vec3(ring, m_minor + m_amp, ring);
    m_boundsMax = m_center + glm::vec3(ring, m_minor + m_amp, ring);

    const int N = 96;
    auto surfacePoint = [&](float alpha, float beta) {
        glm::vec3 u(std::cos(alpha), 0.0f, std::sin(alpha));
        glm::vec3 v(0.0f, 1.0f, 0.0f);
        glm::vec3 c = m_major * u;
        glm::vec3 dir = std::cos(beta) * u + std::sin(beta) * v;
        glm::vec3 qb = c + m_minor * dir;
        float D;
        glm::vec3 gd;
        evaluateDisplacement(qb, D, gd);
        float rho = std::max(m_minor + m_amp * D, 0.05f);
        return c + rho * dir;
    };

    double total = 0.0;
    for (int i = 0; i < N; ++i) {
        double a0 = 2.0 * glm::pi<double>() * i / N;
        double a1 = 2.0 * glm::pi<double>() * (i + 1) / N;
        for (int j = 0; j < N; ++j) {
            double b0 = 2.0 * glm::pi<double>() * j / N;
            double b1 = 2.0 * glm::pi<double>() * (j + 1) / N;
            glm::vec3 p00 = surfacePoint(static_cast<float>(a0), static_cast<float>(b0));
            glm::vec3 p10 = surfacePoint(static_cast<float>(a1), static_cast<float>(b0));
            glm::vec3 p01 = surfacePoint(static_cast<float>(a0), static_cast<float>(b1));
            glm::vec3 e1 = p10 - p00, e2 = p01 - p00;
            total += glm::length(glm::cross(e1, e2));
        }
    }
    m_area = static_cast<float>(total);
}
